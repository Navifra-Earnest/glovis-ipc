// nurirobot.hpp — 누리로봇 스마트모터 RS485 드라이버 (SA/SB/SC 계열)
//
// 비동기 설계: 제어 명령은 응답이 없어 즉시 반환하고, 피드백만 호출자가 poll() 한다.
// RS485는 반이중이라 물리적 동시 통신이 불가능하므로 버스 스레드 대신 라운드로빈으로 돈다.
// 115200에서 축당 왕복 ~1.4ms → 4축 5.6ms. ROS 타이머 콜백에서 그대로 쓸 수 있다.
//
//   navi::nuri::Bus bus("/dev/ttyS2", 9600, "gpiochip3", 3);
//   std::vector<navi::nuri::Motor> axes;
//   for (uint8_t id : {0, 1, 2, 3}) axes.emplace_back(bus, id, 16.0);
//
//   axes[0].setSpeed(10.0, 1.0);               // 논블로킹
//   ... 타이머 콜백마다:
//   for (auto& m : axes) m.poll();             // 축당 ~1.4ms
//   auto fb = axes[0].last();
//
// 하드웨어: Radxa_Rock5A_Hat_V1  U5=THVD1400
//   헤더8 TX  헤더10 RX  헤더12 DE(=RE#)  J3: 1=A 2=GND 3=B
//
// 프로토콜: 누리로봇_RS485통신프로토콜_V1.0.pdf 부록 SA-RS485_V1.0.2 (p.23~26)
//   본문 표(p.7,10)는 MC 계열 기준이라 설정·요청 Mode가 다르다. 부록이 우선.
//
// 링크: -lgpiod
#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <gpiod.h>
#include <termios.h>
#include <unistd.h>

namespace navi::nuri {

using Clock = std::chrono::steady_clock;
using Ms = std::chrono::milliseconds;

// ── 프로토콜 상수 (SA 계열) ────────────────────────────────────────
enum : uint8_t {
    kMdPosSpeed  = 0x01,  // 위치·속도제어    방향(1) 위치(2) 속도(2)
    kMdAccPos    = 0x02,  // 가감속 위치제어  방향(1) 위치(2) 도달시간(1)
    kMdAccSpeed  = 0x03,  // 가감속 속도제어  방향(1) 속도(2) 도달시간(1)
    kMdSetRatio  = 0x09,  // 외부 감속비 설정 (SA 계열: MC는 0x0B)
    kMdSetOnOff  = 0x0A,  // 제어 On/Off      (SA 계열: MC는 0x0C)
    kMdSetPosMode= 0x0B,  // 위치제어 모드    (SA 계열: MC는 0x0D)
    kMdResetPos  = 0x0C,  // 위치 초기화
    kMdOpenLoop  = 0x11,  // Open-loop 구동   방향(1) 듀티(2, 0.01%)

    kReqPing     = 0xA0, kFbPing    = 0xD0,
    kReqPos      = 0xA1, kFbPos     = 0xD1,
    kReqSpeed    = 0xA2, kFbSpeed   = 0xD2,
    kReqRatio    = 0xA6, kFbRatio   = 0xD6,
    kReqOnOff    = 0xA7, kFbOnOff   = 0xD7,
    kReqFirmware = 0xCD, kFbFirmware= 0xFD,
};

inline constexpr uint8_t kBroadcast = 0xFF;

// ~((ID + DataSize + Mode + Values) & 0xFF) — Header와 CS 자신은 제외
inline uint8_t checksum(uint8_t id, uint8_t size, uint8_t mode,
                        const uint8_t* v = nullptr, size_t n = 0) {
    unsigned s = id + size + mode;
    for (size_t i = 0; i < n; ++i) s += v[i];
    return static_cast<uint8_t>(~s);
}

// DataSize = CS 이후 전체 바이트 수 = 1(CS) + 1(Mode) + n
inline std::vector<uint8_t> frame(uint8_t id, uint8_t mode,
                                  const uint8_t* v = nullptr, size_t n = 0) {
    const auto size = static_cast<uint8_t>(2 + n);
    std::vector<uint8_t> p;
    p.reserve(6 + n);
    p = {0xFF, 0xFE, id, size, checksum(id, size, mode, v, n), mode};
    for (size_t i = 0; i < n; ++i) p.push_back(v[i]);   // insert는 GCC12에서 오탐 경고
    return p;
}

inline void put16(std::vector<uint8_t>& v, uint16_t x) {   // 빅엔디안
    v.push_back(static_cast<uint8_t>(x >> 8));
    v.push_back(static_cast<uint8_t>(x & 0xFF));
}
inline uint16_t get16(const uint8_t* p) {
    return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

struct Response {
    uint8_t id = 0, mode = 0;
    std::array<uint8_t, 8> values{};
    size_t n = 0;
};

// ── 버스 ───────────────────────────────────────────────────────────
class Bus {
public:
    // guard_us: 전송 완료 후 DE를 내리기까지의 여유. 실측 근거는 send() 주석 참고.
    Bus(const std::string& port, int baud = 9600,
        const char* de_chip = "gpiochip3", unsigned de_line = 3,
        double guard_us = 200.0)
        : baud_(baud), guard_(guard_us / 1e6) {
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) throw std::runtime_error("포트 열기 실패: " + port);
        configure(baud);

        chip_ = gpiod_chip_open_by_name(de_chip);
        if (!chip_) { ::close(fd_); throw std::runtime_error("gpiod_chip_open 실패"); }
        de_ = gpiod_chip_get_line(chip_, de_line);
        if (!de_ || gpiod_line_request_output(de_, "nuri-de", 0)) {
            gpiod_chip_close(chip_); ::close(fd_);
            throw std::runtime_error("DE 라인 요청 실패");
        }
    }

    ~Bus() {
        if (de_) { gpiod_line_set_value(de_, 0); gpiod_line_release(de_); }
        if (chip_) gpiod_chip_close(chip_);
        if (fd_ >= 0) ::close(fd_);
    }
    Bus(const Bus&) = delete;
    Bus& operator=(const Bus&) = delete;

    // 제어·설정 명령은 응답이 없다. 송신만 하고 즉시 반환.
    //
    // DE를 내리는 시점이 이 드라이버의 핵심이다 (실측):
    //   tcdrain()은 실제 전송 완료보다 3~5ms 늦게 리턴 → 모터 응답(지연 100µs) 앞부분 소실
    //   TIOCOUTQ==0 은 FIFO로 넘어간 시점일 뿐   → 4.4ms 일러서 송신이 잘림
    // 그래서 둘 다 쓰지 않고 전송 시작 시각 + 이론 전송시간(10비트/바이트)으로 계산한다.
    // 9600에서 guard 100~500µs가 안전 구간, 800µs부터 응답 첫 바이트가 잘린다.
    void send(const std::vector<uint8_t>& pkt) {
        // 문서 7.2: 연속 요청은 10ms 이상 간격. 붙여 보내면 응답을 흘린다
        // (실측: Ping 직후 바로 감속비를 물으면 무응답).
        const auto since = Clock::now() - last_tx_;
        if (since < kMinGap) std::this_thread::sleep_for(kMinGap - since);

        flushInput();
        gpiod_line_set_value(de_, 1);
        const auto t0 = Clock::now();
        ::write(fd_, pkt.data(), pkt.size());
        const auto end = t0 + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(pkt.size() * 10.0 / baud_ + guard_));
        while (Clock::now() < end) {}          // busy-wait: sleep은 이 정밀도가 안 나온다
        gpiod_line_set_value(de_, 0);
        last_tx_ = Clock::now();
    }

    // 요청 → 응답. 성공하면 true. retry는 잡음으로 한 번 놓친 경우를 위한 것으로,
    // 응답이 아예 없는 장치를 기다리며 늘어지지 않도록 기본 1회만 재시도한다.
    bool request(uint8_t id, uint8_t req_mode, uint8_t expect_mode, Response& out,
                 Ms deadline = Ms(400), int retry = 1) {
        for (int i = 0; i <= retry; ++i) {
            send(frame(id, req_mode));
            if (readFrame(expect_mode, out, deadline)) return true;
        }
        return false;
    }

private:
    void configure(int baud) {
        termios t{};
        if (tcgetattr(fd_, &t)) throw std::runtime_error("tcgetattr 실패");
        cfmakeraw(&t);
        const speed_t sp = toSpeed(baud);
        cfsetispeed(&t, sp);
        cfsetospeed(&t, sp);
        t.c_cflag |= (CLOCAL | CREAD);
        t.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);   // 8N1, 흐름제어 없음
        t.c_cc[VMIN] = 0;
        t.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &t)) throw std::runtime_error("tcsetattr 실패");
    }

    static speed_t toSpeed(int baud) {
        switch (baud) {
            case 9600:   return B9600;
            case 19200:  return B19200;
            case 38400:  return B38400;
            case 57600:  return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            case 460800: return B460800;
            case 500000: return B500000;
            case 1000000:return B1000000;
            default: throw std::runtime_error("지원하지 않는 보레이트: " + std::to_string(baud));
        }
    }

    void flushInput() {
        uint8_t junk[64];
        while (::read(fd_, junk, sizeof junk) > 0) {}
    }

    // 헤더 0xFFFE 탐색 → DataSize로 프레임 완성 → 체크섬 검증
    bool readFrame(uint8_t expect_mode, Response& out, Ms deadline) {
        std::vector<uint8_t> buf;
        const auto end = Clock::now() + deadline;
        while (Clock::now() < end) {
            uint8_t chunk[64];
            const auto n = ::read(fd_, chunk, sizeof chunk);
            if (n > 0) buf.insert(buf.end(), chunk, chunk + n);
            else std::this_thread::sleep_for(Ms(1));

            for (size_t i = 0; i + 3 < buf.size(); ++i) {
                if (buf[i] != 0xFF || buf[i + 1] != 0xFE) continue;
                const uint8_t id = buf[i + 2], size = buf[i + 3];
                const size_t total = i + 4 + size;
                if (size < 2 || size > 10) break;           // 깨진 길이
                if (buf.size() < total) break;              // 더 받아야 함
                const uint8_t cs = buf[i + 4], mode = buf[i + 5];
                const size_t vn = size - 2;
                const uint8_t* v = buf.data() + i + 6;
                // 펌웨어 버전 응답(0xFD)만 실측상 체크섬이 어긋난다.
                // 모터가 버전 바이트를 합에서 빼고 계산하는 듯 — 그 경우만 예외로 통과시킨다.
                const bool ok = cs == checksum(id, size, mode, v, vn)
                             || (mode == kFbFirmware && cs == checksum(id, size, mode));
                if (!ok || mode != expect_mode) break;
                out.id = id; out.mode = mode; out.n = vn;
                std::memcpy(out.values.data(), v, std::min(vn, out.values.size()));
                return true;
            }
        }
        return false;
    }

    static constexpr Ms kMinGap{12};   // 문서 요구 10ms + 여유

    int fd_ = -1, baud_;
    double guard_;
    gpiod_chip* chip_ = nullptr;
    gpiod_line* de_ = nullptr;
    Clock::time_point last_tx_{};
};

// ── 축 하나 ────────────────────────────────────────────────────────

struct Feedback {
    bool cw = false;
    double degree = 0.0;    // 0.01° 단위 원본 → 도
    double rpm = 0.0;
    // ⚠ 전류는 평균이 아니라 순간(피크) 샘플이다. 무부하 정속에서도 0.1~25.4A로 튀고
    //   0xFE(25.4A = 필드 최대) 포화가 수시로 나온다. 임계 판정에 그대로 쓰지 말 것.
    double amp = 0.0;
    Clock::time_point stamp{};
    bool valid = false;
};

class Motor {
public:
    // expected_ratio: 외부 감속비. 생성 시 모터에서 읽어 다르면 던진다.
    //   4축에서 축마다 EEPROM 설정이 다르면 한 축만 16배 어긋난 채 조용히 돌아간다.
    //   출고 기본값은 1.0이므로 새 모터는 반드시 먼저 설정해야 한다 (nuri_ping.py --set-ratio).
    //   0을 주면 검사를 건너뛴다.
    Motor(Bus& bus, uint8_t id, double expected_ratio = 16.0)
        : bus_(bus), id_(id) {
        Response r;
        if (!bus_.request(id_, kReqPing, kFbPing, r))
            throw std::runtime_error("모터 응답 없음: ID " + std::to_string(id));
        if (expected_ratio > 0.0) {
            const double got = readRatio();
            if (std::abs(got - expected_ratio) > 0.05)
                throw std::runtime_error(
                    "ID " + std::to_string(id) + " 외부 감속비 불일치: 모터 "
                    + std::to_string(got) + " vs 기대 " + std::to_string(expected_ratio)
                    + " (nuri_ping.py --set-ratio 로 맞출 것)");
            ratio_ = got;
        }
    }

    uint8_t id() const { return id_; }
    double ratio() const { return ratio_; }

    // ── 논블로킹 제어 ──────────────────────────────────────────
    // 감속비가 설정돼 있으면 속도·위치는 모두 출력축 기준이다.

    // 가감속 속도제어. ramp_s 동안 목표 속도까지 선형 가감속.
    void setSpeed(double rpm, double ramp_s = 1.0) {
        const bool cw = rpm < 0;
        auto v = std::vector<uint8_t>{static_cast<uint8_t>(cw ? 0x01 : 0x00)};
        put16(v, clamp16(std::fabs(rpm) * 10.0, 1));     // 0.1RPM, 0은 규격 밖
        v.push_back(clamp8(ramp_s * 10.0, 1));           // 0.1s
        sendWith(kMdAccSpeed, v);
        cmd_at_ = Clock::now();
    }

    // 정지 = Open-loop 듀티 0. 이게 기본 정지 경로다.
    //
    // 속도 0 명령은 규격 범위(0x0001~0xFFFD) 밖이라 제어기가 0 RPM을 유지하려 들고,
    // 실측에서 정지 상태로 6.5A가 계속 흐르며 가청 소음이 났다.
    // 문서 5.4: Open-loop는 "상태 유지를 위한 동작이 없다".
    void stop() {
        sendWith(kMdOpenLoop, {0x00, 0x00, 0x00});
        cmd_at_ = Clock::now();
    }

    // ponytail: 램프 정지(brake)는 넣지 않았다.
    // 속도 0은 규격 밖이라 그 자체가 여자 상태를 남긴다 — 실측에서 6.5A + 가청 소음.
    // 부드럽게 세우려면 setSpeed(작은 값)로 줄인 뒤 stop()으로 출력을 끊으면 된다.

    // Open-loop 직접 구동 (피드백 보정 없음). duty_pct 0~100
    void openLoop(bool cw, double duty_pct) {
        auto v = std::vector<uint8_t>{static_cast<uint8_t>(cw ? 0x01 : 0x00)};
        put16(v, clamp16(duty_pct * 100.0, 0, 10000));   // 0.01%
        sendWith(kMdOpenLoop, v);
        cmd_at_ = Clock::now();
    }

    // 제어 On/Off. ⚠ EEPROM 기록이라 상시 정지용으로 쓰지 말 것 (50~300ms 지연).
    // 장시간 세워둘 때만. 정지는 stop()을 쓴다.
    void enable(bool on) {
        sendWith(kMdSetOnOff, {static_cast<uint8_t>(on ? 0x00 : 0x01)});
        std::this_thread::sleep_for(Ms(400));
    }

    // ── 피드백 ─────────────────────────────────────────────────

    // 폴링 결과. "안정화 대기"와 "응답 없음"을 반드시 구분해야 한다 —
    // 둘을 뭉뚱그려 실패로 세면 명령 직후마다 축이 죽은 것으로 오판한다(실측으로 겪음).
    enum class Poll { Ok, Settling, NoResponse };

    // 위치 피드백 1회 갱신. 9600에서 ~15ms, 115200에서 실측 ~21ms 블로킹.
    //
    // 타임아웃을 짧게, 재시도 없이 간다(기본 request는 400ms×2회 = 최악 800ms).
    // 주기 폴링에서 죽은 축 하나가 800ms를 잡아먹으면 제어 루프 전체가 무너진다.
    // 어차피 실패 판정은 연속 횟수로 하므로 한 번쯤 놓쳐도 된다.
    Poll poll(Ms deadline = Ms(40)) {
        Response r;
        if (!bus_.request(id_, kReqPos, kFbPos, r, deadline, 0) || r.n < 6)
            return Poll::NoResponse;
        Feedback fb;
        fb.cw = r.values[0] != 0;
        fb.degree = get16(&r.values[1]) / 100.0;
        fb.rpm = get16(&r.values[3]) / 10.0;
        fb.amp = r.values[5] / 10.0;
        fb.stamp = Clock::now();
        fb.valid = true;
        // 명령 직후 첫 응답은 속도가 345.9 / 509.9 RPM 같은 쓰레기값으로 온다.
        // 위치도 함께 튀는 것으로 보여 이 구간 샘플은 버린다 —
        // 다만 **응답은 온 것**이므로 실패가 아니라 Settling 이다.
        if (fb.stamp - cmd_at_ < kCmdSettle) return Poll::Settling;
        last_ = fb;
        return Poll::Ok;
    }

    const Feedback& last() const { return last_; }
    bool fresh(Ms age = Ms(500)) const {
        return last_.valid && Clock::now() - last_.stamp < age;
    }

    double readRatio() {
        Response r;
        if (!bus_.request(id_, kReqRatio, kFbRatio, r) || r.n < 2)
            throw std::runtime_error("감속비 읽기 실패: ID " + std::to_string(id_));
        return get16(r.values.data()) / 10.0;
    }

    int readFirmware() {
        Response r;
        if (!bus_.request(id_, kReqFirmware, kFbFirmware, r) || r.n < 1) return -1;
        return r.values[0];
    }

    bool ping() {
        Response r;
        return bus_.request(id_, kReqPing, kFbPing, r);
    }

    // ponytail: 위치 제어(moveTo)는 넣지 않았다.
    // 절대 위치의 의미가 실측으로 확정되지 않았다 — 명령 직후 위치가 0.19°로 점프하는
    // 샘플이 대부분인데 한 번은 이전 값(125.28°)에서 이어졌고, CW 구동인데 위치는
    // 증가했다. 랩(655.33°) 처리와 방향 전환 시 기준점을 추측으로 짜면 첫 반전에서
    // 틀린 목표를 명령하게 된다. nuri_position_probe.py 로 판별한 뒤 추가할 것.

private:
    static constexpr Ms kCmdSettle{250};   // 명령 후 이 시간 동안의 피드백은 버린다

    void sendWith(uint8_t mode, const std::vector<uint8_t>& v) {
        bus_.send(frame(id_, mode, v.data(), v.size()));
    }

    static uint16_t clamp16(double x, uint16_t lo = 0, uint16_t hi = 0xFFFD) {
        return static_cast<uint16_t>(std::clamp(x, static_cast<double>(lo),
                                                static_cast<double>(hi)));
    }
    static uint8_t clamp8(double x, uint8_t lo = 0, uint8_t hi = 0xFF) {
        return static_cast<uint8_t>(std::clamp(x, static_cast<double>(lo),
                                               static_cast<double>(hi)));
    }

    Bus& bus_;
    uint8_t id_;
    double ratio_ = 0.0;
    Feedback last_{};
    Clock::time_point cmd_at_{};
};

}  // namespace navi::nuri
