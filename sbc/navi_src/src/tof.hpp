// tof.hpp — TF-Luna 거리센서. UART(HAT V1) · I2C(HAT V2) 양쪽을 지원한다.
//
//   navi::TofStream tof(cfg.tof);
//   auto r = tof.latest();
//   if (r.valid) printf("%d cm\n", r.dist_cm);
//
// 어느 쪽이든 별도 스레드가 읽고 최신 값 하나만 들고 있는다 — 상위는 동일하게 쓴다.
//
// ── UART (cfg.i2c = false) ───────────────────────────────────────
// 센서가 **자동으로 100Hz 로 뱉는다.** 명령이 필요 없어 RX 만 있으면 된다.
// 프레임 9바이트:
//   59 59 Dist_L Dist_H Str_L Str_H Temp_L Temp_H Checksum
//   체크섬 = 앞 8바이트 합의 하위 8비트
//
// ── I2C (cfg.i2c = true) ─────────────────────────────────────────
// 자동 송신이 없다. 호스트가 레지스터를 폴링한다 (기본 10ms = 100Hz).
//   0x00 Dist_L  0x01 Dist_H
//   0x02 Amp_L   0x03 Amp_H
//   0x04 Temp_L  0x05 Temp_H
// 레지스터 주소를 쓴 뒤 연속 읽기 한 번이면 6바이트가 다 온다.
// 체크섬이 없으므로 값 범위로 걸러야 한다 (UART 는 체크섬이 걸러줬다).
//
// 신호강도(strength)가 낮으면 거리값을 믿을 수 없다:
//   < 100      측정 실패 (반사가 약하다 — 검은 물체, 먼 거리)
//   = 65535    포화 (너무 가깝거나 반사가 강하다)
// 두 경우 모두 valid=false 로 준다. 상위에서 판단할 일이 아니다.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// I2C_SLAVE. <linux/i2c-dev.h> 가 없는 환경도 있어 상수를 직접 둔다
// (커널 ABI 라 값이 바뀌지 않는다).
#ifndef I2C_SLAVE
#define I2C_SLAVE 0x0703
#endif

#include "config.hpp"

namespace navi {

struct TofReading {
    int dist_cm = 0;
    int strength = 0;
    double temp_c = 0.0;
    bool valid = false;                  // 강도 판정까지 통과했는가
    std::chrono::steady_clock::time_point stamp{};
};

struct TofStatus {
    bool open = false;
    std::string port, error;
    unsigned long long frames = 0;       // 체크섬 통과
    unsigned long long bad = 0;          // 체크섬 실패 — 배선·보레이트 의심
    unsigned long long weak = 0;         // 강도 미달로 버린 것
    double fps = 0.0;
};

class TofStream {
public:
    explicit TofStream(const Config::Tof& cfg) : cfg_(cfg) {
        openPort();
        run_ = true;
        th_ = std::thread([this] { loop(); });
    }

    ~TofStream() {
        run_ = false;
        if (th_.joinable()) th_.join();
        if (fd_ >= 0) ::close(fd_);
    }
    TofStream(const TofStream&) = delete;
    TofStream& operator=(const TofStream&) = delete;

    bool ok() const {
        std::lock_guard<std::mutex> lk(mu_);
        return st_.open;
    }
    TofReading latest() const {
        std::lock_guard<std::mutex> lk(mu_);
        return last_;
    }
    TofStatus status() const {
        std::lock_guard<std::mutex> lk(mu_);
        return st_;
    }

private:
    // ── I2C (HAT V2) ────────────────────────────────────────────
    bool openI2c() {
        std::lock_guard<std::mutex> lk(mu_);
        st_.port = cfg_.i2c_dev;
        fd_ = ::open(cfg_.i2c_dev, O_RDWR);
        if (fd_ < 0) {
            st_.open = false;
            st_.error = std::string("I2C 열기 실패: ") + std::strerror(errno) +
                        " — 오버레이 rk3588-i2c8-m2 가 켜져 있는지 확인할 것";
            return false;
        }
        if (::ioctl(fd_, I2C_SLAVE, cfg_.i2c_addr) < 0) {
            st_.error = std::string("슬레이브 주소 설정 실패(0x") +
                        (cfg_.i2c_addr < 16 ? "0" : "") + "): " + std::strerror(errno);
            ::close(fd_); fd_ = -1; st_.open = false;
            return false;
        }
        st_.open = true;
        st_.error.clear();
        return true;
    }

    // 레지스터 0x00 부터 6바이트. 실패하면 false (배선·주소 문제).
    bool readI2c(TofReading& r) {
        const uint8_t reg = 0x00;
        if (::write(fd_, &reg, 1) != 1) return false;
        uint8_t b[6];
        if (::read(fd_, b, sizeof b) != static_cast<ssize_t>(sizeof b)) return false;

        const int dist = b[0] | (b[1] << 8);
        const int amp  = b[2] | (b[3] << 8);
        const int traw = b[4] | (b[5] << 8);

        // 체크섬이 없으니 범위로 거른다. TF-Luna 측정 범위는 최대 8m 다.
        // 0 이나 범위 밖은 통신이 깨진 것으로 본다 (전부 0xFF 로 읽히는 경우가 있다).
        if (dist < 0 || dist > 900) return false;

        r.dist_cm = dist;
        r.strength = amp;
        r.temp_c = traw / 8.0 - 256.0;      // 데이터시트 변환식
        r.stamp = std::chrono::steady_clock::now();
        r.valid = (amp >= cfg_.min_strength && amp != 65535);
        return true;
    }

    // ── UART (HAT V1) ───────────────────────────────────────────
    bool openPort() {
        if (cfg_.i2c) return openI2c();
        std::lock_guard<std::mutex> lk(mu_);
        st_.port = cfg_.port;
        fd_ = ::open(cfg_.port, O_RDONLY | O_NOCTTY);
        if (fd_ < 0) {
            st_.open = false;
            st_.error = std::string("포트 열기 실패: ") + std::strerror(errno);
            return false;
        }
        termios t{};
        if (::tcgetattr(fd_, &t) != 0) {
            st_.error = "tcgetattr 실패";
            ::close(fd_); fd_ = -1; st_.open = false;
            return false;
        }
        ::cfmakeraw(&t);
        const speed_t sp = (cfg_.baud == 9600) ? B9600 : B115200;
        ::cfsetispeed(&t, sp);
        ::cfsetospeed(&t, sp);
        t.c_cflag |= (CLOCAL | CREAD);
        t.c_cflag &= ~CRTSCTS;
        // 블로킹으로 두되 100ms 타임아웃 — run_ 을 주기적으로 확인해야 종료가 된다
        t.c_cc[VMIN] = 0;
        t.c_cc[VTIME] = 1;
        ::tcsetattr(fd_, TCSANOW, &t);
        ::tcflush(fd_, TCIFLUSH);
        st_.open = true;
        st_.error.clear();
        return true;
    }

    void loop() {
        using Clock = std::chrono::steady_clock;
        std::vector<uint8_t> buf;
        buf.reserve(256);
        uint8_t chunk[128];
        auto win = Clock::now();
        unsigned in_win = 0;
        int fails = 0;

        while (run_) {
            if (fd_ < 0) {
                std::this_thread::sleep_for(std::chrono::seconds(2));
                if (run_) openPort();
                continue;
            }

            // ── I2C: 폴링이라 흐름이 완전히 다르다. 여기서 갈라 처리하고 돌아간다.
            if (cfg_.i2c) {
                TofReading r;
                if (readI2c(r)) {
                    fails = 0;
                    std::lock_guard<std::mutex> lk(mu_);
                    ++st_.frames;
                    if (!r.valid) ++st_.weak;
                    last_ = r;
                    if (++in_win; Clock::now() - win >= std::chrono::seconds(1)) {
                        st_.fps = in_win / std::chrono::duration<double>(Clock::now() - win).count();
                        in_win = 0;
                        win = Clock::now();
                    }
                } else {
                    std::lock_guard<std::mutex> lk(mu_);
                    ++st_.bad;
                    // I2C 는 타임아웃이 짧아 실패가 빨리 쌓인다 — 기준을 넉넉히 잡는다
                    if (++fails >= kReopenAfter * 10) {
                        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
                        st_.open = false;
                        st_.error = "I2C 응답 없음 — 재연결 시도";
                        fails = 0;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.i2c_period_ms));
                continue;
            }

            const auto n = ::read(fd_, chunk, sizeof chunk);
            if (n <= 0) {
                // 100ms 타임아웃이 계속되면 센서가 빠진 것으로 본다.
                // TF-Luna 는 100Hz 라 정상이면 절대 이렇게 오래 조용하지 않다.
                if (++fails >= kReopenAfter) {
                    std::lock_guard<std::mutex> lk(mu_);
                    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
                    st_.open = false;
                    st_.error = "수신 없음 — 재연결 시도";
                    fails = 0;
                }
                continue;
            }
            fails = 0;
            buf.insert(buf.end(), chunk, chunk + n);

            // 프레임 경계를 찾아 소비한다. 헤더가 깨져 있으면 한 바이트씩 밀며 재동기한다.
            size_t i = 0;
            while (i + 9 <= buf.size()) {
                if (buf[i] != 0x59 || buf[i + 1] != 0x59) { ++i; continue; }
                unsigned sum = 0;
                for (int k = 0; k < 8; ++k) sum += buf[i + k];
                if ((sum & 0xFF) != buf[i + 8]) {
                    std::lock_guard<std::mutex> lk(mu_);
                    ++st_.bad;
                    ++i;
                    continue;
                }
                TofReading r;
                r.dist_cm  = buf[i + 2] | (buf[i + 3] << 8);
                r.strength = buf[i + 4] | (buf[i + 5] << 8);
                r.temp_c   = (buf[i + 6] | (buf[i + 7] << 8)) / 8.0 - 256.0;
                // 강도로 신뢰도를 가린다 — 상위가 매번 판단하게 두지 않는다
                r.valid = (r.strength >= cfg_.min_strength && r.strength != 65535);
                r.stamp = Clock::now();
                {
                    std::lock_guard<std::mutex> lk(mu_);
                    ++st_.frames;
                    if (!r.valid) ++st_.weak;
                    last_ = r;
                }
                ++in_win;
                i += 9;
            }
            if (i) buf.erase(buf.begin(), buf.begin() + i);
            // 동기가 안 맞아 쓰레기가 쌓이는 걸 막는다
            if (buf.size() > 128) buf.erase(buf.begin(), buf.end() - 32);

            if (Clock::now() - win >= std::chrono::seconds(1)) {
                const double sec = std::chrono::duration<double>(Clock::now() - win).count();
                std::lock_guard<std::mutex> lk(mu_);
                st_.fps = in_win / sec;
                in_win = 0;
                win = Clock::now();
            }
        }
    }

    static constexpr int kReopenAfter = 30;   // 100ms 타임아웃 × 30 ≈ 3초

    Config::Tof cfg_;
    int fd_ = -1;
    std::thread th_;
    std::atomic<bool> run_{false};

    mutable std::mutex mu_;
    TofReading last_{};
    TofStatus st_{};
};

}  // namespace navi
