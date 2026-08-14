// actuator.hpp — STSPIN948 액추에이터 (ROCK 3A / 5A 공용)
//
// 비동기 설계: start() 는 즉시 반환하고, 호출자가 자기 주기로 poll() 한다.
// ROS 타이머 콜백에서 그대로 쓸 수 있다.
//
//   navi::Actuator act(navi::Actuator::detect());
//   navi::HallCounter hall(...);
//   act.attachHall(&hall);
//
//   act.start(true, 40.0);                     // 논블로킹
//   ... 타이머 콜백마다:
//   auto st = act.poll();
//   if (st != navi::State::Running) { ... }    // 끝단·막힘·fault
//
// 하드웨어: Radxa_Rock5A_Hat_V1
//   헤더24 PWM1A  헤더26 PHA  헤더22 EN/nFAULT  헤더37 VA  헤더38/40 Hall
//
// ponytail: 보드 차이는 핀 번호뿐 — 추상화 계층 대신 표 하나
#pragma once

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace navi {

struct BoardConfig {
    const char* model;        // /sys/firmware/devicetree/base/model 부분일치
    const char* pwmchip;      // PWM1A
    int gpio_dir;             // PHA
    int gpio_fault;           // EN/nFAULT (읽기 전용)
    int adc_channel;          // VA
    // 홀 두 채널은 보드에 따라 **서로 다른 칩**에 놓인다 (5A: 핀35=chip4, 핀31=chip1).
    const char* hall_chip;    // 채널 A 의 칩
    unsigned hall_a;
    const char* hall_chip_b;  // 채널 B 의 칩 (같으면 hall_chip 과 동일하게)
    unsigned hall_b;
};

// HAT 리비전별 핀 배정.
//   V1: ToF UART(핀7·29), Hall 핀35·31        ← 현재 실물
//   V2: ToF I2C(핀3·5)+RDY(핀36), Hall 핀38·40
// 기본은 V1. 환경변수 NAVI_HAT_REV=2 또는 detect(2)로 V2 선택.
//
// 🔴 pwmchip 번호는 보드마다 다르다 — 이름이 아니라 **주소**로 확인할 것.
//    RK3588 의 PWM 베이스는 fd8b0000 부터 0x10 씩 올라간다:
//        fd8b0010 = PWM1 → 5A 에서 pwmchip0   ← ACC_PWM(핀24, PWM1_M2)
//        fd8b0030 = PWM3 → 5A 에서 pwmchip1
//    문서만 보고 "PWM1 이니까 pwmchip1" 로 적었다가 5A 실기에서 틀린 걸 확인했다
//    (2026-08-07, `write 실패: .../pwmchip1/pwm0/period`).
//    확인법:
//        for c in /sys/class/pwm/pwmchip*; do echo "$c $(readlink -f $c/device)"; done
inline constexpr BoardConfig kBoardsV1[] = {
    // GPIO4_D1 / GPIO0_C1 / VIN5 / Hall 핀35=gpiochip3:4, 핀31=gpiochip3:0
    {"ROCK3 Model A", "pwmchip1", 153, 17, 5, "gpiochip3", 4, "gpiochip3", 0},
    // GPIO1_A4 / GPIO1_B5 / VIN2 / Hall 핀35=gpiochip4:0, 핀31=gpiochip1:9  ← 칩이 다르다 (실측)
    {"ROCK 5A",       "pwmchip0",  36, 45, 2, "gpiochip4", 0, "gpiochip1", 9},
};

inline constexpr BoardConfig kBoardsV2[] = {
    // Hall 핀38=gpiochip3:6, 핀40=gpiochip3:5
    {"ROCK3 Model A", "pwmchip1", 153, 17, 5, "gpiochip3", 6, "gpiochip3", 5},
    // Hall 핀38=gpiochip4:5, 핀40=gpiochip4:9 (실측)
    {"ROCK 5A",       "pwmchip0",  36, 45, 2, "gpiochip4", 5, "gpiochip4", 9},
};

enum class State {
    Idle,         // 정지
    Running,      // 구동 중
    Reached,      // 목표 위치 도달 (moveTo)
    EndOfStroke,  // 홀 정지 + 전류 낮음 → 내장 리밋 스위치
    Blocked,      // 홀 정지 + 전류 높음 → 장애물
    Fault,        // EN/nFAULT LOW (U6 fault 또는 UVLO)
};

inline const char* toString(State s) {
    switch (s) {
        case State::Idle:        return "정지";
        case State::Running:     return "구동중";
        case State::Reached:     return "목표도달";
        case State::EndOfStroke: return "스트로크끝단";
        case State::Blocked:     return "막힘";
        case State::Fault:       return "FAULT";
    }
    return "?";
}

// 홀 카운터 인터페이스 (hall.hpp의 HallCounter가 만족)
struct IHall {
    virtual ~IHall() = default;
    virtual long position() const = 0;
    virtual long edges() const = 0;
    virtual void reset(long p) = 0;
    virtual std::chrono::milliseconds sinceLastEdge() const = 0;
};

class Actuator {
public:
    // R35 50mΩ x 증폭기 A_CL 10 → 0.5 V/A
    static constexpr double kVoltsPerAmp = 0.5;
    static constexpr double kAdcVref = 1.8;
    static constexpr int kAdcMax = 1023;
    static constexpr unsigned kPeriodNs = 50'000;   // 20kHz

    // ── 튜닝 파라미터 ──────────────────────────────────────────
    // 실측 기준: 정상 구동 0.3~0.6A, 듀티 20%에서 홀 18ms 간격.
    // 끝단은 내장 리밋 스위치가 회로를 끊어 전류가 0.05A로 떨어진다.
    double current_limit = 2.0;                    // 액추에이터 정격(MA5 2A)
    double blocked_current = 1.5;                  // 이 이상이면 장애물
    std::chrono::milliseconds hall_timeout{300};   // 홀 정지 판정
    std::chrono::milliseconds start_grace{1000};   // 기동 직후 유예
    double current_ema = 0.3;                      // 전류 평활 계수(0~1, 클수록 민감)

    explicit Actuator(const BoardConfig& cfg) : cfg_(cfg) {
        exportGpio(cfg_.gpio_dir, "out");
        exportGpio(cfg_.gpio_fault, "in");   // 오픈드레인 겸용 — 출력으로 쓰지 말 것
        setupPwm();
        i_avg_ = rawCurrent();
    }

    ~Actuator() { try { stop(); } catch (...) {} }
    Actuator(const Actuator&) = delete;
    Actuator& operator=(const Actuator&) = delete;

    void attachHall(IHall* h) { hall_ = h; }

    // ── 논블로킹 제어 ──────────────────────────────────────────

    // 방향: false = EXT(J8.1 +), true = RET(J8.2 +)
    void start(bool ret, double duty_pct) {
        if (duty_pct < 0.0 || duty_pct > 100.0) throw std::out_of_range("duty 0~100");
        if (hall_) hall_->reset(hall_->position());   // 위치는 유지, 타이머만 초기화
        write(gpioPath(cfg_.gpio_dir) / "value", ret ? "1" : "0");
        auto p = pwmPath();
        write(p / "duty_cycle",
              std::to_string(static_cast<unsigned>(kPeriodNs * duty_pct / 100.0)));
        write(p / "enable", "1");
        t0_ = std::chrono::steady_clock::now();
        target_active_ = false;
        state_ = State::Running;
    }

    // ── ① 절대 위치 이동 ──
    // 방향은 현재 위치와 target 차이로 자동 결정
    void moveTo(long target, double duty_pct) {
        if (!hall_) throw std::runtime_error("moveTo에는 홀 카운터가 필요하다");
        const long now = hall_->position();
        if (std::abs(target - now) <= tolerance) { state_ = State::Reached; return; }
        target_ = target;
        const bool ret = target < now;
        start(ret, duty_pct);
        target_active_ = true;
        dir_ret_ = ret;
    }

    // ── ② 상대 위치 이동 ──
    void moveBy(long delta, double duty_pct) {
        if (!hall_) throw std::runtime_error("moveBy에는 홀 카운터가 필요하다");
        moveTo(hall_->position() + delta, duty_pct);
    }

    // ── ③ 조그 ── 목표 없이 계속 구동. stop() 또는 끝단/막힘까지
    void jog(bool ret, double duty_pct) { start(ret, duty_pct); }

    // PWM=0 은 코스트가 아니라 브레이크(양쪽 LS on). 데이터시트 Table 9
    void stop() {
        auto p = pwmPath();
        if (std::filesystem::exists(p)) {
            write(p / "duty_cycle", "0");
            write(p / "enable", "0");
        }
        if (state_ == State::Running) state_ = State::Idle;
    }

    // 호출자 주기로 부른다. 상태를 갱신하고 반환한다.
    State poll() {
        if (state_ != State::Running) return state_;

        // 전류는 PWM 리플로 튀므로 평활한다
        i_avg_ += current_ema * (rawCurrent() - i_avg_);

        if (fault()) return finish(State::Fault);

        const auto elapsed = std::chrono::steady_clock::now() - t0_;

        if (target_active_ && hall_) {
            const long p = hall_->position();
            if ((dir_ret_ && p <= target_) || (!dir_ret_ && p >= target_))
                return finish(State::Reached);
        }

        if (elapsed > start_grace && hall_ && hall_->sinceLastEdge() > hall_timeout) {
            // 홀이 멈췄다. 전류로 끝단인지 막힘인지 가른다
            return finish(i_avg_ >= blocked_current ? State::Blocked
                                                    : State::EndOfStroke);
        }

        if (i_avg_ >= current_limit) return finish(State::Blocked);

        return State::Running;
    }

    State state() const { return state_; }
    bool running() const { return state_ == State::Running; }
    long position() const { return hall_ ? hall_->position() : 0; }
    double current() const { return i_avg_; }          // 평활값
    double rawCurrent() const {
        return std::stoi(read(adcPath())) * kAdcVref / kAdcMax / kVoltsPerAmp;
    }
    // EN/nFAULT: LOW = fault 또는 UVLO(VBAT 미인가)
    bool fault() const { return read(gpioPath(cfg_.gpio_fault) / "value") == "0"; }

    std::chrono::milliseconds elapsed() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0_);
    }

    // 편의용 블로킹 래퍼. 내부에서 poll()을 돌린다
    State runFor(bool ret, double duty, std::chrono::milliseconds dur,
                 std::chrono::milliseconds tick = std::chrono::milliseconds(10)) {
        start(ret, duty);
        while (poll() == State::Running) {
            if (elapsed() >= dur) { stop(); return State::Idle; }
            std::this_thread::sleep_for(tick);
        }
        stop();
        return state_;
    }

    // ── ④ 호밍 ── RET 끝까지 밀고 그 지점을 0으로 잡는다
    // 끝단은 액추에이터 내장 리밋 스위치가 멈춘다 (전류가 0.05A로 떨어짐)
    State home(double duty,
               std::chrono::milliseconds limit = std::chrono::seconds(40)) {
        const auto st = runFor(true, duty, limit);
        if (st == State::EndOfStroke && hall_) hall_->reset(0);
        return st;
    }

    // ── 위치 제어 ──────────────────────────────────────────────

    long strokeCounts() const { return stroke_; }
    bool calibrated() const { return stroke_ > 0; }

    // 양쪽 끝을 찍어 전체 스트로크를 잰다. RET 끝이 0, EXT 끝이 stroke_
    // 반환: 성공하면 스트로크 카운트, 실패하면 0
    long calibrate(double duty, std::chrono::milliseconds limit = std::chrono::seconds(40)) {
        if (!hall_) throw std::runtime_error("calibrate에는 홀 카운터가 필요하다");
        if (home(duty, limit) != State::EndOfStroke) return 0;          // RET 끝 = 0
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        if (runFor(false, duty, limit) != State::EndOfStroke) return 0;  // EXT 끝
        stroke_ = hall_->position();
        return stroke_;
    }

    // 0.0(RET 끝) ~ 1.0(EXT 끝) 비율로 이동
    void moveToRatio(double r, double duty) {
        if (!calibrated()) throw std::runtime_error("calibrate 먼저");
        if (r < 0.0) r = 0.0;
        if (r > 1.0) r = 1.0;
        moveTo(static_cast<long>(stroke_ * r), duty);
    }

    // 목표 도달 판정 허용 오차(카운트). 관성 오버슈트만큼은 어쩔 수 없다
    long tolerance = 2;

    // hat_rev: 1 또는 2. 0이면 환경변수 NAVI_HAT_REV를 보고, 없으면 V1
    static const BoardConfig& detect(int hat_rev = 0) {
        if (hat_rev == 0) {
            const char* e = std::getenv("NAVI_HAT_REV");
            hat_rev = (e && *e == '2') ? 2 : 1;
        }
        const auto model = read("/sys/firmware/devicetree/base/model");
        if (hat_rev == 2) {
            for (const auto& b : kBoardsV2)
                if (model.find(b.model) != std::string::npos) return b;
        } else {
            for (const auto& b : kBoardsV1)
                if (model.find(b.model) != std::string::npos) return b;
        }
        throw std::runtime_error("지원하지 않는 보드: " + model
                                 + " (HAT rev " + std::to_string(hat_rev) + ")");
    }

    // 현재 선택된 HAT 리비전 (로그용)
    static int hatRev(int hat_rev = 0) {
        if (hat_rev) return hat_rev;
        const char* e = std::getenv("NAVI_HAT_REV");
        return (e && *e == '2') ? 2 : 1;
    }

private:
    State finish(State s) { stop(); state_ = s; return s; }

    std::filesystem::path pwmPath() const {
        return std::filesystem::path("/sys/class/pwm") / cfg_.pwmchip / "pwm0";
    }
    std::filesystem::path adcPath() const {
        return "/sys/bus/iio/devices/iio:device0/in_voltage"
               + std::to_string(cfg_.adc_channel) + "_raw";
    }
    static std::filesystem::path gpioPath(int n) {
        return std::filesystem::path("/sys/class/gpio") / ("gpio" + std::to_string(n));
    }

    static std::string read(const std::filesystem::path& p) {
        std::ifstream f(p);
        if (!f) throw std::runtime_error("read 실패: " + p.string());
        std::string s;
        std::getline(f, s);
        while (!s.empty() && (s.back() == '\0' || s.back() == '\n')) s.pop_back();
        return s;
    }
    static void write(const std::filesystem::path& p, const std::string& v) {
        std::ofstream f(p);
        if (!f) throw std::runtime_error("write 실패: " + p.string());
        f << v;
        if (!f) throw std::runtime_error("write 오류: " + p.string() + " <- " + v);
    }
    static void exportGpio(int n, const char* dir) {
        if (!std::filesystem::exists(gpioPath(n))) {
            write("/sys/class/gpio/export", std::to_string(n));
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        write(gpioPath(n) / "direction", dir);
    }

    // RK 계열 PWM sysfs 제약 (실기 확인):
    //   1) period 가 0이면 enable/duty 쓰기가 EINVAL
    //   2) polarity 기본값이 inversed — 그대로 두면 duty 10%가 90%로 나간다
    //   3) polarity 는 period 설정 후, enable 전에만 바꿀 수 있다
    void setupPwm() {
        auto chip = std::filesystem::path("/sys/class/pwm") / cfg_.pwmchip;
        auto p = pwmPath();
        if (!std::filesystem::exists(p)) {
            write(chip / "export", "0");
            for (int i = 0; i < 50 && !std::filesystem::exists(p / "period"); ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        try { write(p / "enable", "0"); } catch (...) {}
        try { write(p / "duty_cycle", "0"); } catch (...) {}
        write(p / "period", std::to_string(kPeriodNs));
        if (read(p / "polarity") != "normal") write(p / "polarity", "normal");
        write(p / "duty_cycle", "0");
    }

    const BoardConfig& cfg_;
    IHall* hall_ = nullptr;
    State state_ = State::Idle;
    std::chrono::steady_clock::time_point t0_{};
    double i_avg_ = 0.0;
    long target_ = 0;
    long stroke_ = 0;          // calibrate()로 측정한 전체 스트로크
    bool target_active_ = false;
    bool dir_ret_ = false;
};

}  // namespace navi
