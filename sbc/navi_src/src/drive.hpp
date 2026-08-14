// drive.hpp — 구동 모터 4축 묶음.
//
// 두 층으로 나눠 뒀다:
//   ① setWheelRpm()  — 축별 속도. 차체 치수를 몰라도 지금 바로 쓸 수 있다
//   ② setBodyVelocity() / odometry() — 운동학. Config의 차체 치수를 채우면 열린다
//
// 운동학을 넣을 때 손볼 곳은 아래 bodyToWheels() / wheelsToBody() 두 함수뿐이다.
// 지금은 4륜 스키드스티어를 가정해 채워뒀고, 치수가 0이면 예외로 막는다.
// 메카넘이면 BodyVel에 vy를 추가하고 두 함수만 고치면 된다.
#pragma once

#include <array>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "config.hpp"
#include "nurirobot.hpp"

namespace navi {

// 차체 속도. 전진 +x, 반시계 +z (ROS REP-103 관례)
struct BodyVel {
    double vx = 0.0;   // m/s
    double wz = 0.0;   // rad/s
};

using WheelRpm = std::array<double, 4>;   // Config::wheels 순서 (FL FR RL RR)

// ── 운동학 ─────────────────────────────────────────────────────────
// 4륜 스키드스티어: 좌우 바퀴를 각각 한 덩어리로 보고 차동 구동처럼 다룬다.
//
//   v_left  = vx - wz * (track/2) * slip
//   v_right = vx + wz * (track/2) * slip
//   rpm = v / (2π r) * 60
//
// slip_factor는 실측 보정값이다. 스키드스티어는 바퀴가 옆으로 미끄러지며 회전하므로
// 이론값대로 명령하면 실제 회전이 덜 된다 — 제자리 회전을 시켜보고 맞춘다.
inline void requireBody(const Config& c, const char* what) {
    if (c.wheels.size() != 4)
        throw std::runtime_error(
            std::string(what) + ": 운동학은 4축이 다 올라와야 쓸 수 있다 (현재 "
            + std::to_string(c.wheels.size()) + "축) — 축별 제어는 setWheelRpm 사용");
    if (c.wheel_radius_m <= 0.0 || c.track_width_m <= 0.0)
        throw std::runtime_error(
            std::string(what) + ": 차체 치수가 없다 — config.hpp의 wheel_radius_m / "
            "track_width_m 을 실측해서 채울 것 (그 전에는 setWheelRpm 사용)");
}

inline WheelRpm bodyToWheels(const Config& c, BodyVel v) {
    requireBody(c, "setBodyVelocity");
    const double half = c.track_width_m / 2.0 * c.slip_factor;
    const double vl = v.vx - v.wz * half;
    const double vr = v.vx + v.wz * half;
    const double k = 60.0 / (2.0 * M_PI * c.wheel_radius_m);   // m/s → RPM
    return {vl * k, vr * k, vl * k, vr * k};                   // FL FR RL RR
}

inline BodyVel wheelsToBody(const Config& c, const WheelRpm& rpm) {
    requireBody(c, "odometry");
    const double k = 2.0 * M_PI * c.wheel_radius_m / 60.0;     // RPM → m/s
    const double vl = (rpm[0] + rpm[2]) / 2.0 * k;             // FL, RL
    const double vr = (rpm[1] + rpm[3]) / 2.0 * k;             // FR, RR
    const double half = c.track_width_m / 2.0 * c.slip_factor;
    return {(vl + vr) / 2.0, (vr - vl) / (2.0 * half)};
}

// ── 축 상태 ────────────────────────────────────────────────────────
struct WheelState {
    uint8_t id = 0;
    const char* label = "";
    double rpm = 0.0;          // 부호 적용된 값 (전진 +)
    double amp = 0.0;          // ⚠ 순간 피크 샘플이라 신뢰할 수 없다. 로그용
    double amp_median = 0.0;   // 최근 N개 중앙값 — 과전류 판정은 이걸로 한다
    int over_hits = 0;         // 연속 과전류 횟수
    double degree = 0.0;       // 위치 — 의미 미확정, 참고용
    bool alive = false;        // 최근 응답이 있었는가
    int consecutive_fail = 0;
    nuri::Clock::time_point stamp{};
};

class Drive {
public:
    // 모터가 하나라도 안 잡히면 던진다. 구동계는 부분 동작을 허용하지 않는다
    // (한 바퀴만 도는 4륜차는 위험하다).
    Drive(const Config& cfg, nuri::Bus& bus) : cfg_(cfg) {
        motors_.reserve(cfg_.wheels.size());
        for (size_t i = 0; i < cfg_.wheels.size(); ++i) {
            const auto& w = cfg_.wheels[i];
            try {
                motors_.emplace_back(bus, w.id, cfg_.gear_ratio);
            } catch (const std::exception& e) {
                throw std::runtime_error(std::string("휠 ") + w.label + " (ID "
                                         + std::to_string(w.id) + ") 초기화 실패: " + e.what());
            }
            state_[i].id = w.id;
            state_[i].label = w.label;
        }
    }

    // ── ① 축별 속도 — 차체 치수 없이도 쓸 수 있다 ──────────────
    // rpm은 출력축 기준, 전진이 +. Config::wheels[].sign으로 좌우 방향을 뒤집는다.
    void setWheelRpm(const WheelRpm& rpm, double ramp_s = 0.5) {
        for (size_t i = 0; i < motors_.size(); ++i) {
            const double v = clampRpm(rpm[i]) * cfg_.wheels[i].sign;
            if (std::fabs(v) < 0.05) motors_[i].stop();   // 0은 규격 밖 → 출력 차단으로 처리
            else motors_[i].setSpeed(v, ramp_s);
        }
        target_ = rpm;
        cmd_at_ = nuri::Clock::now();
    }

    void setWheelRpm(double all, double ramp_s = 0.5) {
        setWheelRpm(WheelRpm{all, all, all, all}, ramp_s);
    }

    // ── ② 운동학 — 차체 치수를 채우면 열린다 ────────────────────
    void setBodyVelocity(BodyVel v, double ramp_s = 0.5) {
        setWheelRpm(bodyToWheels(cfg_, v), ramp_s);
    }

    // 측정된 바퀴 속도에서 되짚은 차체 속도. 적분(위치 오도메트리)은 상위에서 한다 —
    // 여기서 누적하면 시간 기준이 tick 주기에 묶여 테스트가 어렵다.
    BodyVel measuredBody() const {
        WheelRpm m{};
        for (size_t i = 0; i < state_.size(); ++i) m[i] = state_[i].rpm;
        return wheelsToBody(cfg_, m);
    }

    // ── 정지 ───────────────────────────────────────────────────
    // Open-loop 듀티 0. 속도 0 명령은 규격 밖이라 여자 상태가 남는다(실측 6.5A + 소음).
    //
    // ⚠ 이건 **즉시 차단**이다. 관성으로만 서기 때문에 뚝 선다.
    //   평상시 정지는 아래 decelerate() 를 쓴다. 이 함수는 e-stop 처럼
    //   "지금 당장 끊어야 하는" 경우를 위한 것이다.
    void stop() {
        decel_active_ = false;
        for (auto& m : motors_) m.stop();
        target_ = {};
        cmd_at_ = nuri::Clock::now();
    }

    // 감속 정지. secs 동안 현재 명령속도에서 0 근처까지 단계적으로 낮춘 뒤 출력을 끊는다.
    //
    // 🔴 모터의 하드웨어 램프(Mode 0x03 도달시간)에 맡기면 안 된다.
    //    "0.8초에 걸쳐 1RPM 으로 가라"고 한 번 던져놓고 기다려 봤더니
    //    실측이 이랬다 — 중간값 없이 그대로 있다가 한 번에 떨어졌다:
    //        0.64s  FR+11.0    ← 명령 보낸 지 0.64초, 아직 그대로
    //        0.76s  FR+1.4     ← 뚝
    //    가속에는 먹는데 감속에는 안 듣는다. 그래서 여기서 직접 단계를 만든다.
    //
    //    tick 마다 남은 시간 비율로 목표를 낮춰 setSpeed 를 다시 보낸다.
    //    속도 0 은 규격 밖이라(여자 유지 6.5A + 소음) kCreep 까지만 내리고 듀티를 끊는다.
    void decelerate(double secs = 0.8) {
        if (secs <= 0.0) { stop(); return; }
        bool moving = false;
        for (size_t i = 0; i < motors_.size(); ++i)
            if (std::fabs(target_[i]) >= kCreep) { moving = true; break; }
        if (!moving) { stop(); return; }

        decel_from_ = target_;                 // 어디서부터 줄일지 기억한다
        decel_active_ = true;
        decel_t0_ = nuri::Clock::now();
        decel_last_ = decel_t0_ - kDecelStep;  // 첫 단계는 바로 나가게
        decel_secs_ = secs;
        cmd_at_ = decel_t0_;
    }

private:
    // 감속 진행 — tick 에서 부른다. 다 끝났으면 true.
    //
    // ⚠ 매 tick(20ms) 마다 새 명령을 보내면 poll() 이 영영 Settling(250ms) 을 벗어나지 못해
    //   **피드백이 갱신되지 않는다**(화면 RPM 이 굳은 것처럼 보인다).
    //   단계 간격을 그보다 넉넉히 두어, 단계 사이에 실측이 한 번은 들어오게 한다.
    bool stepDecel() {
        const auto now = nuri::Clock::now();
        const double el = std::chrono::duration<double>(now - decel_t0_).count();
        const double t = el / decel_secs_;                // 0 → 1
        if (t >= 1.0) return true;
        // 선형(1-t)이면 끝에서 여전히 뚝 끊기는 느낌이 남는다.
        // 뒤로 갈수록 완만해지게 해서 마지막을 낮은 속도에서 오래 머물게 한다.
        const double k = (1.0 - t) * (1.0 - t);           // ease-out
        if (now - decel_last_ < kDecelStep) return false; // 아직 다음 단계 아님
        decel_last_ = now;

        for (size_t i = 0; i < motors_.size(); ++i) {
            const double from = decel_from_[i];
            if (std::fabs(from) < kCreep) continue;
            double v = from * k;
            // 부호는 유지한다 — 뒤집히면 역토크가 걸려 오히려 덜컥인다
            if (std::fabs(v) < kCreep) v = (from > 0 ? kCreep : -kCreep);
            target_[i] = v;
            // ramp 는 짧게: 다음 단계가 곧 덮어쓰므로 길게 주면 따라오지 못한다
            motors_[i].setSpeed(v * cfg_.wheels[i].sign, 0.1);
        }
        return false;
    }

public:

    // ── 주기 갱신 ──────────────────────────────────────────────
    // 축을 하나씩 돌며 피드백을 받는다. RS485가 반이중이라 어차피 직렬이므로
    // 매 tick에 한 축씩만 갱신해 루프 시간을 일정하게 유지한다.
    // 9600bps에서 축당 ~15ms, 115200에서 ~1.4ms.
    //
    // 반환: 정상이면 true. **축이 하나라도 실패로 판정되면 전 축을 세우고 false**를 낸다.
    //       4륜이 모두 구동부라 한 축만 죽어도 조향이 틀어져 거동을 신뢰할 수 없다.
    //       호출자(Robot)는 이걸 받아 e-stop을 걸어야 한다.
    bool tick() {
        if (motors_.empty()) return true;

        // 감속 진행. 다 내려왔으면 출력을 끊는다 —
        // 여기까지 와야 여자가 풀린다(안 하면 kCreep 으로 계속 돌며 전류를 먹는다).
        if (decel_active_ && stepDecel()) stop();

        // 정지 중에는 폴링을 띄엄띄엄 한다.
        //
        // 서 있는 로봇에게 50Hz로 "지금 몇 RPM이냐"고 계속 묻는 건 낭비다 —
        // 답이 바뀔 리가 없고, RS485 버스와 CPU만 쓴다.
        //
        // 🔴 그렇다고 아예 끊으면 안 된다. 폴링이 곧 모터 생존 확인이라,
        //    멈춘 사이에 축이 죽으면 움직이라고 명령한 뒤에야 알게 된다.
        //    kIdleDivider=10 이면 50Hz → 5Hz 로, 실패 판정(motor_fail_limit=5)까지
        //    최악 3초 남짓이다. 정지 상태에서 그 정도면 충분하다.
        //    명령이 들어오면 바로 전속 폴링으로 돌아온다(아래 target_ 검사).
        bool commanded_stop = true;
        for (size_t i = 0; i < motors_.size(); ++i)
            if (std::fabs(target_[i]) >= 0.05) { commanded_stop = false; break; }
        if (commanded_stop) {
            if (++idle_skip_ < kIdleDivider) return true;
        }
        idle_skip_ = 0;

        auto& m = motors_[cursor_];
        auto& s = state_[cursor_];
        const auto pr = m.poll(cfg_.poll_timeout);

        // 명령 직후 안정화 구간은 값만 안 쓰고 넘어간다. 실패로 세면
        // 명령을 넣을 때마다 축이 죽은 것으로 오판한다.
        if (pr == nuri::Motor::Poll::Settling) {
            s.consecutive_fail = 0;
            cursor_ = (cursor_ + 1) % motors_.size();
            return true;
        }

        if (pr == nuri::Motor::Poll::Ok) {
            const auto& f = m.last();
            // 모터는 부호 없는 크기 + 방향 플래그로 준다. 장착 방향(sign)까지 되돌려
            // 전진이 +가 되도록 맞춘다.
            const double mag = f.rpm * (f.cw ? -1.0 : 1.0);
            s.rpm = mag * cfg_.wheels[cursor_].sign;
            s.amp = f.amp;
            s.degree = f.degree;
            s.stamp = f.stamp;
            s.alive = true;
            s.consecutive_fail = 0;

            // 🔴 과전류 감시 — 이걸 안 넣어서 모터 4대를 태웠다 (2026-08-07)
            //
            //    브레이크 미해제·기계적 구속·결선 이상이면 무부하인데도 정격의 몇 배가
            //    흐른다. 원인이 뭐든 **전류가 계속 높으면 일단 세운다.**
            //    모터 내장 보호(15A 퓨즈)는 실제로 안 걸렸다 — 믿을 수 없다.
            //
            //    전류는 순간 피크 샘플이라 0xFE(25.4A) 포화가 수시로 튄다.
            //    단발 스파이크로 오작동하지 않게 **중앙값**으로 판정한다.
            auto& w = amp_hist_[cursor_];
            w.push_back(f.amp);
            if (static_cast<int>(w.size()) > cfg_.current_window) w.erase(w.begin());
            if (static_cast<int>(w.size()) == cfg_.current_window) {
                auto sorted = w;
                std::sort(sorted.begin(), sorted.end());
                s.amp_median = sorted[sorted.size() / 2];
                if (s.amp_median > cfg_.overcurrent_a) {
                    if (++s.over_hits >= cfg_.overcurrent_hits) {
                        s.rpm = 0.0;
                        failed_ = s.label;
                        overcurrent_ = true;
                        overcurrent_a_ = s.amp_median;
                        stop();
                        cursor_ = (cursor_ + 1) % motors_.size();
                        return false;      // 호출자(Robot)가 e-stop 을 건다
                    }
                } else {
                    s.over_hits = 0;
                }
            }
        } else if (++s.consecutive_fail >= cfg_.motor_fail_limit) {
            // 한 축이 응답을 잃으면 **전 축을 세운다**. 4륜 구동이라 한 바퀴만 빠져도
            // 좌우 균형이 깨져 어디로 갈지 모른다 — 부분 동작을 허용하지 않는다.
            s.alive = false;
            s.rpm = 0.0;
            failed_ = s.label;
            stop();
            cursor_ = (cursor_ + 1) % motors_.size();
            return false;
        }
        cursor_ = (cursor_ + 1) % motors_.size();
        return true;
    }

    // 전 축을 한 번씩 갱신 (기동 확인·디버그용). tick()보다 오래 걸린다.
    bool pollAll() {
        bool ok = true;
        for (size_t i = 0; i < motors_.size(); ++i) ok = tick() && ok;
        return ok;
    }

    // 마지막으로 실패 판정된 축 라벨 (e-stop 사유에 쓴다)
    const char* failedAxis() const { return failed_; }

    // tick() 이 false 를 낸 이유가 과전류인지 (아니면 응답 없음인지)
    bool overcurrent() const { return overcurrent_; }
    double overcurrentAmp() const { return overcurrent_a_; }

    const std::array<WheelState, 4>& state() const { return state_; }
    const WheelRpm& target() const { return target_; }
    size_t count() const { return motors_.size(); }

    // state_는 4칸 고정 배열이지만 실제 축 수는 motors_.size()다 — 그만큼만 본다
    bool allAlive() const {
        for (size_t i = 0; i < motors_.size(); ++i) if (!state_[i].alive) return false;
        return true;
    }
    int aliveCount() const {
        int n = 0;
        for (size_t i = 0; i < motors_.size(); ++i) if (state_[i].alive) ++n;
        return n;
    }
    nuri::Clock::time_point lastCommand() const { return cmd_at_; }

    // 차체 치수가 채워져 운동학을 쓸 수 있는가
    bool kinematicsReady() const {
        return cfg_.wheel_radius_m > 0.0 && cfg_.track_width_m > 0.0;
    }

private:
    double clampRpm(double v) const {
        return std::clamp(v, -cfg_.max_wheel_rpm, cfg_.max_wheel_rpm);
    }

    const Config& cfg_;
    std::vector<nuri::Motor> motors_;
    std::array<WheelState, 4> state_{};
    WheelRpm target_{};
    size_t cursor_ = 0;
    // 과전류 판정용 이동 창 (축별). 중앙값을 쓰므로 스파이크에 안 흔들린다.
    std::array<std::vector<double>, 4> amp_hist_{};
    bool overcurrent_ = false;
    double overcurrent_a_ = 0.0;
    // 정지 중 폴링 완화 (위 tick() 주석 참고). 10 = 50Hz → 5Hz
    static constexpr unsigned kIdleDivider = 10;
    unsigned idle_skip_ = 0;

    // 감속 정지 (decelerate)에서 "여기까지 내리고 듀티를 끊는다"는 속도.
    //
    // 속도 0 명령은 규격 밖(0x0001~0xFFFD)이라 쓸 수 없다 — 제어기가 0 RPM 을 유지하려
    // 들면서 실측 6.5A 가 흐르고 소음이 난다. 그래서 어딘가에선 반드시 끊어야 하고,
    // **그 순간의 덜컥임은 원리상 없앨 수 없다.** 낮출수록 덜하다.
    //   1.0 → 체감상 "뚝" 하고 섬
    //   0.3 → 출력축 0.3RPM(3분에 한 바퀴). 사실상 멈춘 상태에서 끊긴다
    // 모터 분해능이 0.1RPM 이라 이보다 더 내려도 의미가 없다.
    static constexpr double kCreep = 0.3;   // RPM
    bool decel_active_ = false;
    WheelRpm decel_from_{};                 // 감속 시작 시점의 명령속도
    nuri::Clock::time_point decel_t0_{};
    nuri::Clock::time_point decel_last_{};
    double decel_secs_ = 0.8;
    // 단계 간격. poll() 의 안정화 구간(250ms)보다 길어야 실측이 갱신된다.
    static constexpr auto kDecelStep = std::chrono::milliseconds(300);
    const char* failed_ = "";
    nuri::Clock::time_point cmd_at_{};
};

}  // namespace navi
