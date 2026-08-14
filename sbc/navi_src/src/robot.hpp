// robot.hpp — 장치 전체를 한 곳에서 소유하고 한 주기로 돌린다.
//
//   navi::Config cfg;
//   navi::Robot robot(cfg);
//   robot.setWheelRpm(20.0);
//   while (running) { robot.tick(); }        // tick()이 주기 대기까지 한다
//
// 설계:
//   · 구동계(RS485)는 tick()에서 순차 처리 — 반이중이라 어차피 직렬, 락이 필요 없다
//   · 카메라는 각자 스레드 — 블로킹 grab이 제어 루프를 막으면 안 된다
//   · 모든 정지 경로(시그널·예외·워치독·소멸자)가 estop() 하나로 수렴한다
//   · 🔴 **어떤 장치가 없어도 프로그램은 뜬다.** 없는 장치는 상태·알람으로 알린다.
//     기동 자체가 막히면 원인조차 못 본다 — 현장에서 제일 곤란한 상황이다.
//     구동계가 없으면 구동 명령만 거부하고(accept), 센서 수집·발행은 계속한다.
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "camstream.hpp"
#include "config.hpp"
#include "drive.hpp"
#include "hall.hpp"       // HallCounter — actuator.hpp를 함께 끌어온다
#include "nurirobot.hpp"
#include "tof.hpp"
#include "vidserver.hpp"

// 열화상은 v4l2 로 읽으므로 TmSDK 없이도 빌드·동작한다.
// (SDK 가 깔린 환경에서만 FFC 가 추가로 열린다 — 조건부는 thermal.hpp 안에 있다)
#include "thermal.hpp"

namespace navi {

struct RobotStatus {
    // 구동계가 안 잡혀도 프로그램은 뜬다 (센서 수집은 계속해야 하므로).
    // 이 경우 drive_ok=false 이고 **모든 구동 명령이 거부된다.**
    bool drive_ok = false;
    std::string drive_error;

    std::vector<WheelState> wheels;   // 실제 올라온 축만
    int wheels_alive = 0;
    BodyVel body{};                 // 운동학 준비됐을 때만 채운다
    bool kinematics = false;

    bool actuator_present = false;
    State actuator_state = State::Idle;
    long actuator_position = 0;
    double actuator_current = 0.0;

    std::vector<CamStatus> cams;
    VideoStatus video{};

    bool tof_present = false;
    TofReading tof{};
    TofStatus tof_status{};

    bool thermal_present = false;
    // 장치는 열려 있는데 프레임이 안 오는 상태. SDK 가 블로킹되면 이렇게 된다 —
    // open 만 보면 정상으로 보여서 놓친다.
    bool thermal_stalled = false;
    ThermalReading thermal{};       // ℃
    ThermalStatus thermal_status{};

    bool estopped = false;          // 래치 — reset() 필요
    bool watchdog_tripped = false;  // 명령 재개 시 자동 해제
    std::string estop_reason;
    double tick_ms = 0.0;           // 최근 tick 실제 소요 시간
    double tick_max_ms = 0.0;       // 최대치 — 주기를 못 맞추는지 확인용
    unsigned long long ticks = 0;
};

class Robot {
public:
    explicit Robot(const Config& cfg) : cfg_(cfg) {
        // ── 구동계 ──
        //
        // 🔴 모터가 없다고 프로그램이 죽으면 안 된다.
        //    카메라·ToF 는 모터와 무관한데 같이 못 뜬다. 실제로 모터 4대가 손상됐을 때
        //    센서 데이터 수집조차 못 하는 상태가 됐다 (2026-08-07).
        //
        //    대신 **구동 명령은 전부 거부한다.** 축이 하나라도 빠진 4륜차를 굴리면
        //    어디로 갈지 모른다 — 부분 동작은 허용하지 않는다는 원칙은 그대로다.
        //    상태 토픽에 drive_ok=false 로 나가므로 상위가 알 수 있다.
        try {
            bus_ = std::make_unique<nuri::Bus>(cfg_.rs485_port, cfg_.rs485_baud,
                                               cfg_.de_chip, cfg_.de_line, cfg_.de_guard_us);
            drive_ = std::make_unique<Drive>(cfg_, *bus_);
        } catch (const std::exception& e) {
            drive_err_ = e.what();
            drive_.reset();
            bus_.reset();
        }

        // ── 액추에이터: 없어도 계속 (degraded) ──
        if (cfg_.use_actuator) {
            try {
                const auto& board = Actuator::detect(cfg_.hat_rev);
                act_ = std::make_unique<Actuator>(board);
                hall_ = std::make_unique<HallCounter>(board.hall_chip, board.hall_a, board.hall_chip_b, board.hall_b);
                act_->attachHall(hall_.get());
            } catch (const std::exception& e) {
                act_err_ = e.what();
                act_.reset();
                hall_.reset();
            }
        }

        // ── 카메라 ──
        for (const auto& c : cfg_.cams) cams_.push_back(std::make_unique<CamStream>(c));

        // ── IR 영상 송출 ──
        // 카메라를 놓지 않고 여기서 인코딩한다. 보는 사람이 없으면 VPU 도 안 돈다.
        if (cfg_.video.enabled && !cams_.empty()) {
            try {
                video_ = std::make_unique<VideoServer>(cfg_.video, cams_.front().get());
            } catch (const std::exception& e) {
                video_err_ = e.what();
            }
        }

        // ── ToF ──
        if (cfg_.tof.enabled) tof_ = std::make_unique<TofStream>(cfg_.tof);

        // ── 열화상: TmSDK로 따로 연다 (UVC만으로는 프레임이 안 나온다) ──
        if (cfg_.thermal.enabled) {
            auto tcfg = cfg_.thermal;
            // 프레임을 내보낼 거면 이미지를 만들어 둬야 한다.
            // 설정 두 개(stream_thermal, keep_image)를 따로 맞추게 하면 반드시 실수한다 —
            // 스트리밍을 켰는데 이미지가 안 나가는 걸로 한 번 헤맸다.
            if (cfg_.mqtt.stream_thermal) tcfg.keep_image = true;
            thermal_ = std::make_unique<ThermalStream>(tcfg);
        }

        cmd_at_ = nuri::Clock::now();
    }

    ~Robot() {
        try { estop("종료"); } catch (...) {}
        video_.reset();    // CamStream 을 참조하므로 먼저 정리한다
        cams_.clear();     // 스레드 정리
    }
    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    // ── 명령 ───────────────────────────────────────────────────
    // 두 가지 정지 상태를 구분한다:
    //   워치독 트립 — 명령이 다시 들어오면 자동 해제된다 (통신 복구는 정상 상황)
    //   e-stop      — **래치된다.** reset()을 명시적으로 불러야 풀린다
    //                 (시그널·예외·장치 고장. 명령 하나로 풀리면 e-stop이 아니다)
    // e-stop 중에는 명령을 전부 무시한다.

    void setWheelRpm(const WheelRpm& rpm, double ramp_s = 0.5) {
        if (!accept()) return;
        drive_->setWheelRpm(rpm, ramp_s);
        cmd_at_ = nuri::Clock::now();
    }
    void setWheelRpm(double all, double ramp_s = 0.5) {
        setWheelRpm(WheelRpm{all, all, all, all}, ramp_s);
    }
    // 차체 치수를 채우지 않으면 예외를 던진다 (drive.hpp의 requireBody)
    void setBodyVelocity(BodyVel v, double ramp_s = 0.5) {
        if (!accept()) return;
        drive_->setBodyVelocity(v, ramp_s);
        cmd_at_ = nuri::Clock::now();
    }
    // 평상시 정지 — 감속해서 선다. secs=0 이면 즉시 차단.
    // e-stop·워치독 트립은 이걸 쓰지 않는다(아래 estop 참고). 그쪽은 즉시 끊어야 한다.
    void stopDrive(double secs = -1.0) {
        if (!drive_) return;
        if (secs < 0.0) secs = cfg_.decel_secs;
        drive_->decelerate(secs);
        cmd_at_ = nuri::Clock::now();
    }

    // 명령 의도가 아직 유효하다고 알린다. 상위 제어기(MQTT·ROS 등)가 살아 있는 동안
    // 주기적으로 불러야 워치독이 안 걸린다 — 그게 워치독의 목적이다.
    void keepalive() {
        if (estopped_) return;          // e-stop은 래치 — keepalive로 풀리지 않는다
        cmd_at_ = nuri::Clock::now();
    }

    // 액추에이터 — 없으면 조용히 무시한다 (호출자가 매번 검사하지 않도록)
    //
    // 🔴 구동계와 독립이다. 모터가 안 잡혀도 액추에이터는 움직여야 한다.
    //    accept() 를 쓰면 !drive_ 에서 막혀 조용히 무시된다 (실제로 그랬다).
    void actuatorStart(bool ret, double duty = -1.0) {
        if (!act_ || !acceptCommon()) return;
        act_->start(ret, duty < 0 ? cfg_.actuator_duty : duty);
        cmd_at_ = nuri::Clock::now();
    }
    void actuatorStop() { if (act_) act_->stop(); }
    Actuator* actuator() { return act_.get(); }

    // ── 주기 갱신 ──────────────────────────────────────────────
    // 한 번 부르면: 구동 축 1개 폴링 + 액추에이터 상태 갱신 + 워치독 검사 + 주기 대기.
    void tick() {
        const auto t0 = nuri::Clock::now();

        // 축이 하나라도 죽으면 Drive가 전 축을 세우고 false를 낸다.
        // 4륜 구동이라 부분 동작은 허용하지 않는다 — e-stop으로 래치해서
        // 사람이 원인을 확인하고 reset() 하기 전까지 다시 못 움직이게 한다.
        if (drive_ && !drive_->tick() && !estopped_) {
            if (drive_->overcurrent()) {
                // 무부하에서 정격을 크게 넘는 전류는 브레이크 미해제·기계적 구속·결선 이상이다.
                // 원인이 뭐든 계속 돌리면 모터가 탄다 (2026-08-07 실제로 4대 손실).
                char buf[160];
                std::snprintf(buf, sizeof buf,
                    "휠 %s 과전류 %.1fA (임계 %.1fA) — 전 축 정지. "
                    "브레이크 해제·기계 구속·결선을 확인할 것",
                    drive_->failedAxis(), drive_->overcurrentAmp(), cfg_.overcurrent_a);
                estop(buf);
            } else {
                estop(std::string("휠 ") + drive_->failedAxis() + " 응답 없음 — 전 축 정지");
            }
        }

        if (act_) act_->poll();

        // 워치독 — 새 명령 없이 이 시간이 지나면 세운다.
        // 통신이 끊긴 채로 계속 굴러가는 상황을 막는 장치다.
        // e-stop과 달리 래치하지 않는다 — 명령이 다시 들어오면 풀린다.
        // 첫 명령이 오기 전에는 워치독을 걸지 않는다 — 기동 후 상위가 붙기까지의
        // 공백에 이벤트만 시끄럽게 나가고, 정지 상태를 다시 정지시키는 의미가 없다.
        if (had_command_ && !estopped_ && !watchdog_tripped_ &&
            nuri::Clock::now() - cmd_at_ > cfg_.watchdog && moving()) {
            watchdog_tripped_ = true;
            if (drive_) drive_->stop();
            if (act_) act_->stop();
        }

        const auto spent = nuri::Clock::now() - t0;
        tick_ms_ = std::chrono::duration<double, std::milli>(spent).count();
        tick_max_ms_ = std::max(tick_max_ms_, tick_ms_);
        ++ticks_;
        if (spent < cfg_.tick_period) std::this_thread::sleep_for(cfg_.tick_period - spent);
    }

    // ── 정지 ───────────────────────────────────────────────────
    // 래치된다. reset()을 부르기 전까지 모든 명령이 무시된다.
    // 여러 번 불려도 안전하고, 첫 사유를 유지한다 (나중 호출이 원인을 덮지 않도록).
    void estop(const std::string& reason = "") {
        if (!estopped_) estop_reason_ = reason;
        estopped_ = true;
        if (drive_) drive_->stop();
        if (act_) act_->stop();
    }

    // e-stop 해제. 원인을 확인하고 사람이(또는 상위 제어기가) 명시적으로 부른다.
    void reset() {
        estopped_ = false;
        watchdog_tripped_ = false;
        estop_reason_.clear();
        cmd_at_ = nuri::Clock::now();
    }

    bool estopped() const { return estopped_; }
    bool watchdogTripped() const { return watchdog_tripped_; }
    const std::string& estopReason() const { return estop_reason_; }

    // 실제로 움직이는 중인가 (워치독 판정용).
    //
    // 축을 하나씩 돌려 폴링하므로 나머지 축의 값은 최대 (축수 × tick)만큼 오래된 것이다.
    // 9600bps·4축이면 ~95ms다. 오래된 샘플을 "정지"로 읽으면 워치독이 그냥 넘어가므로,
    // **모르면 움직인다고 본다** — 안전한 쪽으로 틀린다.
    bool moving() const {
        const auto now = nuri::Clock::now();
        // 🔴 구동계가 없어도 액추에이터는 움직일 수 있다. 워치독이 그걸 못 보면
        //    명령이 끊긴 채로 계속 밀고 나간다 — 장치는 독립이어도 안전은 통합이다.
        if (act_ && act_->running()) return true;
        if (!drive_) return false;
        for (const auto& s : drive_->state()) {
            if (!s.alive) continue;
            if (s.stamp.time_since_epoch().count() == 0) continue;   // 아직 한 번도 안 읽음
            if (now - s.stamp > cfg_.motor_timeout) return true;     // 오래됨 → 불명 → 움직인다고 간주
            if (std::fabs(s.rpm) > 0.5) return true;
        }
        if (act_ && act_->running()) return true;
        // 측정과 무관하게, 방금 0이 아닌 속도를 명령했으면 움직이는 것으로 본다
        for (const double v : drive_->target()) if (std::fabs(v) > 0.5) return true;
        return false;
    }

    // ── 상태 ───────────────────────────────────────────────────
    RobotStatus status() const {
        RobotStatus s;
        if (tof_) {
            s.tof_present = tof_->ok();
            s.tof = tof_->latest();
            s.tof_status = tof_->status();
        }
        s.drive_ok = static_cast<bool>(drive_);
        s.drive_error = drive_err_;
        if (drive_) {
            const auto& ws = drive_->state();
            s.wheels.assign(ws.begin(), ws.begin() + drive_->count());
            s.wheels_alive = drive_->aliveCount();
            s.kinematics = drive_->kinematicsReady();
        }
        if (s.kinematics) {
            try { s.body = drive_->measuredBody(); } catch (...) {}
        }
        s.actuator_present = static_cast<bool>(act_);
        if (act_) {
            s.actuator_state = act_->state();
            s.actuator_position = act_->position();
            s.actuator_current = act_->current();
        }
        for (const auto& c : cams_) s.cams.push_back(c->status());
        if (video_) s.video = video_->status();
        s.thermal_present = thermal_ && thermal_->ok();
        if (thermal_) {
            s.thermal = thermal_->latest();
            s.thermal_status = thermal_->status();
        }
        s.estopped = estopped_;
        s.watchdog_tripped = watchdog_tripped_;
        s.estop_reason = estop_reason_;
        s.tick_ms = tick_ms_;
        s.tick_max_ms = tick_max_ms_;
        s.ticks = ticks_;
        return s;
    }

    // 카메라 접근 — key로 찾는다 ("ir")
    CamStream* cam(const std::string& key) {
        for (auto& c : cams_) if (c->key() == key) return c.get();
        return nullptr;
    }

    ThermalStream* thermal() { return thermal_.get(); }

    // ⚠ 널일 수 있다 — 모터가 안 잡혀도 프로그램은 뜬다.
    //   호출 전에 status().drive_ok 로 확인할 것.
    Drive* drive() { return drive_.get(); }
    const std::string& driveError() const { return drive_err_; }
    const Config& config() const { return cfg_; }
    const std::string& actuatorError() const { return act_err_; }

private:
    // 명령을 받아들일지. e-stop은 래치라 거부하고, 워치독 트립은 여기서 풀린다.
    // 여기서 한 번에 막는다 — 개별 호출부마다 검사하면 하나 빠뜨렸을 때 널 접근으로 죽는다.
    //
    // 🔴 구동계 유무는 **구동 명령에만** 적용한다.
    //    액추에이터는 별개 장치라 모터가 없어도 움직여야 한다. 예전에 accept() 하나로
    //    묶어 두는 바람에, 모터가 안 잡힌 상태에서 `navi act ext` 가 조용히 무시됐다
    //    (PWM period 만 잡히고 duty·enable 이 0 인 채로 "정지" 표시).
    bool accept() {
        if (!drive_) return false;      // 모터가 안 잡혔다 — 구동은 전부 거부
        return acceptCommon();
    }

    // 구동계와 무관한 명령용. e-stop 은 똑같이 지킨다.
    bool acceptCommon() {
        if (estopped_) return false;
        watchdog_tripped_ = false;
        had_command_ = true;
        return true;
    }

    const Config& cfg_;
    std::unique_ptr<nuri::Bus> bus_;
    std::unique_ptr<TofStream> tof_;
    std::string drive_err_;   // 구동계 초기화 실패 사유 (비어 있으면 정상)
    std::unique_ptr<Drive> drive_;
    std::unique_ptr<Actuator> act_;
    std::unique_ptr<HallCounter> hall_;
    std::vector<std::unique_ptr<CamStream>> cams_;
    std::unique_ptr<VideoServer> video_;
    std::string video_err_;
    std::unique_ptr<ThermalStream> thermal_;
    std::string act_err_;

    nuri::Clock::time_point cmd_at_{};
    bool estopped_ = false;          // 래치
    bool watchdog_tripped_ = false;  // 자동 해제
    bool had_command_ = false;       // 첫 명령을 받았는가 (그 전엔 워치독 비활성)
    std::string estop_reason_;
    double tick_ms_ = 0.0, tick_max_ms_ = 0.0;
    unsigned long long ticks_ = 0;
};

}  // namespace navi
