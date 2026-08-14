// main.cpp — 통합 진입점.
//
// 빌드: g++ -std=c++17 -O2 -o navi main.cpp -lgpiod -lpthread -lmosquitto
// 실행:
//   ./navi mqtt                 # 운용 모드 — 상위 서버 MQTT로 명령 받고 상태 발행
//   ./navi                      # 상태 모니터 (모터를 돌리지 않는다 — 기본은 안전)
//   ./navi spin 20 5            # 네 바퀴 20RPM 5초
//   ./navi turn 20 5            # 제자리 회전 (좌우 반대 부호)
//   ./navi body 0.3 0.5 5       # vx 0.3m/s, wz 0.5rad/s — 차체 치수를 채워야 동작
//   ./navi act ext 3            # 액추에이터 EXT 3초
//   ./navi wdtest 5             # 워치독 안전 경로 시험
//
// Ctrl+C(SIGINT) / SIGTERM 은 어느 시점에 들어와도 estop을 거쳐 정리하고 나간다.
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include <csignal>
#include <unistd.h>

#include "confload.hpp"
#include "mqtt.hpp"   // robot.hpp를 함께 끌어온다

namespace {

std::atomic<bool> g_stop{false};

void onSignal(int) { g_stop = true; }   // 핸들러에서는 플래그만 — 나머지는 루프가 한다

const char* stateName(navi::State s) { return navi::toString(s); }

// 터미널이면 한 줄을 덮어쓰고, 파이프·로그면 줄바꿈으로 남긴다.
// (제어문자를 그대로 흘리면 로그가 못 읽게 된다)
const bool g_tty = ::isatty(1) != 0;

void printStatus(const navi::RobotStatus& s) {
    // 20ms마다 찍으면 읽을 수 없다 — 사람이 볼 주기로 줄인다
    static auto last = std::chrono::steady_clock::time_point{};
    const auto now = std::chrono::steady_clock::now();
    if (now - last < std::chrono::milliseconds(250)) return;
    last = now;

    if (g_tty) std::printf("\033[2K\r");
    std::printf("[%llu] tick %.1f/%.1fms  ", s.ticks, s.tick_ms, s.tick_max_ms);
    for (const auto& w : s.wheels)
        // 전류가 높으면 같이 띄운다 — 이게 안 보여서 모터를 태웠다.
        // (임시 std::string 의 c_str() 을 printf 에 넘기면 안 된다 — 그 자리에서 수명이 끝난다)
        if (w.amp_median > 3.0)
            std::printf("%s%s%+.0f⚡%.0fA ", w.label, w.alive ? ":" : "✗", w.rpm + 0.0, w.amp_median);
        else
            std::printf("%s%s%+.0f ", w.label, w.alive ? ":" : "✗", w.rpm + 0.0);
    if (s.kinematics) std::printf("| vx %.2f wz %.2f ", s.body.vx, s.body.wz);
    if (s.actuator_present)
        std::printf("| ACT %s %ld %.2fA ", stateName(s.actuator_state),
                    s.actuator_position, s.actuator_current);
    for (const auto& c : s.cams)
        if (c.open) std::printf("| %s %.0ffps d%llu ", c.key.c_str(), c.fps, c.drops);
    if (s.thermal_present)
        std::printf("| 열 %.1f~%.1f℃ 중앙%.1f %.0ffps ", s.thermal.lo, s.thermal.hi,
                    s.thermal.center, s.thermal_status.fps);
    if (s.watchdog_tripped) std::printf("| ⏱ 워치독 정지 ");
    if (s.estopped) std::printf("| ⛔ %s", s.estop_reason.c_str());
    if (!g_tty) std::printf("\n");
    std::fflush(stdout);
}

void banner(navi::Robot& robot) {
    const auto s = robot.status();
    const auto& cfg = robot.config();
    if (s.drive_ok) {
        std::printf("── 구동 모터 %zu축 @ %s %d bps\n", robot.drive()->count(),
                    cfg.rs485_port, cfg.rs485_baud);
        for (const auto& w : s.wheels)
            std::printf("     %s  ID %u\n", w.label, w.id);
    } else {
        // 모터가 없어도 센서 수집은 계속한다. 다만 구동 명령은 전부 거부된다.
        std::printf("── 🔴 구동 모터 없음 — %s\n", robot.driveError().c_str());
        std::printf("     구동 명령은 거부된다. 센서 수집·발행은 계속한다\n");
    }
    std::printf("── 운동학: %s\n", s.kinematics
        ? "준비됨"
        : "미설정 (config.hpp의 wheel_radius_m / track_width_m 실측 필요)");
    if (s.actuator_present) {
        // 리비전을 함께 찍는다 — 틀리면 홀 핀이 어긋나 카운트만 0 이 되는데,
        // 배너에 안 나오면 그걸 알 방법이 없다.
        const auto& b = navi::Actuator::detect(robot.config().hat_rev);
        std::printf("── 액추에이터: 정상 (HAT V%d, 홀 %s:%u · %s:%u)\n",
                    navi::Actuator::hatRev(robot.config().hat_rev),
                    b.hall_chip, b.hall_a, b.hall_chip_b, b.hall_b);
    } else {
        std::printf("── 액추에이터: 없음 — %s\n", robot.actuatorError().c_str());
        // PWM·GPIO sysfs는 root만 쓸 수 있다. 이전 데모들도 sudo로 돌렸다.
        if (robot.actuatorError().find("/sys/class/gpio") != std::string::npos ||
            robot.actuatorError().find("/sys/class/pwm") != std::string::npos)
            std::printf("     ↳ sysfs 권한 문제로 보인다 — sudo 로 실행할 것\n");
    }

    // 9600bps에서는 축당 왕복이 ~15ms라 4축이면 주기를 못 맞춘다 (실측 1축에 23.6ms).
    if (cfg.rs485_baud < 115200)
        std::printf("── ⚠ RS485 %d bps: 축당 폴링이 느려 tick 주기(%ldms)를 넘긴다.\n"
                    "     모터 EEPROM을 115200(코드 0x0D)으로 올리고 config를 맞출 것\n",
                    cfg.rs485_baud, static_cast<long>(cfg.tick_period.count()));
    for (const auto& c : s.cams) {
        if (c.open)
            std::printf("── 카메라 %s: %s  %ux%u %s  (%s)\n", c.key.c_str(), c.card.c_str(),
                        c.width, c.height, navi::cam::fourccStr(c.format).c_str(),
                        c.path.c_str());
        else
            std::printf("── 카메라 %s: 없음 — %s\n", c.key.c_str(), c.error.c_str());
    }
    {
        const auto& t = s.thermal_status;
        std::printf("── 열화상: %s  frames=%llu drops=%llu %s%s\n",
                    s.thermal_present ? "열림" : "없음", t.frames, t.drops,
                    t.via_v4l2 ? "[v4l2] " : "",
                    t.v4l2_error.empty() ? "" : ("v4l2실패: " + t.v4l2_error).c_str());
    }
    if (s.tof_present)
        std::printf("── ToF: %d cm (강도 %d)\n", s.tof.dist_cm, s.tof.strength);
    else
        std::printf("── ToF: 없음\n");
    std::printf("\n");
}

// 지정 시간 동안 tick을 돌리며 상태를 찍는다. 시그널이 오면 즉시 빠져나온다.
void runFor(navi::Robot& robot, double secs) {
    const auto t0 = navi::nuri::Clock::now();
    int n = 0;
    while (!g_stop &&
           std::chrono::duration<double>(navi::nuri::Clock::now() - t0).count() < secs) {
        // 상위 제어기가 살아 있다는 신호. 이걸 빼면 워치독이 걸려 스스로 선다 —
        // 실제 ROS 노드도 cmd_vel을 받을 때마다 이 자리를 갱신하게 된다.
        robot.keepalive();
        robot.tick();
        if (++n % 10 == 0) printStatus(robot.status());
    }
    std::printf("\n");
}

// 감속 정지 후, 실제로 다 설 때까지 tick 을 돌린다.
//
// 🔴 stopDrive() 만 부르고 바로 빠져나오면 **감속이 진행될 시간이 없다.**
//    stopDrive 는 모터에 "N초에 걸쳐 줄여라"라고 시켜놓을 뿐이고,
//    실제로 줄어드는 동안 tick 이 돌아야 마지막에 듀티를 끊어준다.
//    이걸 빠뜨려서 감속을 넣고도 계속 뚝 섰다 (2026-08-07).
//    프로그램이 그냥 끝나면 소멸자가 출력을 차단해 버리니 결과가 같아진다.
void stopAndSettle(navi::Robot& robot, const navi::Config& cfg) {
    robot.stopDrive();
    // 감속 시간 + 여유. keepalive 를 계속 줘야 워치독이 끼어들어 즉시 차단하지 않는다.
    const auto t0 = navi::nuri::Clock::now();
    const double wait = cfg.decel_secs + 0.4;
    while (std::chrono::duration<double>(navi::nuri::Clock::now() - t0).count() < wait) {
        robot.keepalive();
        robot.tick();
        printStatus(robot.status());
    }
    std::printf("\n정지 완료\n");
}

// ── 알람 ───────────────────────────────────────────────────────────
//
// 이벤트와 나눈 이유: 이벤트는 "무슨 일이 있었다"는 기록이고,
// 알람은 **지금 사람이 조치해야 하는 상태**다. 상위 대시보드가 다르게 다룬다.
//
// 상태가 바뀔 때만 발행한다 — 매 tick 보내면 retain 토픽을 계속 덮어써 의미가 없다.
// 해제될 때도 한 번 보낸다(active=false). 안 보내면 이미 풀린 알람이 영영 남는다.
struct AlarmState {
    bool drive_down = false;
    bool estop = false;
    bool watchdog = false;
    bool cam_down = false;
    bool tof_down = false;
    bool thermal_down = false;
    bool thermal_fault = false;
    bool thermal_stalled = false;
    // 브로커 끊김은 알람으로 못 알린다(알람 자체가 못 나간다) — LWT state/online 이 그 역할이다

    // 🔴 알람은 retain 이라 프로세스가 죽어도 브로커에 남는다. 재시작했을 때
    //    "변화 없음"으로 아무것도 안 보내면 **지난 실행의 알람이 영영 살아남는다.**
    //    실제로 겪었다 — ToF·열화상이 멀쩡한데 tof_down/thermal_fault 가 계속 active 였다.
    //    그래서 첫 평가에서는 변화가 없어도 전부 한 번 발행해 현재 상태로 덮는다.
    bool primed = false;
};

void raiseAlarms(navi::MqttLink& link, const navi::RobotStatus& s, AlarmState& a) {
    using Sev = navi::MqttLink::Sev;

    // 연결 전에 보내봐야 버려진다. 끊긴 동안은 prime 을 되돌려, 다시 붙으면 전체를
    // 한 번 더 보낸다 — 브로커가 재시작해 retain 을 잃었을 수도 있다.
    if (!link.connected()) { a.primed = false; return; }

    auto set = [&](bool& prev, bool now, const char* key, Sev sev, const std::string& msg) {
        if (prev == now && a.primed) return;
        prev = now;
        link.publishAlarm(key, sev, now ? msg : "해제됨", now);
    };

    // 구동계가 안 잡히면 이동 자체가 불가하다 — 가장 심각하다
    set(a.drive_down, !s.drive_ok, "drive_down", Sev::Critical,
        "구동계 없음 — " + s.drive_error);

    set(a.estop, s.estopped, "estop", Sev::Critical,
        s.estop_reason.empty() ? "비상정지" : s.estop_reason);

    // 워치독은 통신이 끊겼다는 뜻이라 경고. 명령이 다시 오면 자동 해제된다
    set(a.watchdog, s.watchdog_tripped, "watchdog", Sev::Warn,
        "명령 끊김으로 정지 — 상위 연결 확인");

    bool cam_bad = false;
    std::string cam_msg;
    for (const auto& c : s.cams)
        if (!c.open) { cam_bad = true; cam_msg += c.key + " "; }
    set(a.cam_down, cam_bad, "camera_down", Sev::Warn, "카메라 없음: " + cam_msg);

    set(a.tof_down, !s.tof_present, "tof_down", Sev::Warn, "거리센서 없음");

    set(a.thermal_down, !s.thermal_present, "thermal_down", Sev::Warn,
        "열화상 없음 — " + s.thermal_status.error);

    // 센서가 스스로 고장을 보고한 경우. 재연결로 안 풀리므로 사람이 봐야 한다 —
    // 그래서 warn 이 아니라 critical 이다.
    set(a.thermal_fault, s.thermal_status.fault, "thermal_fault", Sev::Critical,
        "열화상 센서 고장 — 통신은 되는데 이미징이 안 된다. 교체·점검 필요");

    // open=true 인데 프레임이 안 오는 상태. SDK 가 블로킹된 경우가 있다.
    set(a.thermal_stalled, s.thermal_stalled, "thermal_stalled", Sev::Warn,
        "열화상 프레임 끊김 — 장치는 열려 있으나 갱신이 없다");

    a.primed = true;
}

// ── MQTT 모드 ──────────────────────────────────────────────────────
// 상위 서버에서 명령을 받아 돌린다. 이게 실제 운용 모드다.
//
// 명령이 올 때마다 워치독이 갱신되므로, 상위나 네트워크가 죽으면 500ms 뒤 스스로 선다.
// mosquitto 콜백은 별도 스레드에서 오지만 MqttLink가 큐로 받아주므로, 여기서는
// 한 스레드에서 안전하게 꺼내 적용한다.
void runMqtt(navi::Robot& robot, const navi::Config& cfg) {
    navi::MqttLink link(cfg);
    std::printf("MQTT %s  prefix=%s\n", link.broker().c_str(), cfg.mqtt.prefix);
    std::printf("  구독: %s/cmd/{wheel,body,actuator,stop,estop,reset}\n", cfg.mqtt.prefix);
    std::printf("  발행: %s/state  %s/event  %s/state/online(LWT)\n\n",
                cfg.mqtt.prefix, cfg.mqtt.prefix, cfg.mqtt.prefix);

    auto last_pub = std::chrono::steady_clock::now();
    // 상태가 그대로여도 이 주기로는 반드시 한 번 발행한다.
    // 상위가 "값이 안 오네 = 죽었나" 하고 오해하지 않게 하는 최소 신호다.
    constexpr auto kStateHeartbeat = std::chrono::seconds(2);
    auto last_hb = last_pub;
    std::string last_digest;
    bool was_connected = false, was_estop = false, was_wd = false;

    // 프레임 전송 — 상위가 cmd/stream 으로 켠다. 기본은 꺼짐.
    bool stream_on = cfg.mqtt.stream_enabled;
    int  stream_fps = cfg.mqtt.stream_fps;
    auto last_frame = last_pub;
    AlarmState alarm_state;

    while (!g_stop) {
        // ① 상위 명령 적용 — 한 tick에 밀린 것까지 다 소화한다
        while (auto c = link.nextCommand()) {
            using K = navi::Command::Kind;
            if (!c->error.empty()) {
                link.publishEvent("bad_command", c->error);
                std::printf("⚠ %s\n", c->error.c_str());
                continue;
            }
            switch (c->kind) {
                case K::Wheel:    robot.setWheelRpm(c->wheel, c->ramp); break;
                case K::Body:     robot.setBodyVelocity(c->body, c->ramp); break;
                case K::Actuator: robot.actuatorStart(c->actuator_ret, c->actuator_duty); break;
                case K::Stop:     robot.stopDrive(); break;
                case K::Estop:    robot.estop(c->reason); break;
                case K::Reset:    robot.reset(); break;
                case K::ThermalFFC: {
                    // 셔터를 닫고 균일도를 다시 잡는 동안 1초쯤 화면이 멈춘다.
                    // 안 되는 이유를 그대로 올린다 — 조작자가 부른 명령이다.
                    const std::string err = robot.thermal() ? robot.thermal()->runFFC()
                                                            : "열화상이 안 열렸다";
                    link.publishEvent("thermal_ffc", err.empty() ? "OK" : err);
                    break;
                }
                case K::Stream: {
                    // 상위가 볼 때만 켠다. 아무도 안 보는데 프레임을 흘리면 대역폭만 먹는다.
                    stream_on = c->stream_on;
                    if (c->stream_fps > 0) stream_fps = c->stream_fps;
                    link.publishEvent("stream", stream_on
                        ? ("on " + std::to_string(stream_fps) + "fps") : "off");
                    break;
                }
                case K::Unknown:  break;
            }
        }

        robot.tick();
        const auto s = robot.status();

        // ② 상태 발행 (주기) — 최신만 중요하므로 QoS 0
        //
        //    정지해서 아무것도 안 변할 때 5Hz로 같은 JSON을 계속 흘리는 건 낭비다.
        //    내용이 그대로면 건너뛰되, 상위가 "죽었나?" 하고 오해하지 않도록
        //    heartbeat 주기로는 반드시 한 번 보낸다. (ticks 처럼 매번 변하는 필드는
        //    비교에서 빼야 하므로, 비교용 JSON은 toJson 이 아니라 아래 요약을 쓴다)
        const auto now = std::chrono::steady_clock::now();
        if (now - last_pub >= cfg.mqtt.state_period) {
            auto payload = navi::toJson(s);
            const auto digest = navi::stateDigest(s);
            const bool changed = digest != last_digest;
            const bool heartbeat = now - last_hb >= kStateHeartbeat;
            if (changed || heartbeat) {
                last_pub = now;
                last_digest = digest;
                last_hb = now;
                link.publish("state", payload);
            } else {
                last_pub = now;      // 다음 검사도 같은 주기로 돈다
            }
        }

        // ③ 프레임 전송 (초당 stream_fps 장)
        //
        //    상태 토픽과 주기가 다르다 — 상태는 "변할 때만", 프레임은 "일정 간격"이다.
        //    ⚠ 큰 이미지를 MQTT 로 밀면 브로커 큐가 밀린다. 열화상은 80x60 BMP(14KB)라
        //      10fps 에 140KB/s 로 감당되지만, IR 1080p 는 그 10배가 넘는다.
        //      IR 영상은 MediaMTX(RTSP/WebRTC)로 보내는 게 맞다 — 그쪽이 영상용이다.
        if (stream_on && stream_fps > 0) {
            const auto period = std::chrono::milliseconds(1000 / stream_fps);
            if (now - last_frame >= period) {
                last_frame = now;
                if (cfg.mqtt.stream_thermal && robot.thermal()) {
                    std::vector<uint8_t> bmp;
                    if (robot.thermal()->image(bmp))
                        link.publishFrame("thermal", bmp.data(), bmp.size());
                }
                // ToF 는 이미지가 아니라 값이지만, 프레임 주기로 보내야 상위가
                // 영상과 같은 시간축에 놓을 수 있다. 상태 토픽(변할 때만)과 별개다.
    {
        const auto& t = s.thermal_status;
        std::printf("── 열화상: %s  frames=%llu drops=%llu %s%s\n",
                    s.thermal_present ? "열림" : "없음", t.frames, t.drops,
                    t.via_v4l2 ? "[v4l2] " : "",
                    t.v4l2_error.empty() ? "" : ("v4l2실패: " + t.v4l2_error).c_str());
    }
    if (s.tof_present) {
                    const navi::Json tj{{"dist_cm", s.tof.dist_cm},
                                        {"strength", s.tof.strength},
                                        {"valid", s.tof.valid}};
                    const auto ts = tj.dump();
                    link.publishFrame("tof", ts.data(), ts.size());
                }
            }
        }

        // ④ 알람 — 사람이 조치해야 하는 상태만. 상태가 바뀔 때만 보낸다.
        //    retain 이라 상위가 나중에 붙어도 현재 걸린 알람을 바로 본다.
        raiseAlarms(link, s, alarm_state);

        // ③ 상태 변화는 이벤트로 따로 알린다. 상태 토픽을 놓쳐도 원인은 남아야 한다.
        if (s.estopped && !was_estop) link.publishEvent("estop", s.estop_reason);
        if (s.watchdog_tripped && !was_wd) link.publishEvent("watchdog", "명령 끊김으로 정지");
        was_estop = s.estopped;
        was_wd = s.watchdog_tripped;

        if (link.connected() != was_connected) {
            was_connected = link.connected();
            std::printf("\n%s%s\n", was_connected ? "✅ 브로커 연결" : "⚠ 브로커 끊김 — ",
                        was_connected ? "" : link.lastError().c_str());
        }
        printStatus(s);
    }
    // 나가기 전에 반드시 세운다. 소멸자에서도 한 번 더 부르지만 여기서 사유를 남긴다.
    robot.estop("MQTT 모드 종료");
    link.publishEvent("shutdown", "정상 종료");
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    const std::string mode = argc > 1 ? argv[1] : "status";
    navi::Config cfg;

    // 현장 값은 navi.conf 로 덮어쓴다 (없으면 config.hpp 기본값).
    // 재빌드 없이 휠 sign·차체 치수·MQTT 주소를 고치기 위한 것이다.
    const auto cl = navi::loadConf(cfg);
    if (cl.found) {
        std::printf("설정: %s (%zu개 적용)\n", cl.path.c_str(), cl.applied.size());
        for (const auto& k : cl.unknown)
            std::printf("  ⚠ 모르는 키: %s\n", k.c_str());
    } else {
        std::printf("설정: %s 없음 — 내장 기본값 사용\n", cl.path.c_str());
    }

    try {
        navi::Robot robot(cfg);
        banner(robot);

        if (mode == "status") {
            std::printf("상태 모니터 — Ctrl+C로 종료 (모터는 돌리지 않는다)\n");
            while (!g_stop) {
                robot.tick();
                printStatus(robot.status());
            }
        } else if (mode == "spin" || mode == "turn") {
            const double rpm = argc > 2 ? std::atof(argv[2]) : 10.0;
            const double secs = argc > 3 ? std::atof(argv[3]) : 3.0;
            const bool turn = mode == "turn";
            std::printf("%s %.1f RPM, %.1f초\n", turn ? "제자리 회전" : "직진", rpm, secs);
            // turn은 좌우 부호를 뒤집는다. Config::wheels[].sign 이 장착 방향을 이미
            // 보정하므로, 여기서는 좌(FL/RL)와 우(FR/RR)에 반대 부호를 준다.
            robot.setWheelRpm(turn ? navi::WheelRpm{rpm, -rpm, rpm, -rpm}
                                   : navi::WheelRpm{rpm, rpm, rpm, rpm});
            runFor(robot, secs);
            stopAndSettle(robot, cfg);
        } else if (mode == "body") {
            const double vx = argc > 2 ? std::atof(argv[2]) : 0.0;
            const double wz = argc > 3 ? std::atof(argv[3]) : 0.0;
            const double secs = argc > 4 ? std::atof(argv[4]) : 3.0;
            std::printf("차체 속도 vx %.2f m/s, wz %.2f rad/s, %.1f초\n", vx, wz, secs);
            robot.setBodyVelocity({vx, wz});
            runFor(robot, secs);
            stopAndSettle(robot, cfg);
        } else if (mode == "wheelcheck") {
            // sign(좌우 장착 방향)을 맞추기 위한 도구.
            // 축을 하나씩 천천히 돌려주고, 사용자가 실물이 어느 쪽으로 구르는지 보고 정한다.
            const double rpm = argc > 2 ? std::atof(argv[2]) : 5.0;
            const double secs = argc > 3 ? std::atof(argv[3]) : 3.0;
            const auto& ws = robot.status().wheels;
            std::printf("축을 하나씩 %.1f RPM 으로 %.1f초씩 돌립니다.\n", rpm, secs);
            std::printf("각 축이 **전진 방향**으로 구르는지 보세요.\n");
            std::printf("반대로 구르면 navi.conf 의 그 축 sign 을 뒤집으면 됩니다.\n\n");
            for (size_t i = 0; i < ws.size() && !g_stop; ++i) {
                navi::WheelRpm one{};
                one[i] = rpm;                       // 이 축만 양의 속도
                std::printf("── [%zu/%zu] %s (ID %u, 현재 sign %+d) 회전 중...\n",
                            i + 1, ws.size(), ws[i].label, ws[i].id, cfg.wheels[i].sign);
                robot.setWheelRpm(one);
                runFor(robot, secs);
                stopAndSettle(robot, cfg);
                std::this_thread::sleep_for(std::chrono::milliseconds(900));
            }
            std::printf("\n끝났습니다. 반대로 돈 축의 sign 을 navi.conf 에서 뒤집으세요.\n");
        } else if (mode == "mqtt") {
            runMqtt(robot, cfg);
        } else if (mode == "wdtest") {
            // 워치독은 안전 경로인데 정상 동작에서는 절대 타지 않는다 — 일부러 태워 확인한다.
            // 명령만 넣고 keepalive를 부르지 않으면 watchdog(ms) 뒤에 스스로 서야 한다.
            const double rpm = argc > 2 ? std::atof(argv[2]) : 5.0;
            std::printf("워치독 시험: %.1f RPM 명령 후 keepalive 없이 대기 — %ldms 뒤 정지해야 한다\n",
                        rpm, static_cast<long>(cfg.watchdog.count()));
            robot.setWheelRpm(rpm);
            const auto t0 = navi::nuri::Clock::now();
            double tripped_at = -1.0;
            while (!g_stop &&
                   std::chrono::duration<double>(navi::nuri::Clock::now() - t0).count() < 3.0) {
                robot.tick();      // keepalive를 부르지 않는다
                const auto s = robot.status();
                if (s.watchdog_tripped && tripped_at < 0)
                    tripped_at = std::chrono::duration<double, std::milli>(
                        navi::nuri::Clock::now() - t0).count();
                printStatus(s);
            }
            std::printf("\n");
            if (tripped_at < 0) std::printf("❌ 워치독이 발동하지 않았다 — 안전 경로 점검 필요\n");
            else std::printf("✅ %.0fms 에 워치독 정지 (설정 %ldms)\n", tripped_at,
                             static_cast<long>(cfg.watchdog.count()));
            robot.stopDrive();
        } else if (mode == "act") {
            const bool ret = argc > 2 && std::strcmp(argv[2], "ret") == 0;
            const double secs = argc > 3 ? std::atof(argv[3]) : 3.0;
            if (!robot.actuator()) throw std::runtime_error("액추에이터가 없다");
            std::printf("액추에이터 %s, %.1f초\n", ret ? "RET" : "EXT", secs);
            robot.actuatorStart(ret);
            runFor(robot, secs);
            robot.actuatorStop();
        } else {
            std::fprintf(stderr,
                "모드: mqtt | status | spin RPM SEC | turn RPM SEC | body VX WZ SEC | "
                "act ext|ret SEC | wdtest [RPM] | wheelcheck [RPM] [SEC]\n");
            return 2;
        }

        robot.estop(g_stop ? "시그널" : "정상 종료");
        std::printf("\n정지 완료\n");
        return 0;
    } catch (const std::exception& e) {
        // Robot 소멸자가 estop을 한 번 더 부른다 — 예외로 빠져나가도 장치는 선다
        std::fprintf(stderr, "\n🔴 %s\n", e.what());
        return 1;
    }
}
