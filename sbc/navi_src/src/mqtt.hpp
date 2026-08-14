// mqtt.hpp — 상위 통신. libmosquitto + JSON.
//
// 스레드 경계가 이 파일의 핵심이다:
//   mosquitto_loop_start()가 만든 네트워크 스레드에서 메시지 콜백이 온다.
//   Robot은 단일 스레드 전제로 짜여 있으므로, 콜백에서 Robot을 직접 건드리면 안 된다.
//   → 콜백은 명령을 큐에 넣기만 하고, 메인 루프가 poll()로 꺼내 적용한다.
//
// 워치독과의 관계: 상위에서 명령이 올 때마다 cmd_at_이 갱신되므로, 상위 서버나
// 네트워크가 죽으면 watchdog(500ms) 뒤에 스스로 선다. 그게 워치독을 둔 이유다.
//
// 링크: -lmosquitto
#pragma once

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <mosquitto.h>
#include <nlohmann/json.hpp>

#include "config.hpp"
#include "robot.hpp"   // RobotStatus — 코어는 MQTT를 모른다 (의존 방향 한쪽)

namespace navi {

using Json = nlohmann::json;

// ── 상위에서 오는 명령 ─────────────────────────────────────────────
struct Command {
    enum class Kind { Wheel, Body, Actuator, Stop, Estop, Reset, ThermalFFC, Stream, Unknown };
    Kind kind = Kind::Unknown;

    WheelRpm wheel{};          // Kind::Wheel
    bool stream_on = false;    // Kind::Stream — 프레임 전송 켜기/끄기
    int  stream_fps = 0;       // 0 이면 설정값 유지
    BodyVel body{};            // Kind::Body
    double ramp = 0.5;
    bool actuator_ret = false; // Kind::Actuator
    double actuator_duty = -1.0;
    std::string reason;        // Kind::Estop
    std::string error;         // 파싱 실패 사유 (Kind::Unknown)
};

class MqttLink {
public:
    explicit MqttLink(const Config& cfg) : cfg_(cfg), m_(cfg.mqtt) {
        // 환경변수로 브로커를 덮어쓸 수 있게 한다 — 현장에서 재빌드 없이 바꾸려고.
        if (const char* h = std::getenv("NAVI_MQTT_HOST")) host_ = h;
        else host_ = m_.host;
        if (const char* p = std::getenv("NAVI_MQTT_PORT")) port_ = std::atoi(p);
        else port_ = m_.port;

        mosquitto_lib_init();
        mo_ = mosquitto_new(m_.client_id, true, this);
        if (!mo_) throw std::runtime_error("mosquitto_new 실패");

        if (m_.user) mosquitto_username_pw_set(mo_, m_.user, m_.pass);
        mosquitto_connect_callback_set(mo_, onConnect);
        mosquitto_disconnect_callback_set(mo_, onDisconnect);
        mosquitto_message_callback_set(mo_, onMessage);

        // LWT: 연결이 끊기면 브로커가 대신 online=false를 발행한다.
        // 상위가 "장치가 살아있는지"를 폴링하지 않아도 되게 하는 장치.
        const auto lwt = topic("state/online");
        const std::string off = R"({"online":false})";
        mosquitto_will_set(mo_, lwt.c_str(), static_cast<int>(off.size()), off.data(),
                           m_.qos_cmd, true);

        // 브로커가 아직 없어도 기동은 계속한다 — 재연결은 loop가 알아서 한다.
        const int rc = mosquitto_connect_async(mo_, host_.c_str(), port_, m_.keepalive_s);
        if (rc != MOSQ_ERR_SUCCESS) last_error_ = mosquitto_strerror(rc);
        mosquitto_loop_start(mo_);
    }

    ~MqttLink() {
        if (mo_) {
            publish("state/online", R"({"online":false})", true);
            mosquitto_disconnect(mo_);
            mosquitto_loop_stop(mo_, false);
            mosquitto_destroy(mo_);
        }
        mosquitto_lib_cleanup();
    }
    MqttLink(const MqttLink&) = delete;
    MqttLink& operator=(const MqttLink&) = delete;

    bool connected() const { return connected_; }
    const std::string& lastError() const { return last_error_; }
    std::string broker() const { return host_ + ":" + std::to_string(port_); }

    // 메인 루프에서 부른다. 쌓인 명령을 하나 꺼낸다 (없으면 nullopt).
    std::optional<Command> nextCommand() {
        std::lock_guard<std::mutex> lk(mu_);
        if (q_.empty()) return std::nullopt;
        auto c = q_.front();
        q_.pop_front();
        return c;
    }

    void publish(const std::string& sub, const std::string& payload, bool retain = false) {
        if (!mo_) return;
        const auto t = topic(sub);
        mosquitto_publish(mo_, nullptr, t.c_str(), static_cast<int>(payload.size()),
                          payload.data(), m_.qos_state, retain);
    }

    // 이벤트는 상태와 달리 놓치면 안 된다 (정지 사유 등) → QoS는 명령과 같게.
    void publishEvent(const std::string& kind, const std::string& detail) {
        if (!mo_) return;
        const Json j{{"event", kind}, {"detail", detail}};
        const auto s = j.dump();
        const auto t = topic("event");
        mosquitto_publish(mo_, nullptr, t.c_str(), static_cast<int>(s.size()), s.data(),
                          m_.qos_cmd, false);
    }

    // ── 알람 ────────────────────────────────────────────────────────
    //
    // event 와 나누는 이유: event 는 "무슨 일이 있었다"는 기록이고,
    // alarm 은 **사람이 조치해야 하는 상태**다. 상위 대시보드가 다르게 다뤄야 한다.
    //
    // retain=true 로 보낸다 — 상위가 나중에 붙어도 현재 걸린 알람을 바로 안다.
    // 해제될 때 같은 키로 active=false 를 보내 지운다.
    enum class Sev { Info, Warn, Critical };

    void publishAlarm(const std::string& key, Sev sev, const std::string& msg,
                      bool active = true) {
        if (!mo_) return;
        const char* sv = sev == Sev::Critical ? "critical"
                       : sev == Sev::Warn     ? "warn" : "info";
        const Json j{{"key", key}, {"severity", sv}, {"active", active}, {"message", msg}};
        const auto s = j.dump();
        const auto t = topic("alarm/" + key);
        mosquitto_publish(mo_, nullptr, t.c_str(), static_cast<int>(s.size()), s.data(),
                          m_.qos_cmd, true);   // retain
    }

    // 이미지 프레임. 토픽마다 따로 보낸다 — 상위가 필요한 것만 구독하게.
    // QoS 0: 프레임은 최신만 의미가 있고, 재전송이 밀리면 오히려 지연이 된다.
    void publishFrame(const std::string& key, const void* data, size_t len) {
        if (!mo_ || !data || !len) return;
        const auto t = topic("frame/" + key);
        mosquitto_publish(mo_, nullptr, t.c_str(), static_cast<int>(len), data, 0, false);
    }

private:
    std::string topic(const std::string& sub) const {
        return std::string(m_.prefix) + "/" + sub;
    }

    static void onConnect(mosquitto* mo, void* self, int rc) {
        auto* me = static_cast<MqttLink*>(self);
        if (rc != 0) {
            me->last_error_ = "connect rc=" + std::to_string(rc);
            return;
        }
        me->connected_ = true;
        me->last_error_.clear();
        // 재연결 때마다 다시 구독해야 한다 (세션을 clean으로 잡았으므로)
        for (const char* s : {"cmd/wheel", "cmd/body", "cmd/actuator",
                              "cmd/stop", "cmd/estop", "cmd/reset", "cmd/thermal/ffc", "cmd/stream"})
            mosquitto_subscribe(mo, nullptr, me->topic(s).c_str(), me->m_.qos_cmd);
        const std::string on = R"({"online":true})";
        mosquitto_publish(mo, nullptr, me->topic("state/online").c_str(),
                          static_cast<int>(on.size()), on.data(), me->m_.qos_cmd, true);
    }

    static void onDisconnect(mosquitto*, void* self, int rc) {
        auto* me = static_cast<MqttLink*>(self);
        me->connected_ = false;
        if (rc != 0) me->last_error_ = "예기치 않은 연결 끊김";
    }

    // ⚠ 여기는 mosquitto 네트워크 스레드다. Robot을 건드리지 말고 큐에만 넣는다.
    static void onMessage(mosquitto*, void* self, const mosquitto_message* msg) {
        auto* me = static_cast<MqttLink*>(self);
        const std::string t(msg->topic ? msg->topic : "");
        const std::string p(static_cast<const char*>(msg->payload),
                            msg->payloadlen > 0 ? msg->payloadlen : 0);
        me->enqueue(me->parse(t, p));
    }

    void enqueue(Command c) {
        std::lock_guard<std::mutex> lk(mu_);
        // 큐가 밀리면 오래된 명령을 버린다 — 최신 속도 명령이 중요하고,
        // 밀린 옛 명령을 뒤늦게 적용하면 오히려 위험하다.
        if (q_.size() >= kMaxQueue) q_.pop_front();
        q_.push_back(std::move(c));
    }

    Command parse(const std::string& t, const std::string& payload) {
        Command c;
        const std::string sub = t.rfind(std::string(m_.prefix) + "/", 0) == 0
                              ? t.substr(std::strlen(m_.prefix) + 1) : t;

        // 페이로드가 없어도 되는 명령들
        if (sub == "cmd/stop")  { c.kind = Command::Kind::Stop;  return c; }
        if (sub == "cmd/reset") { c.kind = Command::Kind::Reset; return c; }
        // 열화상 셔터 보정(FFC). 온도가 드리프트하면 상위에서 걸어준다.
        if (sub == "cmd/thermal/ffc") { c.kind = Command::Kind::ThermalFFC; return c; }
        if (sub == "cmd/stream") {
            // {"on":true, "fps":10}  — 상위가 볼 때만 켠다. 아무도 안 보는데 흘리면 낭비다
            c.kind = Command::Kind::Stream;
            Json js;
            if (!payload.empty()) { try { js = Json::parse(payload); } catch (...) {} }
            c.stream_on  = js.value("on", true);
            c.stream_fps = js.value("fps", 0);
            return c;
        }

        Json j;
        if (!payload.empty()) {
            j = Json::parse(payload, nullptr, false);   // 예외 대신 discarded
            if (j.is_discarded()) {
                c.error = "JSON 파싱 실패: " + t;
                return c;
            }
        }

        if (sub == "cmd/estop") {
            c.kind = Command::Kind::Estop;
            c.reason = j.value("reason", "상위 요청");
            return c;
        }
        if (sub == "cmd/wheel") {
            // {"rpm":[20,20,20,20], "ramp":0.5}  — 축 수보다 적게 와도 앞부터 채운다
            if (!j.contains("rpm") || !j["rpm"].is_array()) {
                c.error = "cmd/wheel: rpm 배열이 필요하다";
                return c;
            }
            c.kind = Command::Kind::Wheel;
            const auto& a = j["rpm"];
            for (size_t i = 0; i < c.wheel.size() && i < a.size(); ++i)
                c.wheel[i] = a[i].get<double>();
            c.ramp = j.value("ramp", 0.5);
            return c;
        }
        if (sub == "cmd/body") {
            c.kind = Command::Kind::Body;
            c.body.vx = j.value("vx", 0.0);
            c.body.wz = j.value("wz", 0.0);
            c.ramp = j.value("ramp", 0.5);
            return c;
        }
        if (sub == "cmd/actuator") {
            // {"dir":"ext"|"ret", "duty":30}
            c.kind = Command::Kind::Actuator;
            c.actuator_ret = j.value("dir", std::string("ext")) == "ret";
            c.actuator_duty = j.value("duty", -1.0);
            return c;
        }
        c.error = "알 수 없는 토픽: " + t;
        return c;
    }

    static constexpr size_t kMaxQueue = 32;

    const Config& cfg_;
    const Config::Mqtt& m_;
    std::string host_;
    int port_ = 1883;
    mosquitto* mo_ = nullptr;
    std::atomic<bool> connected_{false};
    std::string last_error_;

    std::mutex mu_;
    std::deque<Command> q_;
};

// double 을 자리수 잘라 싣는다.
//
// 🔴 그냥 넣으면 nlohmann 이 왕복 정확도를 보장하려고 17자리를 다 쓴다:
//        "current":0.03519061583577713 , "fps":4.559635347139115
//    전류를 소수 14자리까지 보낼 이유가 없다. 5Hz 로 흘리는 상태 토픽에서
//    이 낭비가 payload 의 절반을 넘는다.
//    센서 분해능을 넘는 자리는 어차피 노이즈다 — 전류/온도 2자리, fps 1자리면 충분하다.
inline double r2(double v) { return std::round(v * 100.0) / 100.0; }
inline double r1(double v) { return std::round(v * 10.0) / 10.0; }

// 발행할 가치가 있는 변화만 뽑은 요약.
//
// toJson 전체를 비교하면 안 된다 — ticks·tick_ms·fps·frames 는 매 tick 달라져서
// "변화 없음"이 영영 성립하지 않는다. 여기서는 **상위가 판단에 쓰는 값**만 넣는다:
// 축이 도는지, 살아있는지, e-stop 인지, 액추에이터가 뭘 하는지, 카메라가 열렸는지.
// 전류·온도는 노이즈로 계속 흔들리므로 넣지 않는다(heartbeat 로 어차피 나간다).
inline std::string stateDigest(const RobotStatus& s) {
    std::string d;
    d.reserve(96);
    d += s.estopped ? "E" : "-";
    d += s.watchdog_tripped ? "W" : "-";
    d += std::to_string(s.wheels_alive);
    for (const auto& w : s.wheels) {
        d += '|';
        d += std::to_string(static_cast<int>(std::lround(w.rpm)));   // 정수 RPM 만
        d += w.alive ? 'a' : 'x';
    }
    d += '|';
    d += std::to_string(static_cast<int>(s.actuator_state));
    d += std::to_string(s.actuator_position);
    for (const auto& c : s.cams) d += c.open ? 'o' : 'x';
    d += s.drive_ok ? 'D' : '-';
    // 거리는 10cm 단위로 뭉뚱그린다 — 1cm 흔들림마다 발행하면 의미가 없다
    if (s.tof_present) { d += 'T'; d += std::to_string(s.tof.dist_cm / 10); }
    return d;
}

// ── 상태 → JSON ────────────────────────────────────────────────────
// 상위 대시보드가 바로 꽂아 쓸 수 있는 평평한 구조로 둔다.
inline std::string toJson(const RobotStatus& s) {
    Json j;
    j["ticks"] = s.ticks;
    j["tick_ms"] = r2(s.tick_ms);
    j["tick_max_ms"] = r2(s.tick_max_ms);
    j["estop"] = s.estopped;
    j["estop_reason"] = s.estop_reason;
    j["watchdog"] = s.watchdog_tripped;

    j["wheels_alive"] = s.wheels_alive;
    for (const auto& w : s.wheels) {
        j["wheels"].push_back({
            {"label", w.label},
            {"id", w.id},
            {"rpm", r1(w.rpm)},
            {"alive", w.alive},
            // 전류는 순간 피크 샘플이라 신뢰할 수 없다 — 상위에서 임계 판정에 쓰지 말 것
            {"amp_raw", r2(w.amp)},
            // 판정에 쓸 값은 이쪽이다 — amp_raw 는 순간 피크라 스파이크가 튄다
            {"amp", r2(w.amp_median)},
            {"fail", w.consecutive_fail},
        });
    }

    j["kinematics"] = s.kinematics;
    if (s.kinematics) {
        j["body"]["vx"] = r2(s.body.vx);
        j["body"]["wz"] = r2(s.body.wz);
    }

    j["actuator"]["present"] = s.actuator_present;
    if (s.actuator_present) {
        j["actuator"]["state"] = toString(s.actuator_state);
        j["actuator"]["position"] = s.actuator_position;
        j["actuator"]["current"] = r2(s.actuator_current);
    }

    j["tof"]["present"] = s.tof_present;
    if (s.tof_present) {
        j["tof"]["dist_cm"]  = s.tof.dist_cm;
        j["tof"]["strength"] = s.tof.strength;
        j["tof"]["temp_c"]   = r1(s.tof.temp_c);
        // 신호강도 미달이면 거리값을 믿으면 안 된다 — 상위가 매번 판단하지 않게 여기서 가린다
        j["tof"]["valid"]    = s.tof.valid;
        j["tof"]["fps"]      = r1(s.tof_status.fps);
        j["tof"]["bad"]      = s.tof_status.bad;
    }

    // 구동계가 없어도 프로그램은 뜬다. 이 값으로 상위가 구동 가능 여부를 안다.
    j["drive_ok"] = s.drive_ok;
    if (!s.drive_ok) j["drive_error"] = s.drive_error;

    for (const auto& c : s.cams) {
        j["cams"].push_back({
            {"key", c.key},
            {"open", c.open},
            {"fps", r1(c.fps)},
            {"frames", c.frames},
            {"drops", c.drops},
            {"reopens", c.reopens},
            {"error", c.error},
        });
    }

    // IR 영상 송출 상태 — IPC 가 "지금 붙어서 볼 수 있나"를 여기서 판단한다.
    j["video"]["listening"] = s.video.listening;
    j["video"]["clients"] = s.video.clients;
    if (s.video.clients > 0) {
        j["video"]["fps"] = r1(s.video.fps);
        j["video"]["frames"] = s.video.frames;
        j["video"]["dropped"] = s.video.dropped;
    }
    if (!s.video.error.empty()) j["video"]["error"] = s.video.error;

    // 열화상은 온도 요약만 싣는다. 영상(80x60 BMP 14KB)까지 넣으면 상태 토픽이 무거워지고
    // 5Hz로 흘리면 브로커에 부담이다 — 이미지가 필요하면 별도 토픽으로 낮은 주기에 보낸다.
    j["thermal"]["present"] = s.thermal_present;
    if (s.thermal_present) {
        j["thermal"]["unit"] = "C";
        j["thermal"]["lo"] = r2(s.thermal.lo);
        j["thermal"]["avg"] = r2(s.thermal.avg);
        j["thermal"]["hi"] = r2(s.thermal.hi);
        j["thermal"]["center"] = r2(s.thermal.center);
        j["thermal"]["valid"] = s.thermal.valid;
        j["thermal"]["fps"] = r1(s.thermal_status.fps);
        j["thermal"]["frames"] = s.thermal_status.frames;
        j["thermal"]["drops"] = s.thermal_status.drops;
        j["thermal"]["firmware"] = s.thermal_status.firmware;
    } else {
        j["thermal"]["error"] = s.thermal_status.error;
    }
    return j.dump();
}

}  // namespace navi
