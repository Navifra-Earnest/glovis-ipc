// confload.hpp — navi.conf 를 읽어 Config 를 덮어쓴다.
//
// config.hpp 는 컴파일 상수라 값 하나 바꾸려면 재빌드해야 한다. 현장에서 바뀌는 값
// (휠 sign, 차체 치수, MQTT 주소 …)은 파일로 빼서 재빌드 없이 고칠 수 있게 한다.
//
// 파일이 없으면 조용히 넘어간다 — config.hpp 기본값이 그대로 쓰인다.
//
//   navi.conf 를 실행 경로에 두거나  NAVI_CONF=/경로/navi.conf
#pragma once

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "config.hpp"

namespace navi {

struct ConfLoad {
    std::string path;
    bool found = false;
    std::vector<std::string> applied;   // 적용된 키 (로그용)
    std::vector<std::string> unknown;   // 모르는 키 — 오타를 잡아준다
};

inline std::string trim(std::string s) {
    const auto ws = " \t\r\n";
    const auto b = s.find_first_not_of(ws);
    if (b == std::string::npos) return "";
    return s.substr(b, s.find_last_not_of(ws) - b + 1);
}

// wheel = 2, FR, -1   →  Config::Wheel
// Config 가 const char* 를 들고 있으므로 문자열 실체를 여기 보관해 수명을 맞춘다.
//
// ⚠ vector 를 쓰면 안 된다 — push_back 으로 재할당될 때 앞서 넘겨준 c_str() 포인터가
//   전부 무효가 되어 라벨이 깨진다(실제로 겪음). deque 는 요소 주소가 유지된다.
inline std::deque<std::string>& labelPool() {
    static std::deque<std::string> pool;
    return pool;
}

inline ConfLoad loadConf(Config& cfg, const char* explicit_path = nullptr) {
    ConfLoad r;
    if (explicit_path && *explicit_path) {
        r.path = explicit_path;
    } else if (const char* e = std::getenv("NAVI_CONF")) {
        r.path = e;
    } else {
        // 후보를 순서대로 찾는다. 실행 위치가 세 가지라 하나로 고정할 수 없다:
        //   설치본     /opt/navi/navi.conf        (systemd 가 WorkingDirectory 로 잡는다)
        //   개발 빌드  ./etc/navi.conf            (저장소 루트에서 ./navi 실행)
        //   그 외      ./navi.conf                (바이너리 옆에 둔 경우)
        for (const char* c : {"navi.conf", "etc/navi.conf", "/opt/navi/navi.conf"}) {
            if (std::ifstream(c)) { r.path = c; break; }
            r.path = c;                       // 다 없으면 마지막 후보를 사유에 남긴다
        }
    }

    std::ifstream f(r.path);
    if (!f) return r;                  // 없으면 기본값 그대로
    r.found = true;

    std::vector<Config::Wheel> wheels;
    std::string line;
    while (std::getline(f, line)) {
        if (const auto h = line.find('#'); h != std::string::npos) line = line.substr(0, h);
        line = trim(line);
        if (line.empty()) continue;
        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string k = trim(line.substr(0, eq));
        const std::string v = trim(line.substr(eq + 1));
        if (k.empty() || v.empty()) continue;

        auto num = [&](double& dst) { dst = std::atof(v.c_str()); r.applied.push_back(k); };
        auto inum = [&](int& dst) { dst = std::atoi(v.c_str()); r.applied.push_back(k); };
        auto ms = [&](Ms& dst) { dst = Ms(std::atoi(v.c_str())); r.applied.push_back(k); };

        if      (k == "rs485_baud")       inum(cfg.rs485_baud);
        else if (k == "de_guard_us")      num(cfg.de_guard_us);
        // 🔴 보드가 바뀌면 여기가 달라진다 — 재빌드 없이 고칠 수 있어야 한다.
        //    40핀 12번(RS485_DE)의 GPIO 가 보드마다 다르다:
        //      ROCK 3A (RK3568)  GPIO3_A3 → gpiochip3 line 3
        //      ROCK 5A (RK3588S) GPIO4_A1 → gpiochip4 line 1   (실기 gpiofind 로 확인)
        //    칩 번호는 뱅크 번호와 반드시 같지는 않다(커널 프로브 순서). `gpiofind PIN_12` 로 실측할 것.
        else if (k == "de_line")          { unsigned u = std::strtoul(v.c_str(), nullptr, 10);
                                            cfg.de_line = u; r.applied.push_back(k); }
        else if (k == "gear_ratio")       num(cfg.gear_ratio);
        else if (k == "max_wheel_rpm")    num(cfg.max_wheel_rpm);
        else if (k == "wheel_radius_m")   num(cfg.wheel_radius_m);
        else if (k == "track_width_m")    num(cfg.track_width_m);
        else if (k == "wheel_base_m")     num(cfg.wheel_base_m);
        else if (k == "slip_factor")      num(cfg.slip_factor);
        else if (k == "decel_secs")       num(cfg.decel_secs);
        else if (k == "overcurrent_a")    num(cfg.overcurrent_a);
        else if (k == "overcurrent_hits") inum(cfg.overcurrent_hits);
        else if (k == "current_window")   inum(cfg.current_window);
        else if (k == "stream_fps")       inum(cfg.mqtt.stream_fps);
        else if (k == "tof_baud")         inum(cfg.tof.baud);
        else if (k == "tof_min_strength") inum(cfg.tof.min_strength);
        else if (k == "tof_i2c_period_ms") inum(cfg.tof.i2c_period_ms);
        // 🔴 HAT V1 은 UART, V2 는 I2C 다. 보드를 바꾸면 반드시 함께 바꿀 것.
        else if (k == "tof_i2c")          { cfg.tof.i2c = (v != "0" && v != "false"); r.applied.push_back(k); }
        else if (k == "tof_i2c_addr")     { cfg.tof.i2c_addr = (int)std::strtol(v.c_str(), nullptr, 0);
                                            r.applied.push_back(k); }   // 0x10 표기를 받는다
        else if (k == "tof_i2c_dev") {
            labelPool().push_back(v);
            cfg.tof.i2c_dev = labelPool().back().c_str();
            r.applied.push_back(k);
        }
        else if (k == "stream_enabled")   { cfg.mqtt.stream_enabled = (v != "0" && v != "false"); r.applied.push_back(k); }
        else if (k == "stream_thermal")   { cfg.mqtt.stream_thermal = (v != "0" && v != "false"); r.applied.push_back(k); }
        else if (k == "stream_ir")        { cfg.mqtt.stream_ir      = (v != "0" && v != "false"); r.applied.push_back(k); }
        else if (k == "tof_enabled")      { cfg.tof.enabled         = (v != "0" && v != "false"); r.applied.push_back(k); }
        else if (k == "tof_port") {
            labelPool().push_back(v);
            cfg.tof.port = labelPool().back().c_str();
            r.applied.push_back(k);
        }
        else if (k == "tick_period_ms")   ms(cfg.tick_period);
        else if (k == "watchdog_ms")      ms(cfg.watchdog);
        else if (k == "poll_timeout_ms")  ms(cfg.poll_timeout);
        else if (k == "motor_fail_limit") inum(cfg.motor_fail_limit);
        // 🔴 HAT 리비전에 따라 홀 센서 핀이 다르다 (V1 핀35·31 → V2 핀38·40).
        //    틀리면 액추에이터는 도는데 홀 카운트만 0 이라 원인을 찾기 어렵다.
        else if (k == "hat_rev")          inum(cfg.hat_rev);
        else if (k == "mqtt_port")        inum(cfg.mqtt.port);
        else if (k == "actuator_duty")    num(cfg.actuator_duty);
        else if (k == "thermal_colormap") inum(cfg.thermal.colormap);
        else if (k == "thermal_cdc_port") {
            labelPool().push_back(v);
            cfg.thermal.cdc_port = labelPool().back().c_str();
            r.applied.push_back(k);
        }
        else if (k == "thermal_use_sdk") { cfg.thermal.use_sdk = (v != "0" && v != "false"); r.applied.push_back(k); }
        else if (k == "thermal_enabled") {
            cfg.thermal.enabled = (v != "0" && v != "false");
            r.applied.push_back(k);
        } else if (k == "mqtt_host" || k == "mqtt_prefix") {
            labelPool().push_back(v);
            (k == "mqtt_host" ? cfg.mqtt.host : cfg.mqtt.prefix) = labelPool().back().c_str();
            r.applied.push_back(k);
        } else if (k == "de_chip" || k == "rs485_port") {
            // labelPool 은 deque 다 — vector 면 재할당 때 c_str() 이 무효화된다
            labelPool().push_back(v);
            (k == "de_chip" ? cfg.de_chip : cfg.rs485_port) = labelPool().back().c_str();
            r.applied.push_back(k);
        } else if (k == "wheel") {
            // "ID, 라벨, sign"
            std::stringstream ss(v);
            std::string id, label, sign;
            std::getline(ss, id, ',');
            std::getline(ss, label, ',');
            std::getline(ss, sign, ',');
            label = trim(label);
            if (label.empty()) label = "W" + trim(id);
            labelPool().push_back(label);
            wheels.push_back({static_cast<uint8_t>(std::atoi(trim(id).c_str())),
                              labelPool().back().c_str(),
                              std::atoi(trim(sign).c_str()) < 0 ? -1 : +1});
        } else {
            r.unknown.push_back(k);
        }
    }
    if (!wheels.empty()) {
        cfg.wheels = std::move(wheels);
        r.applied.push_back("wheel×" + std::to_string(cfg.wheels.size()));
    }
    return r;
}

}  // namespace navi
