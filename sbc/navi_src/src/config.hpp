// config.hpp — 현장에서 바뀌는 값은 전부 여기 모은다.
//
// 실측·조정이 필요한 값에는 왜 그 값인지 근거를 달아둔다. 코드 어딘가에 흩어진 상수를
// 찾아다니지 않으려고 만든 파일이니, 새 상수는 되도록 이 안에 둘 것.
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <vector>

#include "uvccam.hpp"   // fourcc

namespace navi {

using Ms = std::chrono::milliseconds;

struct Config {
    // ── RS485 구동 모터 (누리로봇 SB60PB60) ──────────────────────
    const char* rs485_port = "/dev/ttyS2";
    // 🔴 모터 EEPROM도 115200(Mode 0x07, 코드 0x0D)으로 맞춰야 한다.
    //    출고 기본은 9600이고, 그 속도로는 축당 왕복이 15ms라 4축이면 60ms —
    //    20ms 제어 주기를 못 맞춘다. 115200이면 축당 1.4ms, 4축 5.6ms로 여유가 생긴다.
    //    모터를 1대씩 연결해 nuri_ping.py 로 바꾼 뒤 이 값을 고칠 것.
    int rs485_baud = 115200;
    // 🔴 보드마다 다르다. `gpiofind PIN_12` 로 실측할 것 — 계산하지 말 것.
    //    5A(RK3588S) GPIO4_A1 → gpiochip4:1   ← 현재 실기
    //    3A(RK3568)  GPIO3_A3 → gpiochip3:3
    //    5A 에서 3A 값을 쓰면 **이더넷 RGMII 선을 토글해 네트워크가 죽는다.**
    const char* de_chip = "gpiochip4";
    unsigned de_line = 1;
    // DE 해제 여유. 보레이트마다 안전 구간이 다르다 (실측):
    //     9600bps : 100~500µs 가 5/5, 800µs부터 응답 첫 바이트 잘림 → 200 사용
    //   115200bps : 300µs 만 5/5. 200µs는 4/5로 가끔 놓치고(감속비가 0.0으로 읽힘),
    //               500µs부터 첫 바이트가 잘린다 → 창이 좁으니 300 을 지켜야 한다
    // 보레이트를 바꾸면 nuri_ping.py --de-sweep 으로 다시 재고 이 값을 맞출 것.
    double de_guard_us = 200.0;   // 5A 실측. 3A 는 300
    double gear_ratio = 16.0;      // 모터 EEPROM 값과 일치해야 한다 (다르면 생성자가 던짐)

    // 휠 4축.
    //   id    — 모터에 할당한 RS485 ID. 출고는 전부 0이라 1대씩 붙여 부여해야 한다
    //   sign  — 좌우 장착 방향이 반대라 부호를 뒤집는다. 전진 시 네 바퀴가 같은 방향으로
    //           굴러가도록 실물 보고 맞출 것
    struct Wheel {
        uint8_t id;
        const char* label;
        int sign;
    };
    // 지금은 실물이 1대(ID 0)뿐이라 한 축만 올린다. 4축이 되면 아래 주석을 풀고
    // ID를 부여할 것 — 출고는 전부 0이라 **한 버스에 물린 뒤에는 설정할 수 없다**.
    // 1대씩 연결해서 nuri_ping.py 로 ID를 준 다음 합쳐야 한다.
    // 운동학(setBodyVelocity/odometry)은 4축이 다 올라와야 열린다.
    std::vector<Wheel> wheels{
        // 실물 2대 연결됨 (2026-07-31): ID 2, 3. 나머지는 붙이면서 1대씩 부여할 것.
        // sign(±1)은 좌우 장착 방향 — 실물 보고 전진 시 같은 방향으로 굴러가게 맞출 것.
        // 2026-08-07 실물 확인. 4대 모두 브레이크 미해제로 손상돼 교체 대기 중이라
        // 현재는 navi.conf 의 wheel 정의가 실제 구성이다.
        {3, "FR", -1},
        {2, "RL", +1},
        {4, "RR", -1},
    };

    double max_wheel_rpm = 150.0;  // 출력축. 48V·1/16에서 정격 187RPM이라 여유를 둔 값

    // ── 차체 치수 — 운동학용. 실측해서 채울 것 ────────────────────
    // 0이면 Drive::setBodyVelocity() / odometry()가 예외를 던진다.
    // 채우기 전에도 setWheelRpm()으로 축별 제어는 된다.
    double wheel_radius_m = 0.0;   // 바퀴 반지름
    double track_width_m = 0.0;    // 좌우 바퀴 중심 간 거리
    double wheel_base_m = 0.0;     // 전후 축 간 거리 (스키드스티어 회전 저항 보정용)
    // 스키드스티어는 바퀴가 미끄러지며 돌기 때문에 이론값보다 덜 회전한다.
    // 실측으로 보정하는 계수 — 1.0에서 시작해 제자리 회전 시켜보고 맞춘다.
    double slip_factor = 1.0;

    // ── 액추에이터 (STSPIN948) ───────────────────────────────────
    bool use_actuator = true;
    int hat_rev = 0;               // 0 = 환경변수 NAVI_HAT_REV 참조, 없으면 V1
    double actuator_duty = 30.0;   // 기본 구동 듀티 [%]

    // ── 카메라 ───────────────────────────────────────────────────
    // name은 v4l2 카드 이름의 부분일치. USB 꽂는 순서로 /dev/videoN이 바뀌므로
    // 경로가 아니라 이름으로 찾는다. w/h/fourcc가 0이면 장치가 보고한 첫 조합을 쓴다.
    struct Cam {
        const char* key;      // 로그·조회용 짧은 이름
        const char* match;    // v4l2 카드 이름 부분일치
        uint32_t width, height, fourcc, fps;
        bool required;        // false면 없어도 기동 계속 (degraded)
    };
    // 실측 확인 (2026-07-30):
    //   See3CAM_CU27 (2560:c12c) — UYVY/MJPG × 1920x1080, 1280x720, 640x480
    //   TMC80F       (28e9:080b) — Y16 80x60 @ 9fps  ※ 스트림이 안 켜진다, 아래 주석 참고
    // 열화상은 여기 넣지 않는다 — TmSDK로 따로 다룬다 (위 Thermal 참고).
    std::array<Cam, 1> cams{{
        // FHD@60 UYVY는 249MB/s — USB3 포트에 단독으로 꽂아야 한다
        {"ir", "See3CAM", 1920, 1080, cam::kUYVY, 60, false},
    }};

    // ── 열화상 (ThermoEye TMC80, TmSDK 사용) ─────────────────────
    // UVC로 직접 열면 프레임이 안 나온다 — SDK가 CDC 제어까지 함께 해줘야 스트림이 켜진다.
    // 그래서 uvccam.hpp 가 아니라 thermal.hpp 로 따로 다룬다.
    struct Thermal {
        bool enabled = true;
        int width = 80, height = 60;
        int colormap = 2;          // 2 = Jet (−1 흑백, 4 Rainbow …)
        // FFC(셔터 보정)를 치는 제어 채널. TmSDK 없이 직접 명령한다.
        // UVC(영상)와 별개 인터페이스라 v4l2 로 읽는 중에도 쓸 수 있다.
        const char* cdc_port = "/dev/ttyACM0";
        bool ffc_on_open = true;   // 열 때 셔터 보정 1회
        // 🔴 SDK 를 쓸지. 기본 false — v4l2 로 직접 읽는다.
        //    SDK Open() 이 UVC 장치를 점유하는데 정작 QueryFrame 은 블로킹된다
        //    (40회에 1회 성공). v4l2 는 7.7fps 로 멀쩡하다.
        //    true 로 두면 SDK 를 쓰지만 프레임을 못 받을 수 있다.
        bool use_sdk = false;
        bool keep_image = false;   // 컬러맵 이미지를 들고 있을지. 온도만 쓸 거면 false가 가볍다
        // (예전엔 required 로 기동을 막았다. 지금은 어떤 장치가 없어도 뜬다 —
        //  기동이 막히면 원인조차 못 본다. 없는 건 상태·알람으로 알린다)
    } thermal;

    // ── MQTT (상위 통신) ─────────────────────────────────────────
    // 브로커는 상위 서버에 있다. 주소를 받으면 host만 고치면 된다.
    // 환경변수 NAVI_MQTT_HOST / NAVI_MQTT_PORT 로도 덮어쓸 수 있다 (mqtt.hpp 참고).
    struct Mqtt {
        const char* host = "127.0.0.1";   // ← 상위 서버 주소로 교체
        int port = 1883;
        const char* client_id = "navi";
        const char* user = nullptr;        // 인증 없으면 nullptr
        const char* pass = nullptr;
        const char* prefix = "navi";       // 토픽 앞머리. 장치가 늘면 "navi/<id>" 로
        int keepalive_s = 15;              // MQTT 자체 keepalive (브로커 연결 감시)
        Ms state_period{200};              // 상태 발행 주기 (5Hz)
        int qos_cmd = 1;                   // 명령은 최소 한 번 도착해야 한다
        int qos_state = 0;                 // 상태는 최신만 중요하다

        // ── 프레임 전송 (초당 N장) ────────────────────────────────
        //
        // 상위가 붙어 있을 때만 보낸다. 아무도 안 보는데 흘리면 대역폭만 먹는다 —
        // 구독자 유무는 `cmd/stream` 으로 상위가 켜고 끈다.
        //
        // ⚠ 큰 이미지를 MQTT 로 밀면 브로커 큐가 밀린다. IR 1080p JPEG 이 200KB 인데
        //   10fps 면 2MB/s 다. 그래서 **열화상(80x60, 14KB)과 ToF 만 기본으로 켜고**,
        //   IR 영상은 MediaMTX(RTSP/WebRTC)로 보내는 걸 권한다 — 그쪽이 영상용이다.
        bool stream_enabled = false;       // cmd/stream 으로 켠다
        int stream_fps = 10;               // 초당 프레임 수
        bool stream_thermal = true;        // 열화상 이미지 (BMP 14KB)
        bool stream_ir = false;            // IR 이미지 — 기본 끔 (크다)
        int stream_ir_quality = 60;        // IR JPEG 품질 (켤 때만)
    } mqtt;

    // ── 주기·안전 ────────────────────────────────────────────────
    Ms tick_period{20};      // 50Hz. 115200에서 축당 1.4ms라 4축 폴링해도 여유가 있다
    Ms watchdog{500};        // 이 시간 동안 새 명령이 없으면 자동 정지
    Ms motor_timeout{100};   // 이 시간 넘게 갱신이 없는 축 값은 '모름'으로 본다 (moving() 판정)

    // 평상시 정지에 쓸 감속 시간(초). 모터가 하드웨어로 가감속한다(Mode 0x03).
    // 0 이면 즉시 차단 — 관성으로만 서기 때문에 뚝 선다.
    // ⚠ e-stop·워치독 트립은 이 값과 무관하게 항상 즉시 차단이다. 안전이 우선이다.
    double decel_secs = 0.8;

    // ── ToF 거리센서 (TF-Luna) ───────────────────────────────────
    //
    // 🔴 HAT 리비전에 따라 연결 방식이 다르다. 보드를 바꾸면 i2c 를 함께 바꿀 것.
    //
    //   HAT V1  UART  40핀 7·29 (UART4_M2) → /dev/ttyS4
    //                 센서가 100Hz 로 자동 송신하므로 RX 만 있으면 된다
    //   HAT V2  I2C   40핀 3·5 (I2C8_M2)   → /dev/i2c-8, 주소 0x10
    //                 호스트가 폴링한다. 커널 오버레이 rk3588-i2c8-m2 필요
    //
    // 40핀에 3A·5A 공통 UART 쌍이 (8,10) 하나뿐인데 RS485 가 쓰고 있어서,
    // V2 에서 ToF 를 I2C 로 옮겼다 (TF-Luna 가 I2C 를 네이티브 지원한다).
    struct Tof {
        bool enabled = true;
        bool i2c = false;                    // false=UART(V1), true=I2C(V2)
        const char* port = "/dev/ttyS4";     // UART 일 때
        int baud = 115200;                   // TF-Luna 공장 기본값
        const char* i2c_dev = "/dev/i2c-8";  // I2C 일 때
        int i2c_addr = 0x10;                 // TF-Luna 기본 슬레이브 주소
        int i2c_period_ms = 10;              // 폴링 주기. 센서 갱신이 100Hz 다
        // 신호강도가 이보다 낮으면 거리값을 못 믿는다 (반사가 약한 것).
        // 데이터시트 권장 하한이 100 이다.
        int min_strength = 100;
        // required 없음 — 없어도 기동한다 (위 카메라 주석 참고)
    } tof;

    // ── IR 영상 송출 (H.264) ─────────────────────────────────────
    //
    // 🔴 인코딩이 navi 안에 있어야 하는 이유: UVC 는 한 프로세스만 연다(실측 EBUSY).
    //    외부 인코더를 쓰면 navi 가 카메라를 놓아야 하고, 그러면 IR·열화상·ToF 를
    //    계속 보낼 수 없다. 여기서 인코딩하면 센서 발행과 영상 송출이 동시에 된다.
    //
    // 원본 1080p UYVY 는 249MB/s 라 그대로는 못 보낸다. H.264 로 2Mbps 면
    // 유선은 물론 Wi-Fi AP 로도 나간다.
    struct Video {
        bool enabled = true;
        int port = 5000;          // IPC 에서 tcp://<보드>:5000 으로 받는다
        int fps = 30;             // 인코딩 목표 (카메라가 60이어도 30으로 줄여 보낸다)
        int bps = 2'000'000;
    } video;

    // ── 🔴 과전류 보호 ────────────────────────────────────────────
    //
    // 2026-08-07 모터 4대를 태우고 넣은 장치다.
    // 전자 브레이크(무여자 작동형)를 해제하지 않은 채 구동해서, 무부하인데도
    // 정격 2.7A 의 6~9배가 흘렀다. 그런데도 **아무도 멈추지 않았다** —
    // 모터의 내장 보호(15A 퓨즈)도 안 걸렸고, 소프트웨어에도 감시가 없었다.
    //
    //   지령  5 RPM →  0.2 A   정상
    //   지령 10 RPM → 17.3 A   여기서 멈췄어야 했다
    //   지령 20 RPM → 25.4 A   포화, 이후 컨트롤러 소손
    //
    // 카탈로그 정격전류 2.7A. 무부하 주행에서 이를 크게 넘으면 뭔가 잘못된 것이다
    // (브레이크 미해제, 기계적 구속, 결선 이상). 원인이 뭐든 **일단 세우는 게 맞다.**
    double overcurrent_a = 5.0;      // 이 값을 넘는 상태가 이어지면 e-stop
    int overcurrent_hits = 3;        // 연속 몇 번 넘으면 트립할지 (중앙값 기준)

    // 전류 피드백은 순간 피크 샘플이라 0xFE(25.4A) 포화가 수시로 튄다.
    // 단발 스파이크로 오작동하지 않도록 최근 N 개의 **중앙값**으로 판정한다.
    int current_window = 5;

    // 폴링 1회 타임아웃.
    //
    // 🔴 실측 왕복이 **21ms**다 (이론 1.6ms가 아니다 — nuri_latency.py 로 측정).
    //    Bus::send 의 최소 간격 12ms + 모터 응답 지연 + 처리 시간이 합쳐진 값이다.
    //    20ms로 뒀더니 1ms 차이로 매번 실패해 "휠 응답 없음"으로 e-stop이 걸렸다.
    //    여유를 두되, 길게 잡으면 죽은 축이 그만큼 루프를 잡아먹으니 40ms로 절충한다.
    //    (nurirobot의 기본 request는 400ms×재시도1 = 최악 800ms라 주기 폴링엔 못 쓴다)
    Ms poll_timeout{40};

    // 연속 실패가 이만큼이면 그 축을 죽은 것으로 보고 **전 축을 정지**시킨다.
    // 4륜이 모두 구동부라 한 축만 빠져도 거동을 신뢰할 수 없다.
    int motor_fail_limit = 5;
};

}  // namespace navi
