// nuri_selftest.cpp — 프레임 생성/체크섬을 문서 예제와 바이트 단위로 대조. 하드웨어 불필요.
//
// 빌드: g++ -std=c++17 -O2 -o nuri_selftest nuri_selftest.cpp -lgpiod
// 실행: ./nuri_selftest
#include "nurirobot.hpp"

#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace navi::nuri;

static void expect(const std::vector<uint8_t>& got,
                   std::initializer_list<uint8_t> want, const char* what) {
    const std::vector<uint8_t> w(want);
    if (got != w) {
        std::printf("❌ %s\n   기대:", what);
        for (auto b : w) std::printf(" %02x", b);
        std::printf("\n   실제:");
        for (auto b : got) std::printf(" %02x", b);
        std::printf("\n");
        std::abort();
    }
    std::printf("✅ %s\n", what);
}

int main() {
    // ① p.23 SA 예제 — ID0, CCW 180.00°, 5.0RPM
    {
        std::vector<uint8_t> v{0x00};
        put16(v, 18000);   // 180.00° / 0.01
        put16(v, 50);      // 5.0 RPM / 0.1
        expect(frame(0, kMdPosSpeed, v.data(), v.size()),
               {0xff, 0xfe, 0x00, 0x07, 0x2f, 0x01, 0x00, 0x46, 0x50, 0x00, 0x32},
               "p.23 위치·속도제어 (0x01)");
    }

    // ② p.24 SA 예제 — ID0 외부 감속비 2:1 (0.1 단위 → 20)
    {
        std::vector<uint8_t> v;
        put16(v, 20);
        expect(frame(0, kMdSetRatio, v.data(), v.size()),
               {0xff, 0xfe, 0x00, 0x04, 0xde, 0x09, 0x00, 0x14},
               "p.24 외부 감속비 설정 (0x09)");
    }

    // ③ 본문 3.2 — 위치 피드백 요청
    expect(frame(0, kReqPos), {0xff, 0xfe, 0x00, 0x02, 0x5c, 0xa1},
           "본문 3.2 요청 프레임 (0xA1)");

    // ④ 본문 3.2 — 응답 체크섬 검증
    {
        const uint8_t rx[] = {0xff, 0xfe, 0x00, 0x08, 0x26, 0xd1,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        assert(checksum(rx[2], rx[3], rx[5], rx + 6, 6) == rx[4]);
        std::printf("✅ 본문 3.2 응답 체크섬 (0xD1)\n");
    }

    // ⑤ 펌웨어 버전 응답(0xFD) 체크섬 예외 — 모터가 버전 바이트를 합에서 뺀다
    //    실측 수신: ff fe 00 03 ff fd 01
    {
        const uint8_t rx[] = {0xff, 0xfe, 0x00, 0x03, 0xff, 0xfd, 0x01};
        const uint8_t id = rx[2], size = rx[3], mode = rx[5];
        assert(checksum(id, size, mode, rx + 6, 1) != rx[4]);   // 규격대로면 안 맞고
        assert(checksum(id, size, mode) == rx[4]);              // Values 빼면 맞는다
        std::printf("✅ 펌웨어 응답(0xFD) 체크섬 예외\n");
    }

    // ⑥ Ping 요청 — 실기에서 왕복 확인된 프레임
    expect(frame(0, kReqPing), {0xff, 0xfe, 0x00, 0x02, 0x5d, 0xa0}, "Ping 요청 (0xA0)");

    // ⑦ 정지 = Open-loop 듀티 0
    {
        const uint8_t zero[3] = {0x00, 0x00, 0x00};
        expect(frame(0, kMdOpenLoop, zero, sizeof zero),
               {0xff, 0xfe, 0x00, 0x05, 0xe9, 0x11, 0x00, 0x00, 0x00},
               "정지 (Open-loop 듀티 0)");
    }

    // ⑧⑨ setSpeed()가 실제로 내보내는 바이트 — Motor를 만들지 않고 프레임만 재현한다.
    //     (Python에서 이 바이트로 모터가 10.0RPM에 도달하는 것까지 확인됐다)
    {
        for (const bool cw : {false, true}) {
            std::vector<uint8_t> v{static_cast<uint8_t>(cw ? 0x01 : 0x00)};
            put16(v, 100);        // 10.0 RPM / 0.1
            v.push_back(10);      // 1.0 s / 0.1
            expect(frame(0, kMdAccSpeed, v.data(), v.size()),
                   cw ? std::initializer_list<uint8_t>{0xff, 0xfe, 0x00, 0x06, 0x87,
                                                       0x03, 0x01, 0x00, 0x64, 0x0a}
                      : std::initializer_list<uint8_t>{0xff, 0xfe, 0x00, 0x06, 0x88,
                                                       0x03, 0x00, 0x00, 0x64, 0x0a},
                   cw ? "setSpeed(-10.0, 1.0) → CW" : "setSpeed(10.0, 1.0) → CCW");
        }
    }

    // ⑩ 제어 On/Off — 실기에서 소음을 끊은 그 프레임
    {
        const uint8_t off[1] = {0x01};
        expect(frame(0, kMdSetOnOff, off, 1),
               {0xff, 0xfe, 0x00, 0x03, 0xf1, 0x0a, 0x01}, "제어 Off (0x0A)");
    }

    std::printf("\n전부 통과\n");
    return 0;
}
