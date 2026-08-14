// actuator_demo.cpp — 구동 확인용. 무한 루프 없음, 끝나면 반드시 정지
//
// 빌드: g++ -std=c++17 -O2 -o actuator_demo actuator_demo.cpp
// 실행: sudo ./actuator_demo [듀티%] [초]
#include "actuator.hpp"

#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv) {
    const double duty = argc > 1 ? std::atof(argv[1]) : 30.0;
    const int secs = argc > 2 ? std::atoi(argv[2]) : 3;

    try {
        const auto& board = navi::Actuator::detect();
        std::printf("보드: %s  HAT rev %d  (pwm=%s, dir=%d, fault=%d, adc=%d)\n",
                    board.model, navi::Actuator::hatRev(), board.pwmchip,
                    board.gpio_dir, board.gpio_fault, board.adc_channel);

        navi::Actuator act(board);

        if (act.fault()) {
            std::fprintf(stderr,
                "🔴 EN/nFAULT = LOW — U6 fault 또는 UVLO\n"
                "   VBAT(48V) / +3.3V(C25) 확인 필요\n");
            return 1;
        }
        std::printf("초기: fault=%d, I=%.2fA\n\n", act.fault(), act.current());

        for (bool ret : {false, true}) {
            std::printf("▶ %s, duty=%.0f%%, %ds\n", ret ? "RET(후진)" : "EXT(전진)",
                        duty, secs);
            const auto st = act.runFor(ret, duty, std::chrono::seconds(secs));
            std::printf("  %s  I=%.2fA\n\n", navi::toString(st), act.current());
            if (st == navi::State::Fault) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        std::printf("종료: fault=%d, I=%.2fA\n", act.fault(), act.current());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "오류: %s\n", e.what());
        return 1;
    }
    return 0;
}
