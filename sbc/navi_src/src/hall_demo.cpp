// hall_demo.cpp — 비동기 API 확인. ROS 타이머 콜백과 같은 패턴
//
// 빌드: g++ -std=c++17 -O2 -o hall_demo hall_demo.cpp -lgpiod -lpthread
// 실행: sudo ./hall_demo [듀티%] [초]
#include "actuator.hpp"
#include "hall.hpp"

#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv) {
    const double duty = argc > 1 ? std::atof(argv[1]) : 40.0;
    const int secs = argc > 2 ? std::atoi(argv[2]) : 4;

    try {
        const auto& b = navi::Actuator::detect();
        std::printf("보드: %s  HAT rev %d\n", b.model, navi::Actuator::hatRev());

        navi::Actuator act(b);
        navi::HallCounter hall(b.hall_chip, b.hall_a, b.hall_chip_b, b.hall_b);
        act.attachHall(&hall);

        if (act.fault()) { std::fprintf(stderr, "🔴 EN/nFAULT LOW\n"); return 1; }

        for (bool ret : {false, true}) {
            std::printf("\n▶ %s duty=%.0f%%\n", ret ? "RET(후진)" : "EXT(전진)", duty);
            act.start(ret, duty);                       // 논블로킹

            // ── ROS 타이머 콜백 자리 (여기선 50Hz 루프로 흉내) ──
            navi::State st;
            while ((st = act.poll()) == navi::State::Running) {
                if (act.elapsed() >= std::chrono::seconds(secs)) { act.stop(); break; }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            std::printf("  결과: %-12s 위치 %+ld  전류 %.2fA (%.1fs)\n",
                        navi::toString(act.state()), act.position(),
                        act.current(), act.elapsed().count() / 1000.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "오류: %s\n", e.what());
        return 1;
    }
    return 0;
}
