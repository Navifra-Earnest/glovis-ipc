// nuri_demo.cpp — 다축 구동 확인용. 무한 루프 없음, 끝나면 반드시 정지.
//
// 빌드: g++ -std=c++17 -O2 -o nuri_demo nuri_demo.cpp -lgpiod
// 실행: ./nuri_demo [RPM] [초] [ID...]
//   ./nuri_demo               # ID 0, 10RPM, 3초 정/역
//   ./nuri_demo 15 2 0 1 2 3  # 4축 동시, 15RPM, 2초
//
// 4축이라도 RS485는 반이중이라 순차 트랜잭션이다. 명령은 응답이 없어 축당 ~0.6ms(9600),
// 피드백은 왕복이라 축당 ~15ms(9600) / ~1.4ms(115200). 그래서 폴링은 라운드로빈으로 돈다.
#include "nurirobot.hpp"

#include <cstdio>
#include <cstdlib>
#include <vector>

namespace nu = navi::nuri;

int main(int argc, char** argv) {
    const double rpm = argc > 1 ? std::atof(argv[1]) : 10.0;
    const double secs = argc > 2 ? std::atof(argv[2]) : 3.0;
    std::vector<uint8_t> ids;
    for (int i = 3; i < argc; ++i) ids.push_back(static_cast<uint8_t>(std::atoi(argv[i])));
    if (ids.empty()) ids.push_back(0);

    try {
        nu::Bus bus("/dev/ttyS2", 9600, "gpiochip3", 3);

        std::vector<nu::Motor> axes;
        axes.reserve(ids.size());
        for (uint8_t id : ids) {
            axes.emplace_back(bus, id, 16.0);     // 감속비 불일치면 여기서 던진다
            std::printf("ID %u  펌웨어 v%d  감속비 %.1f:1\n",
                        id, axes.back().readFirmware(), axes.back().ratio());
        }

        for (const bool cw : {false, true}) {
            std::printf("\n[%s] %.1f RPM, %.1f초\n", cw ? "CW(역)" : "CCW(정)", rpm, secs);
            for (auto& m : axes) m.setSpeed(cw ? -rpm : rpm, 1.0);

            const auto t0 = nu::Clock::now();
            while (std::chrono::duration<double>(nu::Clock::now() - t0).count() < secs) {
                std::this_thread::sleep_for(nu::Ms(150));
                for (auto& m : axes) {
                    // poll()은 3상태다. Settling(명령 직후 안정화 구간)은 실패가 아니라
                    // 값을 아직 믿을 수 없다는 뜻이라, 여기서는 Ok 일 때만 찍는다.
                    if (m.poll() != nu::Motor::Poll::Ok) continue;
                    const auto& f = m.last();
                    std::printf("  %.1fs  ID%u  %-3s %8.2f°  %6.1f RPM  %5.1f A\n",
                                std::chrono::duration<double>(f.stamp - t0).count(),
                                m.id(), f.cw ? "CW" : "CCW", f.degree, f.rpm, f.amp);
                }
            }
            for (auto& m : axes) m.stop();        // Open-loop 듀티 0 — 여자까지 끊는다
            std::this_thread::sleep_for(nu::Ms(500));
        }

        std::printf("\n완료 (전 축 정지)\n");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "\n🔴 %s\n", e.what());
        return 1;
    }
}
