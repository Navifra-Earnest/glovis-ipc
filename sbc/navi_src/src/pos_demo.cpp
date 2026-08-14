// pos_demo.cpp — 호밍 / 캘리브레이션 / 절대·상대 위치 / 조그
//
// 빌드: g++ -std=c++17 -O2 -o pos_demo pos_demo.cpp -lgpiod -lpthread
// 실행: sudo ./pos_demo [듀티%]
#include "actuator.hpp"
#include "hall.hpp"

#include <cstdio>
#include <cstdlib>

namespace {

// ROS 타이머 콜백을 흉내낸 대기 루프. 실제 노드에서는 poll()만 주기 호출하면 된다
navi::State spin(navi::Actuator& act, std::chrono::milliseconds limit) {
    navi::State st;
    while ((st = act.poll()) == navi::State::Running) {
        if (act.elapsed() >= limit) { act.stop(); break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return act.state();
}

}  // namespace

int main(int argc, char** argv) {
    const double duty = argc > 1 ? std::atof(argv[1]) : 45.0;

    try {
        const auto& b = navi::Actuator::detect();
        navi::Actuator act(b);
        navi::HallCounter hall(b.hall_chip, b.hall_a, b.hall_chip_b, b.hall_b);
        act.attachHall(&hall);
        std::printf("보드: %s  HAT rev %d  듀티 %.0f%%\n", b.model, navi::Actuator::hatRev(), duty);

        if (act.fault()) { std::fprintf(stderr, "🔴 EN/nFAULT LOW\n"); return 1; }

        // ── 캘리브레이션 (호밍 + 반대 끝 측정) ──
        std::printf("\n[1] 캘리브레이션 — 양 끝단 탐색\n");
        const long stroke = act.calibrate(duty);
        if (!stroke) { std::fprintf(stderr, "🔴 캘리브레이션 실패 (%s)\n",
                                    navi::toString(act.state())); return 1; }
        std::printf("    전체 스트로크 = %ld 카운트\n", stroke);

        // ── 절대 위치 이동 ──
        std::printf("\n[2] 절대 위치 이동\n");
        for (double r : {0.5, 0.25, 0.75}) {
            const long tgt = static_cast<long>(stroke * r);
            act.moveToRatio(r, duty);
            const auto st = spin(act, std::chrono::seconds(30));
            std::printf("    목표 %4ld (%.0f%%) → 도달 %4ld  오차 %+ld  [%s]\n",
                        tgt, r * 100, act.position(), act.position() - tgt,
                        navi::toString(st));
        }

        // ── 상대 위치 이동 ──
        std::printf("\n[3] 상대 이동\n");
        for (long d : {+40L, -80L, +40L}) {
            const long from = act.position();
            act.moveBy(d, duty);
            const auto st = spin(act, std::chrono::seconds(30));
            std::printf("    %+4ld 요청: %4ld → %4ld  실이동 %+ld  [%s]\n",
                        d, from, act.position(), act.position() - from,
                        navi::toString(st));
        }

        // ── 조그 (0.8초만) ──
        std::printf("\n[4] 조그 EXT 0.8s\n");
        const long from = act.position();
        act.jog(false, duty);
        spin(act, std::chrono::milliseconds(800));
        std::printf("    %ld → %ld (%+ld)\n", from, act.position(),
                    act.position() - from);

        // ── 호밍 ──
        std::printf("\n[5] 호밍 (RET 끝 = 0)\n");
        const auto st = act.home(duty);
        std::printf("    결과 %s, 위치 %ld\n", navi::toString(st), act.position());

    } catch (const std::exception& e) {
        std::fprintf(stderr, "오류: %s\n", e.what());
        return 1;
    }
    return 0;
}
