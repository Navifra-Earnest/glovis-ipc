// overcurrent_test.cpp — 과전류 판정 로직 검증 (하드웨어 불필요)
//
// drive.hpp 의 판정과 같은 규칙을 여기서 재현해, 아래를 확인한다:
//   · 스파이크 한 번으로는 트립하지 않는다 (전류 피드백은 0xFE 포화가 수시로 튄다)
//   · 지속적으로 높으면 트립한다
//   · 창이 다 차기 전에는 판정하지 않는다
//
// 빌드: g++ -std=c++17 -O2 -o overcurrent_test overcurrent_test.cpp
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <vector>

namespace {

struct Judge {
    double threshold;
    int need_hits;
    int window;

    std::vector<double> hist{};
    int hits = 0;
    double median = 0.0;

    // 한 샘플 투입. 트립해야 하면 true.
    bool feed(double amp) {
        hist.push_back(amp);
        if (static_cast<int>(hist.size()) > window) hist.erase(hist.begin());
        if (static_cast<int>(hist.size()) < window) return false;   // 창이 덜 참
        auto s = hist;
        std::sort(s.begin(), s.end());
        median = s[s.size() / 2];
        if (median > threshold) return ++hits >= need_hits;
        hits = 0;
        return false;
    }
};

int fails = 0;
void check(bool ok, const char* what) {
    std::printf("  %s %s\n", ok ? "✅" : "🔴", what);
    if (!ok) ++fails;
}

}  // namespace

int main() {
    std::printf("과전류 판정 검증 (임계 5.0A, 연속 3회, 창 5개)\n\n");

    // ① 정상 전류에서는 트립하지 않는다
    {
        Judge j{5.0, 3, 5};
        bool tripped = false;
        for (int i = 0; i < 50; ++i) tripped |= j.feed(0.3);
        check(!tripped, "정상 0.3A 를 50회 — 트립 안 함");
    }

    // ② 스파이크 단발은 무시한다 (이게 핵심 — 0xFE 포화가 수시로 튄다)
    {
        Judge j{5.0, 3, 5};
        bool tripped = false;
        for (int i = 0; i < 30; ++i) tripped |= j.feed(i % 7 == 0 ? 25.4 : 0.3);
        check(!tripped, "0.3A 사이에 25.4A 스파이크가 섞여도 — 트립 안 함");
    }

    // ③ 지속적으로 높으면 트립한다
    {
        Judge j{5.0, 3, 5};
        bool tripped = false;
        int n = 0;
        for (; n < 30 && !tripped; ++n) tripped = j.feed(17.3);
        check(tripped, "17.3A 가 계속되면 — 트립함");
        // 창(5) 이 차고 연속 3회 → 7번째 샘플에서 걸려야 한다
        check(n == 7, "트립 시점이 예상대로 (창 5 + 연속 3)");
    }

    // ④ 실제로 모터를 태운 그 수열
    {
        Judge j{5.0, 3, 5};
        bool tripped = false;
        // 5RPM 구간(정상) → 10RPM 구간(17A)
        for (int i = 0; i < 10; ++i) tripped |= j.feed(0.2);
        check(!tripped, "5RPM 구간 0.2A — 트립 안 함");
        int n = 0;
        for (; n < 20 && !tripped; ++n) tripped = j.feed(17.3);
        check(tripped, "10RPM 구간 17.3A — 트립함 (여기서 멈췄어야 했다)");
        std::printf("     → %d 샘플 만에 정지. 실제로는 아무도 안 멈춰서 4대를 잃었다\n", n);
    }

    // ⑤ 임계 근처에서 흔들리면 연속 카운트가 리셋된다
    {
        Judge j{5.0, 3, 5};
        bool tripped = false;
        for (int i = 0; i < 40; ++i) tripped |= j.feed(i % 2 ? 6.0 : 1.0);
        check(!tripped, "6A/1A 를 번갈아 — 중앙값이 임계 아래라 트립 안 함");
    }

    std::printf("\n%s\n", fails ? "🔴 실패 있음" : "전부 통과");
    return fails ? 1 : 0;
}
