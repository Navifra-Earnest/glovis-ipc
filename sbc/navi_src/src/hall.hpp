// hall.hpp — 쿼드러처 홀 카운터 (libgpiod v1)
//
// 두 채널을 동시에 감시하고 상태 전이(그레이 코드)로 방향까지 판별한다.
// 별도 스레드에서 이벤트를 받아 위치를 누적한다.
//
// 링크: -lgpiod
#pragma once

#include <atomic>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

#include <gpiod.h>
#include <poll.h>

#include "actuator.hpp"   // IHall

namespace navi {

class HallCounter : public IHall {
public:
    // 🔴 두 채널이 **서로 다른 gpiochip** 에 있을 수 있다.
    //    3A(HAT V1)는 핀35·31 이 둘 다 gpiochip3 이었지만,
    //    5A 는 핀35=gpiochip4:0 / 핀31=gpiochip1:9 로 칩이 갈린다.
    //    gpiod_line_bulk 는 한 칩의 라인만 담으므로 bulk 로는 안 된다 —
    //    라인을 따로 잡고 poll() 로 두 fd 를 동시에 기다린다. 같은 칩이어도 그대로 동작한다.
    HallCounter(const char* chipA, unsigned lineA, const char* chipB, unsigned lineB) {
        chipA_ = gpiod_chip_open_by_name(chipA);
        chipB_ = (std::strcmp(chipA, chipB) == 0) ? chipA_ : gpiod_chip_open_by_name(chipB);
        if (!chipA_ || !chipB_) fail("gpiod_chip_open 실패");

        lnA_ = gpiod_chip_get_line(chipA_, lineA);
        lnB_ = gpiod_chip_get_line(chipB_, lineB);
        if (!lnA_ || !lnB_) fail("gpiod_chip_get_line 실패");

        if (gpiod_line_request_both_edges_events(lnA_, "navi-hall") ||
            gpiod_line_request_both_edges_events(lnB_, "navi-hall"))
            fail("이벤트 요청 실패");

        state_ = (gpiod_line_get_value(lnA_) << 1) | gpiod_line_get_value(lnB_);
        run_ = true;
        th_ = std::thread([this] { loop(); });
    }

    // 같은 칩에 두 채널이 있는 보드용 (3A)
    HallCounter(const char* chip, unsigned lineA, unsigned lineB)
        : HallCounter(chip, lineA, chip, lineB) {}

    ~HallCounter() {
        run_ = false;
        if (th_.joinable()) th_.join();
        if (lnA_) gpiod_line_release(lnA_);
        if (lnB_) gpiod_line_release(lnB_);
        if (chipB_ && chipB_ != chipA_) gpiod_chip_close(chipB_);
        if (chipA_) gpiod_chip_close(chipA_);
    }

    HallCounter(const HallCounter&) = delete;
    HallCounter& operator=(const HallCounter&) = delete;

    long position() const override { return pos_.load(); }
    long edges() const override { return edges_.load(); }
    void reset(long p = 0) override { pos_ = p; edges_ = 0; last_ = std::chrono::steady_clock::now(); }

    // 마지막 엣지 이후 경과. 구동 중인데 이 값이 커지면 스톨
    std::chrono::milliseconds sinceLastEdge() const override {
        if (edges_.load() == 0)
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - last_.load());
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_.load());
    }

private:
    [[noreturn]] void fail(const char* msg) {
        if (lnA_) gpiod_line_release(lnA_);
        if (lnB_) gpiod_line_release(lnB_);
        if (chipB_ && chipB_ != chipA_) gpiod_chip_close(chipB_);
        if (chipA_) gpiod_chip_close(chipA_);
        throw std::runtime_error(msg);
    }

    void loop() {
        // 그레이 코드 전이표: 이전상태<<2 | 현재상태 → +1 / -1 / 0(무효)
        static constexpr int kLut[16] = {
            0, +1, -1,  0,
           -1,  0,  0, +1,
           +1,  0,  0, -1,
            0, -1, +1,  0};
        // 라인마다 이벤트 fd 가 하나씩 나온다. 칩이 달라도 poll 로 같이 기다릴 수 있다.
        pollfd pfd[2]{{gpiod_line_event_get_fd(lnA_), POLLIN, 0},
                      {gpiod_line_event_get_fd(lnB_), POLLIN, 0}};
        while (run_) {
            const int r = ::poll(pfd, 2, 100);   // 100ms — run_ 을 주기적으로 확인하려고
            if (r <= 0) continue;                // 타임아웃 또는 오류
            // 이벤트를 읽어 큐를 비운다. 값은 아래에서 현재 레벨로 다시 읽는다
            // (엣지를 놓쳐도 레벨 기준이면 위치가 어긋나지 않는다).
            gpiod_line_event e;
            if (pfd[0].revents & POLLIN) gpiod_line_event_read(lnA_, &e);
            if (pfd[1].revents & POLLIN) gpiod_line_event_read(lnB_, &e);

            const int a = gpiod_line_get_value(lnA_);
            const int b = gpiod_line_get_value(lnB_);
            if (a < 0 || b < 0) continue;
            const int cur = (a << 1) | b;
            if (cur == state_) continue;         // 채터링 — 상태가 안 바뀌면 셀 게 없다
            const int d = kLut[(state_ << 2) | cur];
            state_ = cur;
            if (d) pos_ += d;
            edges_ += 1;
            last_ = std::chrono::steady_clock::now();
        }
    }

    gpiod_chip* chipA_ = nullptr;
    gpiod_chip* chipB_ = nullptr;      // chipA_ 와 같을 수 있다 (같으면 닫지 않는다)
    gpiod_line* lnA_ = nullptr;
    gpiod_line* lnB_ = nullptr;
    std::thread th_;
    std::atomic<bool> run_{false};
    std::atomic<long> pos_{0};
    std::atomic<long> edges_{0};
    std::atomic<std::chrono::steady_clock::time_point> last_{
        std::chrono::steady_clock::now()};
    int state_ = 0;
};

}  // namespace navi
