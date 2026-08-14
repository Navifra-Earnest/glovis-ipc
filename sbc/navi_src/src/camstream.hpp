// camstream.hpp — 카메라 1대 = 스레드 1개. 최신 프레임만 남기고 나머지는 버린다.
//
// 왜 스레드인가: FHD@60fps는 16ms 주기인데 제어 루프는 20ms다. 주기가 안 맞고,
// grab()의 타임아웃 대기가 제어 루프를 막으면 모터 정지 명령이 늦어진다.
//
// 비용: uvccam의 Frame은 mmap 포인터이고 다음 grab()에서 무효화되므로, 스레드 경계를
// 넘기려면 복사가 불가피하다. FHD UYVY는 장당 4MB — 60fps면 240MB/s다. 실제로 그만큼
// 필요하지 않으면 Config에서 fps를 낮추거나 keep_frames=false로 두는 게 낫다.
#pragma once

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "config.hpp"
#include "uvccam.hpp"

namespace navi {

struct FrameMeta {
    uint32_t width = 0, height = 0, format = 0, sequence = 0;
    std::chrono::steady_clock::time_point stamp{};
};

struct CamStatus {
    std::string key, card, path;
    bool open = false;
    uint32_t width = 0, height = 0, format = 0;
    double fps = 0.0;              // 최근 1초 실측
    unsigned long long frames = 0; // 누적 수신
    unsigned long long drops = 0;  // grab 타임아웃
    unsigned long long reopens = 0;// USB 재열거 등으로 다시 연 횟수
    std::string error;             // 열기 실패 사유 (있으면 open=false)
    std::chrono::steady_clock::time_point last{};
};

class CamStream {
public:
    // 열기에 실패해도 던지지 않는다 — required 판단은 호출자(Robot)가 한다.
    // keep_frames=false면 스레드가 프레임을 받아 버리기만 한다 (생존·fps 확인용, 복사 없음).
    CamStream(const Config::Cam& cfg, bool keep_frames = true)
        : cfg_(cfg), key_(cfg.key), keep_(keep_frames) {
        st_.key = cfg.key;
        openCamera();
        // 처음에 못 열려도 스레드는 띄운다 — 나중에 꽂아도 알아서 붙게.
        run_ = true;
        th_ = std::thread([this] { loop(); });
    }

    ~CamStream() { close(); }
    CamStream(const CamStream&) = delete;
    CamStream& operator=(const CamStream&) = delete;

    void close() {
        run_ = false;
        if (th_.joinable()) th_.join();
        cam_.reset();
    }

    // st_.open은 캡처 스레드가 재연결하며 바꾸므로 락을 걸고 읽는다
    bool ok() const {
        std::lock_guard<std::mutex> lk(mu_);
        return st_.open;
    }
    const std::string& key() const { return key_; }

    // 최신 프레임 사본. 새 프레임이 없으면 false (같은 프레임을 두 번 주지 않는다).
    // keep_frames=false로 만들었으면 항상 false.
    bool latest(std::vector<uint8_t>& out, FrameMeta& meta) {
        std::lock_guard<std::mutex> lk(mu_);
        if (!fresh_) return false;
        out = buf_;          // 복사 — 소비자가 마음대로 써도 안전하다
        meta = meta_;
        fresh_ = false;
        return true;
    }

    CamStatus status() const {
        std::lock_guard<std::mutex> lk(mu_);
        return st_;
    }

private:
    // 장치를 찾아 연다. 실패하면 false (사유는 st_.error).
    // 경로가 아니라 카드 이름으로 찾으므로, USB가 재열거돼 /dev/videoN 번호가 바뀌어도 붙는다.
    bool openCamera() {
        try {
            const auto path = cam::find(cfg_.match);
            auto avail = cam::formats(path);
            if (avail.empty()) throw std::runtime_error("지원 포맷 없음");
            // 사양을 모르는 장치는 Config에 0을 넣어두고 여기서 장치 보고값을 쓴다
            const auto pick = avail.front();
            auto c = std::make_unique<cam::Camera>(
                path,
                cfg_.width  ? cfg_.width  : pick.width,
                cfg_.height ? cfg_.height : pick.height,
                cfg_.fourcc ? cfg_.fourcc : pick.format,
                cfg_.fps);
            std::lock_guard<std::mutex> lk(mu_);
            cam_ = std::move(c);
            st_.card = cam_->card();
            st_.path = cam_->path();
            st_.width = cam_->width();
            st_.height = cam_->height();
            st_.format = cam_->format();
            st_.open = true;
            st_.error.clear();
            return true;
        } catch (const std::exception& e) {
            std::lock_guard<std::mutex> lk(mu_);
            cam_.reset();
            st_.open = false;
            st_.error = e.what();
            return false;
        }
    }

    void loop() {
        using Clock = std::chrono::steady_clock;
        auto win = Clock::now();
        unsigned in_win = 0;
        int fails = 0;
        cam::Frame f;
        while (run_) {
            // 아직 못 열었거나 끊긴 상태 — 1초 간격으로 다시 붙어본다.
            // USB 재열거가 나면 기존 핸들은 죽은 채로 남으므로 반드시 다시 열어야 한다.
            if (!cam_) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                if (run_ && openCamera()) {
                    fails = 0;
                    std::lock_guard<std::mutex> lk(mu_);
                    ++st_.reopens;
                }
                continue;
            }

            if (!cam_->grab(f, cam::Ms(300))) {
                {
                    std::lock_guard<std::mutex> lk(mu_);
                    ++st_.drops;
                }
                // 연속 실패가 이어지면 장치가 사라진 것으로 보고 닫는다.
                // 300ms 타임아웃 × 10 = 3초 — 일시적 지연과 실제 단절을 가르는 선.
                if (++fails >= kReopenAfter) {
                    std::lock_guard<std::mutex> lk(mu_);
                    cam_.reset();
                    st_.open = false;
                    st_.error = "연속 " + std::to_string(fails) + "회 실패 — 재연결 시도";
                }
                continue;
            }
            fails = 0;
            {
                std::lock_guard<std::mutex> lk(mu_);
                ++st_.frames;
                st_.last = Clock::now();
                if (keep_) {
                    buf_.assign(f.data, f.data + f.size);
                    meta_ = {f.width, f.height, f.format, f.sequence, st_.last};
                    fresh_ = true;
                }
            }
            // fps는 1초 창으로 실측한다. 설정값이 아니라 실제로 나오는 값을 봐야
            // 대역폭 부족(USB 허브 공유 등)을 알아챌 수 있다.
            if (++in_win; Clock::now() - win >= std::chrono::seconds(1)) {
                const double sec = std::chrono::duration<double>(Clock::now() - win).count();
                std::lock_guard<std::mutex> lk(mu_);
                st_.fps = in_win / sec;
                in_win = 0;
                win = Clock::now();
            }
        }
    }

    static constexpr int kReopenAfter = 10;   // grab 300ms × 10 ≈ 3초

    Config::Cam cfg_;      // 재오픈에 필요하므로 값으로 보관한다
    std::string key_;
    bool keep_;
    std::unique_ptr<cam::Camera> cam_;
    std::thread th_;
    std::atomic<bool> run_{false};

    mutable std::mutex mu_;
    std::vector<uint8_t> buf_;
    FrameMeta meta_{};
    bool fresh_ = false;
    CamStatus st_{};
};

}  // namespace navi
