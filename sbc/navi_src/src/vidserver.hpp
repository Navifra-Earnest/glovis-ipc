// vidserver.hpp — IR 영상을 H.264 로 인코딩해 TCP 로 내보낸다.
//
//   navi::VideoServer vs(cfg.video, cam);   // cam 은 CamStream*
//   ...                                     // 전용 스레드가 알아서 돈다
//
// IPC 에서:
//   ffplay -fflags nobuffer -flags low_delay tcp://<보드>:5000
//
// 🔴 왜 navi 안에 있나
//    UVC 는 한 프로세스만 연다(실측: 두 번째는 EBUSY). 인코딩을 외부 프로세스로 빼면
//    navi 가 카메라를 놓아야 하고, 그러면 IR·열화상·ToF 를 계속 보낼 수 없다.
//    navi 가 카메라를 쥔 채 직접 인코딩해서 둘 다 만족시킨다.
//
// 설계
//   · **보는 사람이 없으면 인코딩도 안 한다.** 클라이언트가 붙을 때만 VPU 를 돌린다
//   · 새 클라이언트에게는 SPS/PPS 를 먼저 보낸다 — 없으면 화면이 안 그려진다
//   · 전송이 밀리면 그 프레임을 버린다. 제어 루프를 막느니 화면이 끊기는 게 낫다
//   · 소켓은 논블로킹 — write 가 막히면 navi 전체가 선다
#pragma once

#include <atomic>
#include <cerrno>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include "camstream.hpp"
#include "config.hpp"
#include "h264enc.hpp"

namespace navi {

struct VideoStatus {
    bool listening = false;
    int clients = 0;
    unsigned long long frames = 0;    // 인코딩한 프레임
    unsigned long long dropped = 0;   // 전송이 밀려 버린 것
    unsigned long long bytes = 0;
    double fps = 0.0;
    std::string error;
};

class VideoServer {
public:
    VideoServer(const Config::Video& cfg, CamStream* cam) : cfg_(cfg), cam_(cam) {
        if (!cfg_.enabled || !cam_) return;
        openListen();
        run_ = true;
        th_ = std::thread([this] { loop(); });
    }

    ~VideoServer() {
        run_ = false;
        if (th_.joinable()) th_.join();
        for (int fd : clients_) ::close(fd);
        if (listen_fd_ >= 0) ::close(listen_fd_);
    }
    VideoServer(const VideoServer&) = delete;
    VideoServer& operator=(const VideoServer&) = delete;

    VideoStatus status() const {
        std::lock_guard<std::mutex> lk(mu_);
        return st_;
    }

private:
    void openListen() {
        listen_fd_ = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
        if (listen_fd_ < 0) { st_.error = std::string("socket: ") + std::strerror(errno); return; }
        int on = 1;
        ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_ANY);
        a.sin_port = htons(static_cast<uint16_t>(cfg_.port));
        if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&a), sizeof a) < 0 ||
            ::listen(listen_fd_, 4) < 0) {
            st_.error = std::string("bind/listen ") + std::to_string(cfg_.port) + ": " + std::strerror(errno);
            ::close(listen_fd_);
            listen_fd_ = -1;
            return;
        }
        std::lock_guard<std::mutex> lk(mu_);
        st_.listening = true;
    }

    void acceptNew() {
        for (;;) {
            const int fd = ::accept(listen_fd_, nullptr, nullptr);
            if (fd < 0) return;                       // EAGAIN — 더 없다
            int fl = ::fcntl(fd, F_GETFL, 0);
            ::fcntl(fd, F_SETFL, fl | O_NONBLOCK);
            int on = 1;
            ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof on);   // 지연을 줄인다
            clients_.push_back(fd);
            pending_hdr_.push_back(true);             // SPS/PPS 를 먼저 보내야 한다
            std::lock_guard<std::mutex> lk(mu_);
            st_.clients = static_cast<int>(clients_.size());
        }
    }

    // 논블로킹 전송. 다 못 보내면 그 프레임은 버린다 — 밀린 영상을 뒤늦게 보내면
    // 지연만 쌓이고, 어차피 다음 프레임이 온다.
    bool sendAll(int fd, const uint8_t* p, size_t n) {
        size_t off = 0;
        int spins = 0;
        while (off < n) {
            const auto w = ::send(fd, p + off, n - off, MSG_NOSIGNAL);
            if (w > 0) { off += static_cast<size_t>(w); continue; }
            if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                if (++spins > 200) return false;      // 20ms 넘게 안 빠지면 포기
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }
            return false;                              // 끊겼다
        }
        return true;
    }

    void broadcast(const std::vector<uint8_t>& pkt, bool key) {
        for (size_t i = 0; i < clients_.size();) {
            bool ok = true;
            if (pending_hdr_[i]) {
                // 키프레임부터 시작해야 그림이 나온다. 그 전까지는 헤더만 들고 기다린다.
                if (!key) { ++i; continue; }
                ok = sendAll(clients_[i], enc_->header().data(), enc_->header().size());
                if (ok) pending_hdr_[i] = false;
            }
            if (ok) ok = sendAll(clients_[i], pkt.data(), pkt.size());
            if (!ok) {
                ::close(clients_[i]);
                clients_.erase(clients_.begin() + i);
                pending_hdr_.erase(pending_hdr_.begin() + i);
                std::lock_guard<std::mutex> lk(mu_);
                st_.clients = static_cast<int>(clients_.size());
                ++st_.dropped;
                continue;
            }
            ++i;
        }
    }

    void loop() {
        using Clock = std::chrono::steady_clock;
        std::vector<uint8_t> frame, pkt;
        FrameMeta meta;
        auto win = Clock::now();
        unsigned in_win = 0;

        while (run_) {
            if (listen_fd_ >= 0) acceptNew();

            // 아무도 안 보면 VPU 를 돌리지 않는다
            if (clients_.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            if (!cam_->latest(frame, meta)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            // 🔴 **프레임 감축.** 없으면 카메라 속도(실측 61fps)로 그대로 인코딩되는데,
            //    MPP 에는 `rc:fps_in_num = cfg_.fps`(30)로 알려주므로 CBR 이 프레임당
            //    예산을 30fps 기준으로 잡는다 → **출력이 정확히 2배**가 된다.
            //    실측 2026-08-18: 목표 2Mbps 인데 스트림이 3.9~4.0Mbps.
            //    (config.hpp 주석에 "카메라가 60이어도 30으로 줄여 보낸다" 고
            //     의도가 적혀 있었지만 구현이 없었다 — 2026-09-03 확인)
            //    덤으로 VPU·CPU 부하와 네트워크 패킷 수도 절반이 된다.
            if (cfg_.fps > 0) {
                const auto now = Clock::now();
                const auto period = std::chrono::microseconds(1'000'000 / cfg_.fps);
                // 첫 프레임은 그냥 보낸다. 이후는 주기를 지킨다.
                // 여유 10% 를 빼는 이유: 카메라 주기와 인코딩 주기가 딱 안 맞으면
                // 매번 "아직 이르다" 로 밀려 실효 fps 가 절반으로 떨어진다.
                if (next_enc_.time_since_epoch().count() != 0 && now < next_enc_) continue;
                next_enc_ = now + period - period / 10;
            }

            // 첫 프레임에서 실제 해상도를 보고 인코더를 만든다 — 설정과 카메라가
            // 다를 수 있으므로 카메라가 주는 값을 따른다.
            if (!enc_) {
                try {
                    enc_ = std::make_unique<H264Enc>(static_cast<int>(meta.width),
                                                     static_cast<int>(meta.height),
                                                     cfg_.fps, cfg_.bps);
                } catch (const std::exception& e) {
                    std::lock_guard<std::mutex> lk(mu_);
                    st_.error = e.what();
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    continue;
                }
            }

            bool key = false;
            if (!enc_->encode(frame.data(), frame.size(), pkt, &key)) continue;
            broadcast(pkt, key);

            std::lock_guard<std::mutex> lk(mu_);
            ++st_.frames;
            st_.bytes += pkt.size();
            if (++in_win; Clock::now() - win >= std::chrono::seconds(1)) {
                st_.fps = in_win / std::chrono::duration<double>(Clock::now() - win).count();
                in_win = 0;
                win = Clock::now();
            }
        }
    }

    Config::Video cfg_;
    CamStream* cam_ = nullptr;
    std::unique_ptr<H264Enc> enc_;
    // Clock 별칭은 루프 함수 안에 있으므로 멤버에는 정규 타입을 쓴다
    std::chrono::steady_clock::time_point next_enc_{};   // 0 = 아직 한 장도 안 보냄
    int listen_fd_ = -1;
    std::vector<int> clients_;
    std::vector<bool> pending_hdr_;
    std::thread th_;
    std::atomic<bool> run_{false};
    mutable std::mutex mu_;
    VideoStatus st_{};
};

}  // namespace navi
