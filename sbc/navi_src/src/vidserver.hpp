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
#include <sys/ioctl.h>
#include <linux/sockios.h>
#include <netinet/in.h>
#include <netinet/ip.h>
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

            // 🔴 영상을 **배경 트래픽(AC_BK)** 으로 표시한다.
            //
            //    Wi-Fi 는 IP DSCP 로 WMM 큐를 고른다(cfg80211_classify8021d).
            //    CS1(DSCP 8, TOS 0x20) → AC_BK. 표시하지 않으면 영상이 구동 명령과
            //    같은 AC_BE 큐를 다투고, 실측에서 명령 RTT 가 이렇게 벌어졌다
            //    (2026-09-03, 같은 자리):
            //        영상 ON  → 평균 75.8 ms · 최대 575 ms
            //        영상 OFF → 평균 10.4 ms · 최대 144 ms
            //    신호가 -64 dBm 로 떨어지면 ON 에서 평균 606 ms · 최대 1680 ms 까지 갔다
            //    → 로봇 워치독(800 ms)이 주행 중에 트립한다.
            //
            //    영상은 몇십 ms 늦어도 사람 눈엔 안 보이지만, 구동 명령이 늦으면
            //    로봇이 선다. **우선순위를 뒤집는 게 맞다.**
            const int tos = 0x20;   // CS1 = 배경
            // 실패해도 영상은 계속 보낸다 — 우선순위는 최적화지 필수 기능이 아니다
            (void)::setsockopt(fd, IPPROTO_IP, IP_TOS, &tos, sizeof tos);
            clients_.push_back(fd);
            pending_hdr_.push_back(true);             // SPS/PPS 를 먼저 보내야 한다
            std::lock_guard<std::mutex> lk(mu_);
            st_.clients = static_cast<int>(clients_.size());
        }
    }

    // 소켓에 아직 안 빠진 바이트. 이걸로 **보내기 전에** 밀렸는지 판단한다.
    static size_t outQueue(int fd) {
        int q = 0;
        return (::ioctl(fd, SIOCOUTQ, &q) == 0 && q > 0) ? static_cast<size_t>(q) : 0;
    }

    // 전송 결과. **끊김과 밀림을 구분한다** — 예전엔 둘 다 false 로 뭉개서
    // 잠깐 밀린 것만으로 클라이언트를 끊었다.
    enum class Send { Ok, Backlog, Dead };

    // 논블로킹 전송. 한 번 시작한 프레임은 끝까지 보낸다 — 중간에 포기하면
    // 스트림이 깨져서 어차피 재동기가 필요하다.
    Send sendAll(int fd, const uint8_t* p, size_t n) {
        size_t off = 0;
        int spins = 0;
        while (off < n) {
            const auto w = ::send(fd, p + off, n - off, MSG_NOSIGNAL);
            if (w > 0) { off += static_cast<size_t>(w); spins = 0; continue; }
            if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                // 🔴 예전엔 20ms(200 spin)에서 포기하고 **연결을 끊었다.** 무선에서
                //    재전송 폭풍으로 수십~수백 ms 막히는 건 흔한 일이라, 그때마다
                //    끊기고 → IPC 가 재접속 → 키프레임 대기로 검은 화면이 길어지는
                //    루프가 돌았다(실측 2026-09-03: 15분에 파이프라인 재시작 60회,
                //    그 사이 로봇은 정상적으로 20.3 fps 를 보내고 있었다).
                //    이제 넉넉히 기다리고, 그래도 안 되면 **연결은 살린 채 재동기**한다.
                if (++spins > 3000) return Send::Backlog;      // 300ms
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }
            return Send::Dead;                          // 진짜 끊겼다 (EPIPE 등)
        }
        return Send::Ok;
    }

    void broadcast(const std::vector<uint8_t>& pkt, bool key) {
        for (size_t i = 0; i < clients_.size();) {
            // 🔴 **보내기 전에** 밀렸는지 본다. 이미 소켓에 한 프레임 넘게 쌓여 있으면
            //    이 프레임은 시작조차 하지 않는다 — 중간에 포기하면 스트림이 깨진다.
            //    지연을 쌓는 대신 프레임을 버리는 게 저지연 영상의 정석이다.
            //    (키프레임도 예외가 아니다. 다음 키프레임에 다시 붙으면 된다)
            if (outQueue(clients_[i]) > kMaxOutQ) {
                std::lock_guard<std::mutex> lk(mu_);
                ++st_.dropped;
                ++i;
                continue;
            }
            Send r = Send::Ok;
            if (pending_hdr_[i]) {
                // 키프레임부터 시작해야 그림이 나온다. 그 전까지는 헤더만 들고 기다린다.
                if (!key) { ++i; continue; }
                r = sendAll(clients_[i], enc_->header().data(), enc_->header().size());
                if (r == Send::Ok) pending_hdr_[i] = false;
            }
            if (r == Send::Ok) r = sendAll(clients_[i], pkt.data(), pkt.size());
            if (r == Send::Backlog) {
                // 프레임을 중간까지 보내고 못 끝냈다 → 스트림이 깨졌다.
                // **연결은 살리고** 헤더+다음 키프레임으로 재동기한다. 끊으면 IPC 가
                // 재접속하며 더 오래 검은 화면이 된다.
                pending_hdr_[i] = true;
                std::lock_guard<std::mutex> lk(mu_);
                ++st_.dropped;
                ++i;
                continue;
            }
            if (r == Send::Dead) {
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
    // 소켓에 이만큼 넘게 쌓여 있으면 새 프레임을 시작하지 않는다.
    // 1.2Mbps/20fps 면 프레임 평균 ~7.5KB → 64KB 는 약 0.4초분이다.
    static constexpr size_t kMaxOutQ = 64 * 1024;

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
