// cam_view.cpp — 카메라 화면을 브라우저로 본다. 설치 각도·초점·구도 잡을 때 쓴다.
//
// 빌드: make cam_view
// 실행: ./cam_view                    # See3CAM, 1280x720, 포트 8080
//       ./cam_view See3CAM 1920 1080 8080
//       브라우저에서  http://192.168.0.85:8080
//
// 카메라가 MJPEG를 직접 내주므로 인코딩 없이 그대로 흘려보낸다 (CPU 거의 안 쓴다).
// 구도 잡기용 십자선·삼분할 격자는 브라우저에서 CSS로 겹친다 — 영상에는 손대지 않는다.
#include "uvccam.hpp"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <csignal>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

namespace cm = navi::cam;

namespace {

std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop = true; }

// 최신 JPEG 한 장만 들고 있는다. 늦게 붙은 브라우저에 옛 프레임을 보낼 이유가 없다.
class Latest {
public:
    void put(const uint8_t* p, size_t n) {
        std::lock_guard<std::mutex> lk(mu_);
        buf_.assign(p, p + n);
        ++seq_;
    }
    // seq가 바뀔 때까지 기다렸다가 복사해 간다 (같은 프레임을 두 번 보내지 않는다)
    bool get(std::vector<uint8_t>& out, uint64_t& seen, int wait_ms = 2000) {
        const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
        while (std::chrono::steady_clock::now() < end && !g_stop) {
            {
                std::lock_guard<std::mutex> lk(mu_);
                if (seq_ != seen && !buf_.empty()) {
                    out = buf_;
                    seen = seq_;
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
        return false;
    }
    uint64_t seq() const {
        std::lock_guard<std::mutex> lk(mu_);
        return seq_;
    }

private:
    mutable std::mutex mu_;
    std::vector<uint8_t> buf_;
    uint64_t seq_ = 0;
};

Latest g_latest;
std::atomic<double> g_fps{0.0};
std::string g_info = "?";

const char* kPage = R"HTML(<!doctype html><meta charset=utf-8>
<title>cam_view</title>
<style>
 body{margin:0;background:#111;color:#ddd;font:13px system-ui,sans-serif;
      display:flex;flex-direction:column;height:100vh}
 header{padding:6px 10px;background:#1c1c1c;display:flex;gap:14px;align-items:center;flex-wrap:wrap}
 header b{color:#fff}
 label{display:flex;gap:4px;align-items:center;cursor:pointer;user-select:none}
 #wrap{flex:1;position:relative;display:flex;align-items:center;justify-content:center;overflow:hidden}
 img{max-width:100%;max-height:100%;display:block}
 #ov{position:absolute;inset:0;pointer-events:none}
 /* 구도용 오버레이 — 영상 위에 겹치기만 한다 */
 .grid,.cross{position:absolute;inset:0;display:none}
 .on{display:block}
 .grid{background:
   linear-gradient(to right,transparent 33.33%,#0f08 33.33%,#0f08 calc(33.33% + 1px),transparent calc(33.33% + 1px)),
   linear-gradient(to right,transparent 66.66%,#0f08 66.66%,#0f08 calc(66.66% + 1px),transparent calc(66.66% + 1px)),
   linear-gradient(to bottom,transparent 33.33%,#0f08 33.33%,#0f08 calc(33.33% + 1px),transparent calc(33.33% + 1px)),
   linear-gradient(to bottom,transparent 66.66%,#0f08 66.66%,#0f08 calc(66.66% + 1px),transparent calc(66.66% + 1px))}
 .cross::before,.cross::after{content:"";position:absolute;background:#f00b}
 .cross::before{left:50%;top:0;bottom:0;width:1px}
 .cross::after{top:50%;left:0;right:0;height:1px}
</style>
<header>
  <b>cam_view</b><span id=info>-</span><span id=fps>-</span>
  <label><input type=checkbox id=g checked> 삼분할 격자</label>
  <label><input type=checkbox id=c checked> 중앙 십자선</label>
</header>
<div id=wrap>
  <img id=v src="/stream">
  <div id=ov><div class="grid on" id=gd></div><div class="cross on" id=cd></div></div>
</div>
<script>
 g.onchange=()=>gd.classList.toggle('on',g.checked);
 c.onchange=()=>cd.classList.toggle('on',c.checked);
 // 상태는 가볍게 폴링한다. 영상 스트림과 별개라 끊겨도 화면은 계속 나온다.
 setInterval(async()=>{try{
   const r=await(await fetch('/stat')).json();
   info.textContent=r.info; fps.textContent=r.fps.toFixed(1)+' fps';
 }catch(e){fps.textContent='(연결 끊김)'}},1000);
</script>
)HTML";

// 전부 보내면 true. 끊겼거나 타임아웃이면 false — 호출자가 그 연결을 포기한다.
bool sendAll(int fd, const void* p, size_t n) {
    auto* b = static_cast<const char*>(p);
    while (n) {
        const auto k = ::send(fd, b, n, MSG_NOSIGNAL);
        if (k <= 0) return false;
        b += k;
        n -= static_cast<size_t>(k);
    }
    return true;
}

void serve(int fd) {
    char req[1024]{};
    const auto n = ::recv(fd, req, sizeof req - 1, 0);
    if (n <= 0) { ::close(fd); return; }
    const std::string r(req, static_cast<size_t>(n));
    const bool stream = r.rfind("GET /stream", 0) == 0;
    const bool stat = r.rfind("GET /stat", 0) == 0;

    if (stat) {
        char body[256];
        const int m = std::snprintf(body, sizeof body, R"({"fps":%.1f,"info":"%s"})",
                                    g_fps.load(), g_info.c_str());
        char hdr[256];
        const int h = std::snprintf(hdr, sizeof hdr,
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
            "Content-Length: %d\r\nConnection: close\r\n\r\n", m);
        sendAll(fd, hdr, static_cast<size_t>(h));
        sendAll(fd, body, static_cast<size_t>(m));
        ::close(fd);
        return;
    }

    // /both — IR과 열화상을 한 화면에. 열화상은 tm_view(8081)가 따로 서빙한다.
    //
    // 한 프로세스에서 둘을 다 열어봤더니 TmSDK(OpenCV 백엔드)와 uvccam(v4l2 직접)이
    // 충돌해 열화상이 계속 실패했다. 프로세스는 나눠 두고 브라우저에서만 합치는 게
    // 확실하다 — <img> 는 교차 출처라도 표시에 제약이 없다.
    if (r.rfind("GET /both", 0) == 0) {
        char page[4096];
        const int len = std::snprintf(page, sizeof page, R"HTML(<!doctype html><meta charset=utf-8>
<title>navi — IR + 열화상</title>
<style>
 body{margin:0;background:#101114;color:#dcdde1;font:13px system-ui,sans-serif;
      display:flex;flex-direction:column;height:100vh}
 header{padding:8px 14px;background:#191a1f;border-bottom:1px solid #2a2b31;
        display:flex;gap:18px;align-items:center;flex-wrap:wrap}
 header b{color:#fff}
 #hi{color:#ff7b72}#lo{color:#79c0ff}#ce{color:#ffd866}
 main{flex:1;display:grid;grid-template-columns:2fr 1fr;gap:10px;padding:10px;min-height:0}
 @media(max-width:900px){main{grid-template-columns:1fr;grid-template-rows:1fr 1fr}}
 .p{background:#000;border:1px solid #2a2b31;border-radius:6px;position:relative;
    display:flex;align-items:center;justify-content:center;overflow:hidden;min-height:0}
 .p img{max-width:100%%;max-height:100%%;display:block}
 #t img{image-rendering:pixelated}
 .c{position:absolute;top:6px;left:8px;background:#000a;padding:2px 8px;border-radius:4px}
 .x{position:absolute;inset:0;pointer-events:none}
 .x::before,.x::after{content:"";position:absolute;background:#f008}
 .x::before{left:50%%;top:0;bottom:0;width:1px}
 .x::after{top:50%%;left:0;right:0;height:1px}
</style>
<header><b>navi</b>
 <span>중앙 <b id=ce>-</b> · 최고 <b id=hi>-</b> · 최저 <b id=lo>-</b></span>
 <span id=st>-</span></header>
<main>
 <div class=p><img src="/stream"><div class=c>IR 1280×720</div><div class=x></div></div>
 <div class=p id=t><img id=th><div class=c>열화상 80×60</div><div class=x></div></div>
</main>
<script>
 const TH='http://'+location.hostname+':8081';
 async function tick(){
   try{
     const r=await fetch(TH+'/frame?'+Date.now());
     const m=JSON.parse(r.headers.get('X-Temp'));
     ce.textContent=m.center.toFixed(1)+'℃';hi.textContent=m.hi.toFixed(1)+'℃';
     lo.textContent=m.lo.toFixed(1)+'℃';
     const b=await r.blob(),u=URL.createObjectURL(b);
     th.onload=()=>URL.revokeObjectURL(u);th.src=u;
   }catch(e){st.textContent='열화상 연결 안 됨 — tm_view 8081 이 떠 있는지 확인';}
   setTimeout(tick,120);
 }
 async function stat(){
   try{const s=await(await fetch('/stat')).json();st.textContent='IR '+s.fps.toFixed(0)+'fps';}catch(e){}
   setTimeout(stat,1000);
 }
 tick();stat();
</script>
)HTML");
        char hdr[256];
        const int h = std::snprintf(hdr, sizeof hdr,
            "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
            "Content-Length: %d\r\nConnection: close\r\n\r\n", len);
        sendAll(fd, hdr, static_cast<size_t>(h));
        sendAll(fd, page, static_cast<size_t>(len));
        ::close(fd);
        return;
    }

    if (!stream) {
        char hdr[256];
        const int h = std::snprintf(hdr, sizeof hdr,
            "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
            "Content-Length: %zu\r\nConnection: close\r\n\r\n", std::strlen(kPage));
        sendAll(fd, hdr, static_cast<size_t>(h));
        sendAll(fd, kPage, std::strlen(kPage));
        ::close(fd);
        return;
    }

    // MJPEG 스트림: multipart/x-mixed-replace — <img> 태그가 그대로 받아 계속 갱신한다
    int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    // 720p MJPEG는 프레임당 400KB나 된다. 브라우저가 못 따라가면 TCP 버퍼가 차고
    // send()가 영원히 막혀 화면이 멈춘 것처럼 보인다(실측). 타임아웃을 걸어
    // 느린 클라이언트는 끊고, 살아있는 쪽 스트림은 계속 돌게 한다.
    timeval sto{2, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &sto, sizeof sto);
    static const char kHdr[] =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-store\r\nConnection: close\r\n\r\n";
    sendAll(fd, kHdr, sizeof kHdr - 1);

    std::vector<uint8_t> jpg;
    uint64_t seen = 0;
    while (!g_stop) {
        if (!g_latest.get(jpg, seen)) break;      // 2초간 새 프레임이 없으면 끊는다
        char part[128];
        const int h = std::snprintf(part, sizeof part,
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n", jpg.size());
        // 하나라도 실패하면(느린 클라이언트, 탭 닫힘) 이 연결은 버린다.
        // 붙잡고 있으면 다른 시청자까지 같이 멈춘다.
        if (!sendAll(fd, part, static_cast<size_t>(h)) ||
            !sendAll(fd, jpg.data(), jpg.size()) ||
            !sendAll(fd, "\r\n", 2)) break;
    }
    ::close(fd);
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGPIPE, SIG_IGN);          // 브라우저가 탭을 닫으면 send가 죽는다

    const std::string want = argc > 1 ? argv[1] : "See3CAM";
    const uint32_t w = argc > 2 ? std::atoi(argv[2]) : 1280;
    const uint32_t h = argc > 3 ? std::atoi(argv[3]) : 720;
    const int port = argc > 4 ? std::atoi(argv[4]) : 8080;
    // 이 카메라의 MJPEG는 압축 품질이 고정이라 해상도를 낮춰도 프레임이 ~400KB로 일정하다
    // (640x480도 720p와 같은 크기, UVC에 quality/compression 컨트롤이 없음).
    // 15fps면 6MB/s라 브라우저가 못 따라가 화면이 멈춘다 → 구도 확인용은 5fps로 충분하다.
    const double view_fps = argc > 5 ? std::atof(argv[5]) : 5.0;

    try {
        const auto path = cm::find(want);
        // MJPEG로 연다 — 브라우저가 바로 먹는 포맷이라 인코딩이 필요 없다.
        cm::Camera cam(path, w, h, cm::kMJPEG, 30);
        g_info = cam.card() + "  " + std::to_string(cam.width()) + "x"
               + std::to_string(cam.height()) + "  MJPG";
        std::printf("%s  (%s)\n", g_info.c_str(), path.c_str());

        std::thread grabber([&cam, view_fps] {
            cm::Frame f;
            auto win = std::chrono::steady_clock::now();
            auto next = win;
            const auto gap = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / view_fps));
            unsigned cnt = 0;
            while (!g_stop) {
                // 카메라에서는 계속 받아 커널 버퍼를 비우되(안 그러면 지연이 쌓인다),
                // 브라우저로 넘기는 건 view_fps로 솎는다.
                if (!cam.grab(f, cm::Ms(500))) continue;
                const auto now = std::chrono::steady_clock::now();
                if (now < next) continue;
                next = now + gap;
                g_latest.put(f.data, f.size);
                if (++cnt; std::chrono::steady_clock::now() - win >= std::chrono::seconds(1)) {
                    g_fps = cnt / std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - win).count();
                    cnt = 0;
                    win = std::chrono::steady_clock::now();
                }
            }
        });

        const int srv = ::socket(AF_INET, SOCK_STREAM, 0);
        int one = 1;
        ::setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = INADDR_ANY;
        a.sin_port = htons(static_cast<uint16_t>(port));
        if (::bind(srv, reinterpret_cast<sockaddr*>(&a), sizeof a) < 0 || ::listen(srv, 8) < 0) {
            g_stop = true;
            grabber.join();
            std::fprintf(stderr, "포트 %d 열기 실패\n", port);
            return 1;
        }

        char host[64] = "192.168.0.85";
        std::printf("\n  http://%s:%d  ← 브라우저에서 열기\n  Ctrl+C 종료\n\n", host, port);

        while (!g_stop) {
            // accept가 시그널에 깨어나도록 타임아웃을 둔다
            timeval tv{1, 0};
            ::setsockopt(srv, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
            const int c = ::accept(srv, nullptr, nullptr);
            if (c < 0) continue;
            std::thread(serve, c).detach();     // 브라우저 탭마다 하나. 끊기면 알아서 정리된다
        }
        ::close(srv);
        g_stop = true;
        grabber.join();
        std::printf("\n종료\n");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "🔴 %s\n", e.what());
        return 1;
    }
}
