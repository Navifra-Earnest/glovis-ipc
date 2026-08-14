// tm_view.cpp — 열화상을 브라우저로 본다. 온도 단위는 ℃로 맞춘다.
//
// 빌드: make tm_view
// 실행: ./tm_view [포트] [컬러맵번호]
//       브라우저에서  http://192.168.0.85:8081
//
// 열화상은 80x60이라 그대로 보면 우표만 하다. 브라우저에서 CSS로 확대하고(픽셀 보간 끔),
// 화면 중앙·최고·최저 온도를 같이 띄운다. 렌즈가 어디를 보는지 확인하는 게 목적이다.
//
// SDK가 컬러맵까지 입혀주므로(ToBitmap) 여기서는 JPEG로만 감싼다.
#include <TmSDK/libTmCore.hpp>

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

using namespace TmSDK;

namespace {

std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop = true; }

std::mutex g_mu;
std::vector<uint8_t> g_jpg;      // 최신 JPEG
uint64_t g_seq = 0;
double g_lo = 0, g_avg = 0, g_hi = 0, g_center = 0;
std::string g_unit = "°C";

// 최소 JPEG 인코더 대신 gstreamer를 쓰지 않고, BMP를 그대로 내보낸다.
// 80x60 RGB면 14KB라 그냥 보내도 부담이 없고, 인코딩 지연도 없다.
std::vector<uint8_t> toBmp(const uint8_t* bgr, int w, int h) {
    const int row = w * 3;
    const int pad = (4 - (row % 4)) % 4;
    const int dataSize = (row + pad) * h;
    const int fileSize = 54 + dataSize;
    std::vector<uint8_t> b(fileSize, 0);
    auto put32 = [&](int off, uint32_t v) { std::memcpy(&b[off], &v, 4); };
    auto put16 = [&](int off, uint16_t v) { std::memcpy(&b[off], &v, 2); };
    b[0] = 'B'; b[1] = 'M';
    put32(2, fileSize);
    put32(10, 54);
    put32(14, 40);
    put32(18, w);
    put32(22, h);            // 양수 = bottom-up
    put16(26, 1);
    put16(28, 24);
    put32(34, dataSize);
    for (int y = 0; y < h; ++y) {
        // BMP는 아래에서 위로 저장한다
        const uint8_t* src = bgr + static_cast<size_t>(h - 1 - y) * row;
        std::memcpy(&b[54 + static_cast<size_t>(y) * (row + pad)], src, row);
    }
    return b;
}

const char* kPage = R"HTML(<!doctype html><meta charset=utf-8>
<title>tm_view — 열화상</title>
<style>
 body{margin:0;background:#111;color:#ddd;font:13px system-ui,sans-serif;
      display:flex;flex-direction:column;height:100vh}
 header{padding:8px 12px;background:#1c1c1c;display:flex;gap:16px;align-items:center;flex-wrap:wrap}
 header b{color:#fff}
 .t{font-variant-numeric:tabular-nums}
 #hi{color:#ff6b6b} #lo{color:#6bb6ff} #ce{color:#ffd76b}
 #wrap{flex:1;position:relative;display:flex;align-items:center;justify-content:center}
 /* 80x60을 그대로 키운다. 보간을 끄면 픽셀이 또렷해 화각 확인이 쉽다 */
 img{height:80vh;image-rendering:pixelated;display:block}
 #cross{position:absolute;pointer-events:none}
 #cross::before,#cross::after{content:"";position:absolute;background:#fff8}
 #cross::before{left:50%;top:-14px;bottom:-14px;width:1px}
 #cross::after{top:50%;left:-14px;right:-14px;height:1px}
</style>
<header>
  <b>열화상 TMC80F</b> <span>80×60</span>
  <span class=t>중앙 <b id=ce>-</b></span>
  <span class=t>최고 <b id=hi>-</b></span>
  <span class=t>최저 <b id=lo>-</b></span>
  <span class=t id=fps>-</span>
</header>
<div id=wrap><img id=v><div id=cross></div></div>
<script>
 // BMP를 계속 갈아끼운다. 9fps라 이 정도로 충분하다.
 let n=0, t0=Date.now();
 async function tick(){
   try{
     const r = await fetch('/frame?'+Date.now());
     const meta = JSON.parse(r.headers.get('X-Temp'));
     ce.textContent = meta.center.toFixed(1)+meta.unit;
     hi.textContent = meta.hi.toFixed(1)+meta.unit;
     lo.textContent = meta.lo.toFixed(1)+meta.unit;
     const b = await r.blob();
     const u = URL.createObjectURL(b);
     v.onload = ()=>URL.revokeObjectURL(u);
     v.src = u;
     if(++n%10===0){ fps.textContent=(n/((Date.now()-t0)/1000)).toFixed(1)+' fps'; }
   }catch(e){ fps.textContent='(연결 끊김)'; }
   setTimeout(tick, 100);
 }
 tick();
</script>
)HTML";

void sendAll(int fd, const void* p, size_t n) {
    auto* b = static_cast<const char*>(p);
    while (n) {
        const auto k = ::send(fd, b, n, MSG_NOSIGNAL);
        if (k <= 0) return;
        b += k;
        n -= static_cast<size_t>(k);
    }
}

void serve(int fd) {
    char req[1024]{};
    const auto n = ::recv(fd, req, sizeof req - 1, 0);
    if (n <= 0) { ::close(fd); return; }
    const std::string r(req, static_cast<size_t>(n));

    if (r.rfind("GET /frame", 0) == 0) {
        std::vector<uint8_t> img;
        char meta[192];
        {
            std::lock_guard<std::mutex> lk(g_mu);
            img = g_jpg;
            std::snprintf(meta, sizeof meta,
                R"({"lo":%.2f,"avg":%.2f,"hi":%.2f,"center":%.2f,"unit":"%s"})",
                g_lo, g_avg, g_hi, g_center, g_unit.c_str());
        }
        char hdr[512];
        // CORS 허용 — cam_view(8080)의 /both 페이지가 여기(8081) 프레임을 fetch 한다.
        // 포트가 다르면 다른 출처라 이 헤더가 없으면 브라우저가 막는다.
        // X-Temp 는 커스텀 헤더라 Expose-Headers 로 열어줘야 JS가 읽을 수 있다.
        const int h = std::snprintf(hdr, sizeof hdr,
            "HTTP/1.1 200 OK\r\nContent-Type: image/bmp\r\nX-Temp: %s\r\n"
            "Access-Control-Allow-Origin: *\r\nAccess-Control-Expose-Headers: X-Temp\r\n"
            "Cache-Control: no-store\r\nContent-Length: %zu\r\nConnection: close\r\n\r\n",
            meta, img.size());
        sendAll(fd, hdr, static_cast<size_t>(h));
        if (!img.empty()) sendAll(fd, img.data(), img.size());
        ::close(fd);
        return;
    }

    char hdr[256];
    const int h = std::snprintf(hdr, sizeof hdr,
        "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
        "Content-Length: %zu\r\nConnection: close\r\n\r\n", std::strlen(kPage));
    sendAll(fd, hdr, static_cast<size_t>(h));
    sendAll(fd, kPage, std::strlen(kPage));
    ::close(fd);
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGPIPE, SIG_IGN);

    const int port = argc > 1 ? std::atoi(argv[1]) : 8081;
    const int cmap = argc > 2 ? std::atoi(argv[2]) : 2;   // 2 = Jet

    auto list = TmLocalCamera::GetCameraList();
    if (list.empty()) { std::fprintf(stderr, "열화상을 못 찾았다\n"); return 1; }

    TmLocalCamera cam;
    list[0].CamTimeout = 5000;
    if (!cam.Open(&list[0])) { std::fprintf(stderr, "Open 실패\n"); return 1; }
    for (int i = 0; i < 40 && !cam.IsConnected(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 기본값이 RAW(켈빈×100)라 그대로 두면 30084 같은 값이 나온다. ℃로 맞춘다.
    cam.SetTempUnit(TempUnit::CELSIUS);
    cam.SetColorMap(static_cast<ColormapTypes>(cmap));
    g_unit = cam.GetTempUnitSymbol();
    if (g_unit.empty()) g_unit = "°C";

    if (auto* ctl = cam.GetTmControl()) {
        std::printf("펌웨어 %s\n", ctl->GetFirmwareVersion().c_str());
        std::printf("FFC: %s\n", ctl->RunFlatFieldCorrection() ? "OK" : "실패");
        std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    }
    std::printf("단위: %s  컬러맵: %d\n", g_unit.c_str(), cmap);

    std::thread grabber([&cam] {
        TmFrame f;
        while (!g_stop) {
            if (!cam.QueryFrame(&f, 80, 60)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(40));
                continue;
            }
            double lo = 0, avg = 0, hi = 0;
            Point loP, hiP;
            f.MinMaxLoc(lo, avg, hi, loP, hiP);
            uint8_t* bgr = f.ToBitmap(ColorOrder::COLOR_BGR);
            if (!bgr) continue;
            auto bmp = toBmp(bgr, f.width, f.height);
            std::lock_guard<std::mutex> lk(g_mu);
            g_jpg.swap(bmp);
            // MinMaxLoc / GetPixel 은 raw(켈빈×100)를 준다. SetTempUnit 은 표시 단위만 정하고
            // 실제 변환은 GetTemperature() 로 해야 한다.
            g_lo = cam.GetTemperature(lo);
            g_avg = cam.GetTemperature(avg);
            g_hi = cam.GetTemperature(hi);
            g_center = cam.GetTemperature(f.GetPixel(40, 30));
            ++g_seq;
            // 🔴 프레임 버퍼를 돌려주지 않으면 계속 쌓인다 (417MB까지 불어나 OOM 발생)
            f.Release();
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
        g_stop = true; grabber.join();
        std::fprintf(stderr, "포트 %d 열기 실패\n", port);
        return 1;
    }
    std::printf("\n  http://192.168.0.85:%d  ← 브라우저에서 열기\n  Ctrl+C 종료\n\n", port);

    while (!g_stop) {
        timeval tv{1, 0};
        ::setsockopt(srv, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        const int c = ::accept(srv, nullptr, nullptr);
        if (c < 0) continue;
        std::thread(serve, c).detach();
    }
    ::close(srv);
    g_stop = true;
    grabber.join();
    cam.Close();
    std::printf("\n종료\n");
    return 0;
}
