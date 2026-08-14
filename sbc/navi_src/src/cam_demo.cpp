// cam_demo.cpp — 카메라 정체 파악 + 캡처 확인. 사양을 몰라도 돌아간다.
//
// 빌드: g++ -std=c++17 -O2 -o cam_demo cam_demo.cpp
// 실행:
//   ./cam_demo                       # 붙어 있는 카메라 전부 나열 (지원 포맷까지)
//   ./cam_demo See3CAM               # 이름으로 찾아 기본 포맷으로 30프레임
//   ./cam_demo See3CAM 1920 1080 UYVY 60
//
// Y16으로 잡히면 열화상 raw다. 프레임마다 min/max/중앙 픽셀 카운트를 찍어주니,
// 손을 대보면 값이 오르는지로 온도 채널인지 바로 확인할 수 있다.
#include "uvccam.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace cm = navi::cam;

static void listAll() {
    bool found = false;
    for (const auto& e : std::filesystem::directory_iterator("/dev")) {
        const auto name = e.path().filename().string();
        if (name.rfind("video", 0) != 0) continue;
        if (name.find("dec") != std::string::npos || name.find("enc") != std::string::npos)
            continue;
        found = true;
        std::printf("── %s\n", e.path().c_str());
        try {
            for (const auto& f : cm::formats(e.path().string()))
                std::printf("     %s  %ux%u\n", cm::fourccStr(f.format).c_str(),
                            f.width, f.height);
        } catch (const std::exception& ex) {
            std::printf("     조회 실패: %s\n", ex.what());
        }
    }
    if (!found)
        std::printf("카메라가 없다. USB 연결과 `lsmod | grep uvcvideo` 확인.\n"
                    "자세히 보려면: sh cam_probe.sh\n");
}

// Y16(16bit raw) 통계 — 열화상이 실제 온도 채널을 주는지 확인용
static void y16Stats(const cm::Frame& f) {
    const auto* p = reinterpret_cast<const uint16_t*>(f.data);
    const size_t n = f.size / 2;
    if (!n) return;
    uint16_t lo = 0xFFFF, hi = 0;
    unsigned long long sum = 0;
    for (size_t i = 0; i < n; ++i) {
        lo = std::min(lo, p[i]);
        hi = std::max(hi, p[i]);
        sum += p[i];
    }
    const size_t mid = (f.height / 2) * f.width + f.width / 2;
    std::printf("   Y16  min %5u  max %5u  평균 %5llu  중앙픽셀 %5u\n",
                lo, hi, sum / n, mid < n ? p[mid] : 0);
}

int main(int argc, char** argv) {
    if (argc < 2) { listAll(); return 0; }

    const std::string want = argv[1];
    const uint32_t w = argc > 2 ? std::atoi(argv[2]) : 0;
    const uint32_t h = argc > 3 ? std::atoi(argv[3]) : 0;
    const uint32_t fps = argc > 5 ? std::atoi(argv[5]) : 0;
    uint32_t fmt = 0;
    if (argc > 4 && std::strlen(argv[4]) >= 4) {
        const char* s = argv[4];
        fmt = cm::fourcc(s[0], s[1], s[2], s[3]);
    }

    try {
        const auto path = cm::find(want);
        std::printf("찾음: %s\n", path.c_str());

        const auto avail = cm::formats(path);
        if (avail.empty()) throw std::runtime_error("지원 포맷이 없다");
        // 인자를 안 줬으면 장치가 첫 번째로 보고한 조합을 그대로 쓴다
        const auto pick = avail.front();
        cm::Camera cam(path, w ? w : pick.width, h ? h : pick.height,
                       fmt ? fmt : pick.format, fps);
        std::printf("%s  %ux%u  %s%s\n\n", cam.card().c_str(), cam.width(), cam.height(),
                    cm::fourccStr(cam.format()).c_str(), fps ? "" : " (fps 기본값)");

        cam.start();
        cm::Frame f;
        int got = 0, miss = 0;
        for (int i = 0; i < 30; ++i) {
            if (!cam.grab(f, cm::Ms(500))) { ++miss; continue; }
            ++got;
            if (got <= 3 || got % 10 == 0) {
                std::printf("  #%u  %zu bytes  %ux%u\n", f.sequence, f.size, f.width, f.height);
                if (f.format == cm::kY16) y16Stats(f);
            }
            if (got == 1) {   // 첫 장은 파일로 남긴다
                const auto out = "/tmp/cam_first." + cm::fourccStr(f.format);
                if (FILE* fp = std::fopen(out.c_str(), "wb")) {
                    std::fwrite(f.data, 1, f.size, fp);
                    std::fclose(fp);
                    std::printf("   → %s 저장\n", out.c_str());
                }
            }
        }
        cam.stop();
        std::printf("\n%d장 수신, %d회 타임아웃\n", got, miss);
        return got ? 0 : 1;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "🔴 %s\n", e.what());
        return 1;
    }
}
