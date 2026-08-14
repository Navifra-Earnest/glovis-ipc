// enc_test.cpp — MPP H.264 인코더 자체 검증. 카메라를 열지 않는다.
//
//   ./enc_test [프레임수]        → /tmp/enc_test.h264
//   ffprobe /tmp/enc_test.h264   로 확인
//
// 카메라 없이 도는 게 중요하다 — navi 가 카메라를 쥔 상태에서도 인코더만 시험할 수 있다.
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "h264enc.hpp"

int main(int argc, char** argv) {
    const int n = argc > 1 ? std::atoi(argv[1]) : 60;
    const int W = 1920, H = 1080, FPS = 30;

    try {
        navi::H264Enc enc(W, H, FPS, 2'000'000);
        std::printf("인코더 준비 — %dx%d, SPS/PPS %zu 바이트\n",
                    enc.width(), enc.height(), enc.header().size());
        if (enc.header().empty()) {
            std::printf("🔴 헤더가 비었다 — 클라이언트가 중간에 붙으면 그림이 안 나온다\n");
            return 1;
        }

        FILE* f = std::fopen("/tmp/enc_test.h264", "wb");
        if (!f) { std::perror("파일 열기"); return 1; }
        std::fwrite(enc.header().data(), 1, enc.header().size(), f);

        // 움직이는 패턴 — 전부 같은 그림이면 압축이 지나치게 잘 돼 검증이 안 된다
        std::vector<uint8_t> uyvy(static_cast<size_t>(W) * H * 2);
        std::vector<uint8_t> pkt;
        size_t total = 0;
        int keys = 0, fails = 0;

        for (int i = 0; i < n; ++i) {
            for (int y = 0; y < H; ++y) {
                auto* row = uyvy.data() + static_cast<size_t>(y) * W * 2;
                for (int x = 0; x < W; ++x) {
                    row[x * 2 + 0] = static_cast<uint8_t>(128 + ((x + i * 7) & 0x3F));   // U/V
                    row[x * 2 + 1] = static_cast<uint8_t>((x + y + i * 5) & 0xFF);       // Y
                }
            }
            bool key = false;
            if (enc.encode(uyvy.data(), uyvy.size(), pkt, &key)) {
                std::fwrite(pkt.data(), 1, pkt.size(), f);
                total += pkt.size();
                if (key) ++keys;
            } else {
                ++fails;
            }
        }
        std::fclose(f);

        std::printf("프레임 %d개 → %zu 바이트 (평균 %zu B/프레임), 키프레임 %d, 실패 %d\n",
                    n, total, n ? total / n : 0, keys, fails);
        std::printf("실효 비트레이트 %.2f Mbps @ %dfps\n",
                    total * 8.0 * FPS / n / 1e6, FPS);
        return fails ? 1 : 0;
    } catch (const std::exception& e) {
        std::printf("🔴 %s\n", e.what());
        return 1;
    }
}
