// tm_watch.cpp — 열화상이 진짜 온도를 읽는지 확인한다.
//
// 프레임이 실제로 갱신되는지(같은 값 반복이 아닌지), 손을 대면 값이 따라 움직이는지 본다.
// tm_probe 가 5프레임이 소수점까지 동일하게 나와서 만든 것.
//
// 빌드: make tm_watch
// 실행: ./tm_watch [초]
#include <TmSDK/libTmCore.hpp>

#include <cstdio>
#include <thread>

using namespace TmSDK;

int main(int argc, char** argv) {
    const double secs = argc > 1 ? std::atof(argv[1]) : 15.0;

    auto list = TmLocalCamera::GetCameraList();
    if (list.empty()) { std::printf("장치 없음\n"); return 1; }
    TmLocalCamera cam;
    list[0].CamTimeout = 5000;
    if (!cam.Open(&list[0])) { std::printf("Open 실패\n"); return 1; }
    for (int i = 0; i < 40 && !cam.IsConnected(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (auto* ctl = cam.GetTmControl()) {
        auto [sc, st] = ctl->GetSystemStatus();
        std::printf("펌웨어 %s   상태 0x%04X\n", ctl->GetFirmwareVersion().c_str(), sc);
        std::printf("FFC: %s\n\n", ctl->RunFlatFieldCorrection() ? "OK" : "실패");
        std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    }

    std::printf("카메라 앞에서 손을 움직여 보세요. 값이 따라 변하면 정상입니다.\n\n");
    TmFrame f;
    const auto t0 = std::chrono::steady_clock::now();
    int got = 0, same = 0;
    double prev_avg = -999, prev_hi = -999;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < secs) {
        if (!cam.QueryFrame(&f, 80, 60)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(60));
            continue;
        }
        double lo = 0, avg = 0, hi = 0;
        Point loP, hiP;
        f.MinMaxLoc(lo, avg, hi, loP, hiP);
        // 직전 프레임과 완전히 같으면 갱신이 안 되는 것 — 그걸 세어서 보여준다
        const bool identical = (avg == prev_avg && hi == prev_hi);
        if (identical) ++same;
        prev_avg = avg;
        prev_hi = hi;
        std::printf("%5.1fs  #%-3d  최저 %6.2f  평균 %6.2f  최고 %6.2f  중앙 %6.2f  %s\n",
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(),
                    ++got, lo, avg, hi, f.GetPixel(40, 30), identical ? "(직전과 동일)" : "");
        f.Release();   // 안 하면 프레임 버퍼가 계속 쌓여 OOM 난다
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }
    cam.Close();
    std::printf("\n%d 프레임 중 %d 개가 직전과 완전히 동일 → %s\n", got, same,
                same > got * 3 / 4 ? "❌ 프레임이 갱신되지 않는다"
                                   : "✅ 프레임이 살아서 갱신된다");
    return 0;
}
