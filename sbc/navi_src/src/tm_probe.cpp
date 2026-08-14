// tm_probe.cpp — ThermoEye TMC80 열화상 최소 확인.
//
// SDK가 UVC(영상)와 CDC(제어)를 함께 다뤄 스트림을 켜준다. UVC만으로는 프레임이 안 나온다.
//
// 빌드: g++ -std=c++17 -O2 -o tm_probe tm_probe.cpp -I/usr/include -lTmCore -L/usr/lib/TmSDK
// 실행: ./tm_probe
#include <TmSDK/libTmCore.hpp>

#include <cstdio>
#include <thread>

using namespace TmSDK;

int main() {
    auto list = TmLocalCamera::GetCameraList();
    std::printf("카메라 %zu대\n", list.size());
    for (auto& c : list)
        std::printf("  [%d] %s  com=%s  media=%zu\n", c.Index, c.Name.c_str(),
                    c.ComPort.c_str(), c.MediaSourcesList.size());
    if (list.empty()) { std::printf("장치를 못 찾았다\n"); return 1; }

    TmLocalCamera cam;
    // 공식 예제(examples/Linux/Camera.cpp)가 Open 전에 설정한다
    list[0].CamTimeout = 5000;
    if (!cam.Open(&list[0])) { std::printf("Open 실패\n"); return 1; }
    std::printf("열림: %s\n", cam.GetDevName().c_str());

    // Open 직후 백그라운드 프레임 스레드가 뜬다 (헤더 주석: "the only reader of the
    // video device"). 그게 첫 프레임을 물어올 때까지 기다린다.
    for (int i = 0; i < 40 && !cam.IsConnected(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::printf("IsOpen=%d IsConnected=%d\n", cam.IsOpen(), cam.IsConnected());

    // 카메라에게 직접 상태를 물어본다 — 영상이 안 나오는 이유가 여기 찍힐 수 있다
    if (auto* ctl = cam.GetTmControl()) {
        std::printf("  펌웨어  : %s\n", ctl->GetFirmwareVersion().c_str());
        auto [scode, stext] = ctl->GetSystemStatus();
        auto [ecode, etext] = ctl->GetSystemError();
        std::printf("  상태    : 0x%04X %s\n", scode, stext.c_str());
        std::printf("  에러    : 0x%04X %s\n", ecode, etext.c_str());
        // 열화상은 셔터를 닫고 화면 균일도를 잡아야(FFC) 정상 영상이 나온다.
        // 초기 보정이 안 된 상태면 스트림이 안 열릴 수 있다.
        std::printf("  FFC 실행: %s\n", ctl->RunFlatFieldCorrection() ? "OK" : "실패");
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    } else {
        std::printf("  (TmControl 없음)\n");
    }
    std::printf("\n프레임 대기...\n\n");

    TmFrame frame;
    int got = 0;
    for (int i = 0; i < 80 && got < 5; ++i) {
        if (!cam.QueryFrame(&frame, 80, 60)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(120));
            continue;
        }
        ++got;
        double lo = 0, avg = 0, hi = 0;
        Point loP, hiP;
        frame.MinMaxLoc(lo, avg, hi, loP, hiP);
        std::printf("#%d  %dx%d   최저 %.2f  평균 %.2f  최고 %.2f   중앙픽셀 %.2f\n",
                    got, frame.width, frame.height, lo, avg, hi, frame.GetPixel(40, 30));
        frame.Release();   // 안 하면 프레임 버퍼가 계속 쌓여 OOM 난다
    }
    cam.Close();
    std::printf("\n%s\n", got ? "✅ 온도 데이터 수신" : "❌ 프레임 없음");
    return got ? 0 : 1;
}
