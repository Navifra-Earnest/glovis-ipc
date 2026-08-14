// tm_ffc_trace.cpp — FFC 명령의 CDC 바이트열을 알아내기 위한 도구.
//
// v4l2 로 전환하면서 FFC(셔터 보정)를 잃었다. SDK 를 켜면 UVC 를 점유해 v4l2 가
// 막히므로, **CDC(ttyACM0) 프로토콜만 알아내서 직접 치는 것**이 목표다.
//
//   sudo strace -f -y -e trace=write,read -s 200 -o /tmp/t.log ./tm_ffc_trace
//   grep -n "MARK\|ttyACM" /tmp/t.log
//
// stdout 에 마커를 찍어 두면 strace 로그에서 FFC 앞뒤 구간을 정확히 가를 수 있다.
// (tm_probe 는 펌웨어·상태 조회가 섞여 있어 어느 write 가 FFC 인지 알 수 없다)
#include <chrono>
#include <cstdio>
#include <thread>

#include <TmSDK/libTmCore.hpp>

static void mark(const char* s) {
    std::printf(">>>MARK %s\n", s);
    std::fflush(stdout);
}

int main() {
    auto list = TmSDK::TmLocalCamera::GetCameraList();
    if (list.empty()) { std::printf("장치 없음\n"); return 1; }

    TmSDK::TmLocalCamera cam;
    list[0].CamTimeout = 5000;
    if (!cam.Open(&list[0])) { std::printf("Open 실패\n"); return 2; }
    for (int i = 0; i < 40 && !cam.IsConnected(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto* ctl = cam.GetTmControl();
    if (!ctl) { std::printf("GetTmControl 실패\n"); cam.Close(); return 3; }

    // 조회 한 번 — 요청/응답 쌍의 기준을 잡는다
    mark("BEFORE_FW");
    const auto fw = ctl->GetFirmwareVersion();
    mark("AFTER_FW");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    mark("BEFORE_FFC");
    const bool ok = ctl->RunFlatFieldCorrection();
    mark("AFTER_FFC");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // FFC 모드 조회 — 같은 계열 명령이라 형식 비교에 쓴다
    mark("BEFORE_MODE");
    const int mode = ctl->GetFlatFieldCorrectionMode();
    mark("AFTER_MODE");

    std::printf("fw=%s ffc=%d mode=%d\n", fw.c_str(), ok, mode);
    cam.Close();
    return 0;
}
