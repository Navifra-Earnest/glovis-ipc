// thermal.hpp — 열화상(ThermoEye TMC80) 스트림. camstream.hpp 와 같은 패턴.
//
// 카메라 1대 = 스레드 1개. 최신 판독값과 이미지만 들고 있고 나머지는 버린다.
//
// 🔴 기본 경로는 v4l2 다. TmSDK 는 없어도 된다 —
//    SDK 가 깔린 빌드에서만 FFC·SDK 컬러맵이 추가로 열린다(use_sdk=1).
//    SDK 의존은 이 파일 안에만 둔다. 상위(robot·mqtt·main)는 열화상이 항상 있다고 본다.
//
//   navi::ThermalStream th(cfg.thermal);
//   auto r = th.latest();           // r.hi, r.center ... ℃
//   std::vector<uint8_t> bmp;
//   if (th.image(bmp)) { ... }      // 컬러맵 입힌 BMP (선택)
//
// 온도 단위 주의: SetTempUnit()은 표시 단위만 정한다. 실제 변환은 GetTemperature()를
// 거쳐야 하고, 그냥 GetPixel()을 쓰면 raw(켈빈×100)가 나온다.
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef NAVI_HAS_TMSDK
#include <TmSDK/libTmCore.hpp>
#endif

#include "config.hpp"
#include "uvccam.hpp"   // SDK 가 프레임을 못 줄 때 v4l2 로 직접 읽는 폴백

namespace navi {

struct ThermalReading {
    double lo = 0, avg = 0, hi = 0, center = 0;   // ℃
    int width = 0, height = 0;
    std::chrono::steady_clock::time_point stamp{};
    bool valid = false;
};

struct ThermalStatus {
    bool open = false;
    // 센서가 스스로 고장을 보고한 상태. 재연결로 안 풀린다 — 사람이 봐야 한다.
    // (통신·펌웨어는 살아 있는데 이미징만 죽는 경우가 있다)
    bool fault = false;
    // SDK 가 프레임을 못 줘서 v4l2 로 직접 읽는 중. 값은 정상이지만 컬러맵이 흑백이다.
    bool via_v4l2 = false;
    std::string v4l2_error;   // 폴백이 실패한 사유 (비어 있으면 시도 안 했거나 성공)
    std::string device, firmware;
    uint16_t status_code = 0;
    unsigned long long frames = 0, drops = 0, reopens = 0;
    double fps = 0.0;
    std::string error;
};

class ThermalStream {
public:
    // 열기에 실패해도 던지지 않는다 — required 판단은 호출자가 한다.
    explicit ThermalStream(const Config::Thermal& cfg) : cfg_(cfg) {
        openCamera();
        run_ = true;
        th_ = std::thread([this] { loop(); });
    }

    ~ThermalStream() {
        run_ = false;
        if (th_.joinable()) th_.join();
#ifdef NAVI_HAS_TMSDK
        if (cam_) { cam_->Close(); cam_.reset(); }
#endif
    }
    ThermalStream(const ThermalStream&) = delete;
    ThermalStream& operator=(const ThermalStream&) = delete;

    bool ok() const {
        std::lock_guard<std::mutex> lk(mu_);
        return st_.open;
    }

    ThermalReading latest() const {
        std::lock_guard<std::mutex> lk(mu_);
        return last_;
    }

    ThermalStatus status() const {
        std::lock_guard<std::mutex> lk(mu_);
        return st_;
    }

    // 마지막 프레임 이후 경과. open=true 인데 이 값이 크면 SDK 가 블로킹된 것이다.
    // (실측: QueryFrame 이 CamTimeout 5초를 무시하고 영영 안 돌아온 적이 있다)
    std::chrono::milliseconds sinceLastFrame() const {
        std::lock_guard<std::mutex> lk(mu_);
        if (last_frame_at_.time_since_epoch().count() == 0)
            return std::chrono::milliseconds::max();
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_frame_at_);
    }

    // 최신 프레임의 컬러맵 이미지(BMP). 새 이미지가 없으면 false.
    // 열화상은 80x60이라 BMP로도 14KB밖에 안 된다 — 인코딩 지연을 안 만드는 쪽을 택했다.
    bool image(std::vector<uint8_t>& out) {
        std::lock_guard<std::mutex> lk(mu_);
        if (!fresh_ || bmp_.empty()) return false;
        out = bmp_;
        fresh_ = false;
        return true;
    }

    // 셔터를 닫고 화면 균일도를 다시 잡는다. 드리프트가 보이면 호출한다.
    // 성공하면 빈 문자열, 아니면 사유 (조작자가 MQTT 로 직접 부르는 명령이다).
    //
    // 🔴 TmSDK 없이 CDC 로 직접 친다. 제어 채널(ttyACM0)은 UVC(영상)와 별개
    //    인터페이스라 **v4l2 로 읽는 중에도 쓸 수 있다** — SDK 를 열면 UVC 를
    //    점유해 v4l2 가 막히는 문제를 이걸로 피한다. 명령 바이트는 SDK 통신을
    //    strace 로 떠서 확인했다 (2026-08-11).
    std::string runFFC() {
        std::lock_guard<std::mutex> lk(mu_);
#ifdef NAVI_HAS_TMSDK
        // SDK 모드로 열려 있으면 그쪽을 쓴다 (이미 포트를 쥐고 있다)
        if (cfg_.use_sdk && cam_) {
            auto* ctl = cam_->GetTmControl();
            if (!ctl || !ctl->RunFlatFieldCorrection()) return "SDK FFC 실패";
            return "";
        }
#endif
        return ffcViaCdc();
    }

private:
    // ── CDC 제어 채널 (FFC) ────────────────────────────────────────
    //
    // 프레임 형식 (SDK 통신을 strace 로 확인):
    //   01 02 00 C0 02 <CMD> 00 00 <CS> 03
    //   CS = 0xC2 XOR CMD      (0x42→0x80, 0x3C→0xFE, 0x48→0x8A 로 검증)
    // 요청을 그대로 에코해 주면 접수된 것이다.
    static constexpr uint8_t kCmdRunFfc = 0x42;

    static std::array<uint8_t, 10> ctrlFrame(uint8_t cmd) {
        return {0x01, 0x02, 0x00, 0xC0, 0x02, cmd, 0x00, 0x00,
                static_cast<uint8_t>(0xC2 ^ cmd), 0x03};
    }

    std::string ffcViaCdc() {
        const int fd = ::open(cfg_.cdc_port, O_RDWR | O_NOCTTY);
        if (fd < 0)
            return std::string("제어 포트 열기 실패 ") + cfg_.cdc_port + ": " + std::strerror(errno);

        termios t{};
        if (::tcgetattr(fd, &t) == 0) {
            ::cfmakeraw(&t);
            ::cfsetispeed(&t, B115200);
            ::cfsetospeed(&t, B115200);
            t.c_cflag |= (CLOCAL | CREAD);
            t.c_cc[VMIN] = 0;
            t.c_cc[VTIME] = 10;      // 1초
            ::tcsetattr(fd, TCSANOW, &t);
        }
        ::tcflush(fd, TCIFLUSH);

        const auto req = ctrlFrame(kCmdRunFfc);
        if (::write(fd, req.data(), req.size()) != static_cast<ssize_t>(req.size())) {
            ::close(fd);
            return "FFC 명령 전송 실패";
        }
        // 셔터가 닫혔다 열리는 데 시간이 걸린다. 에코가 오면 접수된 것이다.
        uint8_t rsp[32]{};
        const auto n = ::read(fd, rsp, sizeof rsp);
        ::close(fd);
        if (n <= 0) return "FFC 응답 없음 — 장치가 제어 포트를 열어두는지 확인할 것";
        if (n < static_cast<ssize_t>(req.size()) ||
            std::memcmp(rsp, req.data(), req.size()) != 0)
            return "FFC 응답이 다르다 (프로토콜 변경 가능성)";
        return "";
    }

    // ── v4l2 폴백 ──────────────────────────────────────────────────
    //
    // SDK 가 프레임을 안 줄 때 커널 UVC 로 직접 읽는다. 같은 장치인데 이쪽은 잘 된다.
    // Y16 80x60 @ 9fps — 픽셀이 raw 온도값(켈빈×100)이라 변환식만 알면 SDK 없이도 쓴다.
    //     ℃ = raw / 100 - 273.15
    // (실측 대조: raw 29765 → 24.5℃, SDK GetTemperature() 결과와 일치)
    void tryV4l2() {
        try {
            // 카드 이름으로 찾는다 — /dev/videoN 은 USB 꽂는 순서로 바뀐다
            const auto path = cam::find("TMC80F");
            auto c = std::make_unique<cam::Camera>(path, cfg_.width, cfg_.height,
                                                   cam::kY16, 9);
            v4l2_ = std::move(c);
            std::lock_guard<std::mutex> lk(mu_);
            st_.via_v4l2 = true;
            st_.error = "SDK 프레임 없음 — v4l2 로 전환";
        } catch (const std::exception& e) {
            // 조용히 삼키면 왜 폴백이 안 되는지 알 수가 없다 — 사유를 남긴다
            v4l2_.reset();
            std::lock_guard<std::mutex> lk(mu_);
            if (st_.v4l2_error.empty()) st_.v4l2_error = e.what();
        }
    }

    bool grabV4l2() {
        cam::Frame f;
        if (!v4l2_->grab(f, cam::Ms(300)) || !f.data) return false;
        const auto* px = reinterpret_cast<const uint16_t*>(f.data);
        const size_t n = f.size / 2;
        if (n < static_cast<size_t>(cfg_.width) * cfg_.height) return false;

        uint32_t lo = 0xFFFF, hi = 0;
        uint64_t sum = 0;
        for (size_t i = 0; i < n; ++i) {
            const uint16_t v = px[i];
            if (v < lo) lo = v;
            if (v > hi) hi = v;
            sum += v;
        }
        const auto toC = [](double raw) { return raw / 100.0 - 273.15; };
        const size_t mid = (static_cast<size_t>(cfg_.height) / 2) * cfg_.width + cfg_.width / 2;

        ThermalReading r;
        r.width = cfg_.width;
        r.height = cfg_.height;
        r.lo = toC(lo);
        r.hi = toC(hi);
        r.avg = toC(static_cast<double>(sum) / n);
        r.center = toC(px[mid < n ? mid : 0]);
        r.stamp = std::chrono::steady_clock::now();
        r.valid = true;

        std::vector<uint8_t> bmp;
        if (cfg_.keep_image) bmp = grayToBmp(px, cfg_.width, cfg_.height, lo, hi);

        std::lock_guard<std::mutex> lk(mu_);
        last_ = r;
        ++st_.frames;
        st_.fault = false;
        last_frame_at_ = r.stamp;
        if (!bmp.empty()) { bmp_.swap(bmp); fresh_ = true; }
        return true;
    }

    // raw 16bit → 흑백 BMP. SDK 의 ToBitmap(컬러맵)을 못 쓰므로 여기서 만든다.
    // 관측된 최저~최고로 정규화한다 — 절대값으로 하면 실내에선 거의 단색이 된다.
    static std::vector<uint8_t> grayToBmp(const uint16_t* px, int w, int h,
                                          uint32_t lo, uint32_t hi) {
        const double span = (hi > lo) ? static_cast<double>(hi - lo) : 1.0;
        std::vector<uint8_t> rgb(static_cast<size_t>(w) * h * 3);
        for (int i = 0; i < w * h; ++i) {
            const auto g = static_cast<uint8_t>((px[i] - lo) / span * 255.0);
            rgb[i * 3 + 0] = rgb[i * 3 + 1] = rgb[i * 3 + 2] = g;
        }
        return toBmp(rgb.data(), w, h);
    }

    bool openCamera() {
        // 🔴 기본은 v4l2 다. SDK 는 열지 않는다 (2026-08-10).
        //
        //    SDK Open() 이 UVC 장치를 점유해 버려서, 같은 장치를 v4l2 로 열면 EBUSY 다.
        //    그런데 정작 SDK QueryFrame 은 이 조합에서 못 쓴다 —
        //    40회에 1회 성공하고 나머지는 실패도 아닌 **블로킹**이다.
        //    (tm_watch 12초짜리가 400초를 기다려도 안 끝났다)
        //
        //    v4l2 로 직접 읽으면 7.7fps 로 멀쩡하고 커널 UVC 로그도 깨끗하다.
        //    Y16 픽셀이 raw 온도(켈빈×100)라 SDK 없이도 값을 다 얻는다.
        //
        //    ⚠ 잃는 것: FFC(셔터 보정)와 SDK 컬러맵. FFC 는 CDC(ttyACM0) 프로토콜을
        //      알아야 직접 칠 수 있는데 아직 모른다. 컬러맵은 흑백으로 대체했다.
        //      SDK 가 고쳐지면 use_sdk=1 로 되돌리면 된다.
#ifndef NAVI_HAS_TMSDK
        // SDK 없이 빌드된 경우. use_sdk=1 이어도 칠 수가 없으니 v4l2 로 간다.
        const bool sdk_path = false;
        if (cfg_.use_sdk) {
            std::lock_guard<std::mutex> lk(mu_);
            st_.error = "TmSDK 없는 빌드 — thermal_use_sdk=1 을 무시하고 v4l2 로 연다";
        }
#else
        const bool sdk_path = cfg_.use_sdk;
#endif
        if (!sdk_path) {
            tryV4l2();
            std::lock_guard<std::mutex> lk(mu_);
            st_.open = static_cast<bool>(v4l2_);
            if (!st_.open && st_.error.empty())
                st_.error = st_.v4l2_error.empty() ? "v4l2 열기 실패" : st_.v4l2_error;
            return st_.open;
        }
#ifdef NAVI_HAS_TMSDK
        try {
            auto list = TmSDK::TmLocalCamera::GetCameraList();
            if (list.empty()) throw std::runtime_error("장치를 찾지 못했다");
            auto c = std::make_unique<TmSDK::TmLocalCamera>();
            list[0].CamTimeout = 5000;          // 공식 예제가 Open 전에 설정한다
            if (!c->Open(&list[0])) throw std::runtime_error("Open 실패");
            for (int i = 0; i < 40 && !c->IsConnected(); ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

            c->SetTempUnit(TmSDK::TempUnit::CELSIUS);
            c->SetColorMap(static_cast<TmSDK::ColormapTypes>(cfg_.colormap));

            std::lock_guard<std::mutex> lk(mu_);
            cam_ = std::move(c);
            st_.open = true;
            st_.error.clear();
            st_.device = cam_->GetDevName();
            if (auto* ctl = cam_->GetTmControl()) {
                st_.firmware = ctl->GetFirmwareVersion();
                st_.status_code = std::get<0>(ctl->GetSystemStatus());
                if (cfg_.ffc_on_open) ctl->RunFlatFieldCorrection();
            }
            return true;
        } catch (const std::exception& e) {
            std::lock_guard<std::mutex> lk(mu_);
            cam_.reset();
            st_.open = false;
            st_.error = e.what();
            return false;
        }
#endif
    }

    void loop() {
        using Clock = std::chrono::steady_clock;
        auto win = Clock::now();
        unsigned in_win = 0;
#ifdef NAVI_HAS_TMSDK
        int fails = 0;
        TmSDK::TmFrame f;
        const bool sdk_path = cfg_.use_sdk;
#else
        const bool sdk_path = false;   // SDK 없는 빌드는 v4l2 만 돈다
#endif
        while (run_) {
            // v4l2 전용 모드 — SDK 는 아예 안 쓴다
            if (!sdk_path) {
                if (!v4l2_) {
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    if (run_) { tryV4l2(); if (v4l2_) { std::lock_guard<std::mutex> lk(mu_); st_.open = true; ++st_.reopens; } }
                    continue;
                }
                if (grabV4l2()) {
                    if (++in_win; Clock::now() - win >= std::chrono::seconds(1)) {
                        const double sec = std::chrono::duration<double>(Clock::now() - win).count();
                        std::lock_guard<std::mutex> lk(mu_);
                        st_.fps = in_win / sec;
                        in_win = 0;
                        win = Clock::now();
                    }
                } else {
                    std::lock_guard<std::mutex> lk(mu_);
                    ++st_.drops;
                }
                continue;
            }

#ifdef NAVI_HAS_TMSDK
            if (!cam_) {
                std::this_thread::sleep_for(std::chrono::seconds(2));
                if (run_ && openCamera()) {
                    fails = 0;
                    std::lock_guard<std::mutex> lk(mu_);
                    ++st_.reopens;
                }
                continue;
            }

            // 🔴 프레임은 v4l2 로 읽는다. SDK 는 CDC 제어(FFC·상태)에만 쓴다 (2026-08-10).
            //
            //    SDK QueryFrame 은 이 조합에서 못 쓴다 — 실측:
            //        40회 시도에 1회 성공, 나머지는 실패도 아니고 **블로킹**된다.
            //        (tm_watch 12초짜리가 400초를 기다려도 안 끝났다)
            //    실패를 반환하면 폴백이라도 걸 텐데, 안 돌아오니 그것도 안 된다.
            //
            //    같은 장치를 v4l2 로 열면 **7.7fps 로 정상**이고 커널 UVC 로그도 깨끗하다.
            //    Y16 픽셀이 raw 온도(켈빈×100)라 변환식만 알면 SDK 없이 다 된다.
            //
            //    ⚠ 3A 시절엔 반대였다 — v4l2 단독으로는 한 장도 못 받아
            //      "SDK 가 CDC 로 스트림을 켜줘야 한다"고 적어뒀다. 지금은 SDK 가 Open 하며
            //      스트림을 켜 둔 상태라 v4l2 가 받을 수 있는 것으로 본다.
            //      그래서 SDK Open 은 유지하고 프레임만 v4l2 로 가져온다.
            if (!v4l2_) tryV4l2();
            if (v4l2_ && grabV4l2()) {
                fails = 0;
                if (++in_win; Clock::now() - win >= std::chrono::seconds(1)) {
                    const double sec = std::chrono::duration<double>(Clock::now() - win).count();
                    std::lock_guard<std::mutex> lk(mu_);
                    st_.fps = in_win / sec;
                    in_win = 0;
                    win = Clock::now();
                }
                continue;
            }

            if (!cam_->QueryFrame(&f, cfg_.width, cfg_.height)) {
                {
                    std::lock_guard<std::mutex> lk(mu_);
                    ++st_.drops;
                }
                // 9fps라 110ms 주기다. 연속 실패가 이어지면 장치가 빠진 것으로 본다.
                //
                // 🔴 실패가 "빠짐"인지 "센서 고장"인지 구분해야 한다.
                //    실측(2026-08-10): 센서가 살아서 통신은 되는데 이미징만 죽은 상태가 있었다.
                //        펌웨어 F.3.1.0 은 읽히는데
                //        상태 0x0500 / 에러 0x0600 = "The device is not functioning"
                //        프레임은 1장 오고 그 뒤 108회 연속 실패
                //    이 경우 재연결해도 소용없다 — 사람이 봐야 한다. 그래서 상태를 물어
                //    error 에 실어 두고, 상위가 알람으로 올릴 수 있게 한다.
                if (++fails >= kReopenAfter) {
                    std::string why = "연속 " + std::to_string(fails) + "회 실패";
                    // 통신이 살아 있으면 센서에게 직접 상태를 물어본다
                    if (auto* ctl = cam_->GetTmControl()) {
                        const auto [sc, stext] = ctl->GetSystemStatus();
                        const auto [ec, etext] = ctl->GetSystemError();
                        if (sc || ec) {
                            char buf[192];
                            std::snprintf(buf, sizeof buf,
                                " — 센서 상태 0x%04X(%s) 에러 0x%04X(%s). "
                                "통신은 되는데 이미징이 안 된다 (재연결로 안 풀린다)",
                                sc, stext.c_str(), ec, etext.c_str());
                            why += buf;
                            std::lock_guard<std::mutex> lk(mu_);
                            st_.status_code = sc;
                            st_.fault = true;      // 하드웨어 고장 — 상위가 알람으로 올린다
                        }
                    }
                    std::lock_guard<std::mutex> lk(mu_);
                    if (cam_) cam_->Close();
                    cam_.reset();
                    st_.open = false;
                    st_.error = why + " — 재연결 시도";
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(40));
                continue;
            }
            fails = 0;
            {
                // 프레임이 다시 오면 고장 표시를 내린다 (빠졌다 다시 꽂은 경우)
                std::lock_guard<std::mutex> lk(mu_);
                st_.fault = false;
                last_frame_at_ = Clock::now();
            }

            double lo = 0, avg = 0, hi = 0;
            TmSDK::Point loP, hiP;
            f.MinMaxLoc(lo, avg, hi, loP, hiP);

            ThermalReading r;
            r.width = f.width;
            r.height = f.height;
            // ⚠ MinMaxLoc/GetPixel은 raw(켈빈×100)를 준다. GetTemperature()를 거쳐야 ℃다.
            r.lo = cam_->GetTemperature(lo);
            r.avg = cam_->GetTemperature(avg);
            r.hi = cam_->GetTemperature(hi);
            r.center = cam_->GetTemperature(f.GetPixel(f.width / 2, f.height / 2));
            r.stamp = Clock::now();
            r.valid = true;

            std::vector<uint8_t> bmp;
            if (cfg_.keep_image) {
                if (uint8_t* bgr = f.ToBitmap(TmSDK::ColorOrder::COLOR_BGR))
                    bmp = toBmp(bgr, f.width, f.height);
            }

            {
                std::lock_guard<std::mutex> lk(mu_);
                last_ = r;
                ++st_.frames;
                if (!bmp.empty()) { bmp_.swap(bmp); fresh_ = true; }
            }

            // 🔴 반드시 호출한다 — 안 하면 프레임마다 SDK 버퍼가 쌓인다.
            //    이걸 빠뜨려 tm_view 가 417MB까지 불어났고, 결국 커널 OOM 이 떠서
            //    화면은 나오는데 유저스페이스만 마비되는 상태가 됐다 (2026-07-31).
            f.Release();

            if (++in_win; Clock::now() - win >= std::chrono::seconds(1)) {
                const double sec = std::chrono::duration<double>(Clock::now() - win).count();
                std::lock_guard<std::mutex> lk(mu_);
                st_.fps = in_win / sec;
                in_win = 0;
                win = Clock::now();
            }
#endif   // NAVI_HAS_TMSDK — 여기까지가 SDK 프레임 경로
        }
    }

    // 24bit BMP. 80x60이면 14KB — 브라우저·상위 어디에 던져도 부담이 없다.
    static std::vector<uint8_t> toBmp(const uint8_t* bgr, int w, int h) {
        const int row = w * 3;
        const int pad = (4 - (row % 4)) % 4;
        const int dataSize = (row + pad) * h;
        std::vector<uint8_t> b(54 + dataSize, 0);
        auto p32 = [&](int o, uint32_t v) { std::memcpy(&b[o], &v, 4); };
        auto p16 = [&](int o, uint16_t v) { std::memcpy(&b[o], &v, 2); };
        b[0] = 'B'; b[1] = 'M';
        p32(2, static_cast<uint32_t>(b.size()));
        p32(10, 54); p32(14, 40);
        p32(18, static_cast<uint32_t>(w));
        p32(22, static_cast<uint32_t>(h));      // 양수 = bottom-up
        p16(26, 1); p16(28, 24);
        p32(34, static_cast<uint32_t>(dataSize));
        for (int y = 0; y < h; ++y)
            std::memcpy(&b[54 + static_cast<size_t>(y) * (row + pad)],
                        bgr + static_cast<size_t>(h - 1 - y) * row, row);
        return b;
    }

    static constexpr int kFallbackAfter = 5;    // SDK 5회 실패하면 v4l2 를 시도한다
    static constexpr int kReopenAfter = 20;   // 110ms 주기 × 20 ≈ 2초

    Config::Thermal cfg_;
#ifdef NAVI_HAS_TMSDK
    std::unique_ptr<TmSDK::TmLocalCamera> cam_;
#endif
    std::unique_ptr<cam::Camera> v4l2_;   // 기본 경로 (SDK 는 선택)
    std::thread th_;
    std::atomic<bool> run_{false};

    mutable std::mutex mu_;
    std::chrono::steady_clock::time_point last_frame_at_{};
    ThermalReading last_{};
    ThermalStatus st_{};
    std::vector<uint8_t> bmp_;
    bool fresh_ = false;
};

}  // namespace navi
