// uvccam.hpp — V4L2 UVC 카메라 캡처 (See3CAM_CU27 / USB 열화상 공용)
//
// mmap 제로카피. grab()이 준 포인터는 다음 grab()까지만 유효하다 (그때 커널에 반환된다).
// 디코딩은 하지 않는다 — Y16·YUYV·UYVY는 raw로 바로 쓰고, MJPEG는 상위에서 풀면 된다.
//
//   auto path = navi::cam::find("See3CAM");        // 카드 이름으로 찾는다
//   navi::cam::Camera cam(path, 1920, 1080, navi::cam::kUYVY);
//   cam.start();
//   navi::cam::Frame f;
//   while (cam.grab(f, navi::cam::Ms(200))) { /* f.data, f.size */ }
//
// 열화상(Y16)은 픽셀이 raw 온도 카운트다. ℃ 변환식은 제조사 스펙이 있어야 한다.
//
// 하드웨어 주의: See3CAM_CU27은 FHD@60fps UYVY에서 ~249MB/s를 쓴다. Rock 3A의 USB3 포트는
// Bus 03 / Bus 08 각 1개뿐이니, 열화상과 같은 허브에 물리지 말고 서로 다른 포트에 꽂아야 한다.
#pragma once

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>

namespace navi::cam {

using Ms = std::chrono::milliseconds;

inline constexpr uint32_t fourcc(char a, char b, char c, char d) {
    return static_cast<uint32_t>(a) | (static_cast<uint32_t>(b) << 8)
         | (static_cast<uint32_t>(c) << 16) | (static_cast<uint32_t>(d) << 24);
}
inline constexpr uint32_t kMJPEG = fourcc('M', 'J', 'P', 'G');
inline constexpr uint32_t kYUYV  = fourcc('Y', 'U', 'Y', 'V');
inline constexpr uint32_t kUYVY  = fourcc('U', 'Y', 'V', 'Y');
inline constexpr uint32_t kY16   = fourcc('Y', '1', '6', ' ');   // 열화상 raw 16bit
inline constexpr uint32_t kGREY  = fourcc('G', 'R', 'E', 'Y');

inline std::string fourccStr(uint32_t f) {
    return {static_cast<char>(f & 0xFF), static_cast<char>((f >> 8) & 0xFF),
            static_cast<char>((f >> 16) & 0xFF), static_cast<char>((f >> 24) & 0xFF)};
}

struct Frame {
    const uint8_t* data = nullptr;
    size_t size = 0;
    uint32_t width = 0, height = 0, format = 0;
    uint32_t sequence = 0;
    timeval ts{};
};

// 카드 이름 부분일치로 장치를 찾는다. USB 꽂는 순서에 따라 videoN이 바뀌므로
// 경로를 하드코딩하지 말 것. 하드웨어 코덱(video-dec0 등)은 건너뛴다.
inline std::string find(const std::string& card_substr) {
    for (const auto& e : std::filesystem::directory_iterator("/dev")) {
        const auto name = e.path().filename().string();
        if (name.rfind("video", 0) != 0) continue;
        if (name.find("dec") != std::string::npos ||
            name.find("enc") != std::string::npos) continue;
        const int fd = ::open(e.path().c_str(), O_RDWR);
        if (fd < 0) continue;
        v4l2_capability cap{};
        const bool ok = ::ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0;
        ::close(fd);
        if (!ok) continue;
        const std::string cardname(reinterpret_cast<const char*>(cap.card));
        if (cardname.find(card_substr) != std::string::npos &&
            (cap.device_caps & V4L2_CAP_VIDEO_CAPTURE))
            return e.path().string();
    }
    throw std::runtime_error("카메라를 찾지 못했다: " + card_substr);
}

// 장치가 실제로 지원하는 (포맷, 해상도) 목록. 사양을 모를 때 이걸로 확인한다.
struct FormatInfo {
    uint32_t format;
    uint32_t width, height;
};
inline std::vector<FormatInfo> formats(const std::string& path) {
    const int fd = ::open(path.c_str(), O_RDWR);
    if (fd < 0) throw std::runtime_error("열기 실패: " + path);
    std::vector<FormatInfo> out;
    v4l2_fmtdesc fd_desc{};
    fd_desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (fd_desc.index = 0; ::ioctl(fd, VIDIOC_ENUM_FMT, &fd_desc) == 0; ++fd_desc.index) {
        v4l2_frmsizeenum fs{};
        fs.pixel_format = fd_desc.pixelformat;
        fs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        for (fs.index = 0; ::ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fs) == 0; ++fs.index) {
            if (fs.type == V4L2_FRMSIZE_TYPE_DISCRETE)
                out.push_back({fd_desc.pixelformat, fs.discrete.width, fs.discrete.height});
        }
    }
    ::close(fd);
    return out;
}

class Camera {
public:
    // fps=0 이면 드라이버 기본값. nbuf는 지연/드롭 절충 — 4면 충분하다.
    Camera(const std::string& path, uint32_t width, uint32_t height,
           uint32_t format, uint32_t fps = 0, unsigned nbuf = 4)
        : path_(path) {
        fd_ = ::open(path.c_str(), O_RDWR | O_NONBLOCK);
        if (fd_ < 0) throw std::runtime_error("열기 실패: " + path);

        v4l2_capability cap{};
        if (xioctl(VIDIOC_QUERYCAP, &cap) < 0) fail("QUERYCAP 실패");
        if (!(cap.device_caps & V4L2_CAP_VIDEO_CAPTURE)) fail("캡처 장치가 아니다");
        if (!(cap.device_caps & V4L2_CAP_STREAMING)) fail("스트리밍 미지원");
        card_ = reinterpret_cast<const char*>(cap.card);

        v4l2_format f{};
        f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        f.fmt.pix.width = width;
        f.fmt.pix.height = height;
        f.fmt.pix.pixelformat = format;
        f.fmt.pix.field = V4L2_FIELD_NONE;
        if (xioctl(VIDIOC_S_FMT, &f) < 0) fail("S_FMT 실패");
        // 드라이버가 요청을 조정할 수 있다 — 실제 값을 받아 쓴다
        w_ = f.fmt.pix.width;
        h_ = f.fmt.pix.height;
        fmt_ = f.fmt.pix.pixelformat;
        if (fmt_ != format)
            throw std::runtime_error("포맷 거부됨: 요청 " + fourccStr(format)
                                     + " → 실제 " + fourccStr(fmt_));

        // 🔴 VIDIOC_S_PARM 은 쓰지 않는다 (실측 근거).
        //
        //    See3CAM의 MJPG는 100fps 고정인데 30fps를 요청하면 스트림이 망가진다 —
        //    첫 프레임이 0바이트로 오고 30장 중 20회가 타임아웃이었다.
        //    그런데 드라이버는 S_PARM 을 **성공으로 보고하고 요청값을 그대로 돌려주므로**
        //    반환값으로는 지원 여부를 가릴 수 없다. 되돌리는 방어도 통하지 않았다.
        //
        //    캡처는 카메라가 내주는 속도 그대로 받고(커널 버퍼를 비워 지연을 막는 효과도 있다),
        //    필요한 fps 제한은 상위에서 프레임을 솎아서 한다. cam_view 가 그렇게 하고 있다.
        (void)fps;

        v4l2_requestbuffers rb{};
        rb.count = nbuf;
        rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        rb.memory = V4L2_MEMORY_MMAP;
        if (xioctl(VIDIOC_REQBUFS, &rb) < 0) fail("REQBUFS 실패");
        if (rb.count < 2) fail("버퍼가 부족하다");

        bufs_.resize(rb.count);
        for (unsigned i = 0; i < rb.count; ++i) {
            v4l2_buffer b{};
            b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            b.memory = V4L2_MEMORY_MMAP;
            b.index = i;
            if (xioctl(VIDIOC_QUERYBUF, &b) < 0) fail("QUERYBUF 실패");
            bufs_[i].len = b.length;
            bufs_[i].p = static_cast<uint8_t*>(
                ::mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, b.m.offset));
            if (bufs_[i].p == MAP_FAILED) fail("mmap 실패");
        }
    }

    ~Camera() {
        try { stop(); } catch (...) {}
        for (auto& b : bufs_) if (b.p && b.p != MAP_FAILED) ::munmap(b.p, b.len);
        if (fd_ >= 0) ::close(fd_);
    }
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    void start() {
        if (streaming_) return;
        for (unsigned i = 0; i < bufs_.size(); ++i) queue(i);
        v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(VIDIOC_STREAMON, &t) < 0) fail("STREAMON 실패");
        streaming_ = true;
    }

    void stop() {
        if (!streaming_) return;
        v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(VIDIOC_STREAMOFF, &t);
        streaming_ = false;
        held_ = -1;
    }

    // 프레임 1장. timeout 안에 안 오면 false.
    // 반환된 f.data는 다음 grab() 호출 시점에 커널로 돌아간다 — 그 전에 쓰거나 복사할 것.
    bool grab(Frame& f, Ms timeout = Ms(1000)) {
        if (!streaming_) start();
        if (held_ >= 0) { queue(static_cast<unsigned>(held_)); held_ = -1; }

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        timeval tv{};
        tv.tv_sec = timeout.count() / 1000;
        tv.tv_usec = (timeout.count() % 1000) * 1000;
        const int r = ::select(fd_ + 1, &fds, nullptr, nullptr, &tv);
        if (r <= 0) return false;                 // 타임아웃 또는 인터럽트

        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        if (xioctl(VIDIOC_DQBUF, &b) < 0) return false;

        held_ = static_cast<int>(b.index);
        f.data = bufs_[b.index].p;
        f.size = b.bytesused;
        f.width = w_;
        f.height = h_;
        f.format = fmt_;
        f.sequence = b.sequence;
        f.ts = b.timestamp;
        return true;
    }

    // UVC 표준 컨트롤. 실패하면 false (장치가 그 컨트롤을 안 가진 것).
    bool setCtrl(uint32_t id, int32_t value) {
        v4l2_control c{};
        c.id = id;
        c.value = value;
        return xioctl(VIDIOC_S_CTRL, &c) == 0;
    }
    bool getCtrl(uint32_t id, int32_t& value) {
        v4l2_control c{};
        c.id = id;
        if (xioctl(VIDIOC_G_CTRL, &c) < 0) return false;
        value = c.value;
        return true;
    }

    // 자동 노출을 끄고 수동값을 넣는다. 열화상·IR에서 프레임 타이밍을 고정할 때 쓴다.
    bool manualExposure(int32_t absolute) {
        setCtrl(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
        return setCtrl(V4L2_CID_EXPOSURE_ABSOLUTE, absolute);
    }

    const std::string& card() const { return card_; }
    const std::string& path() const { return path_; }
    uint32_t width() const { return w_; }
    uint32_t height() const { return h_; }
    uint32_t format() const { return fmt_; }
    bool streaming() const { return streaming_; }

private:
    struct Buf { uint8_t* p = nullptr; size_t len = 0; };

    int xioctl(unsigned long req, void* arg) {
        int r;
        do { r = ::ioctl(fd_, req, arg); } while (r < 0 && errno == EINTR);
        return r;
    }
    void queue(unsigned i) {
        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        xioctl(VIDIOC_QBUF, &b);
    }
    [[noreturn]] void fail(const std::string& msg) {
        throw std::runtime_error(msg + " (" + path_ + ", errno " + std::to_string(errno) + ")");
    }

    std::string path_, card_;
    int fd_ = -1;
    uint32_t w_ = 0, h_ = 0, fmt_ = 0;
    std::vector<Buf> bufs_;
    bool streaming_ = false;
    int held_ = -1;      // grab()이 호출자에게 빌려준 버퍼. 다음 grab()에서 반환한다
};

}  // namespace navi::cam
