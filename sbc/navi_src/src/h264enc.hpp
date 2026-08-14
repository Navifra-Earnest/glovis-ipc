// h264enc.hpp — Rockchip MPP 하드웨어 H.264 인코더.
//
//   navi::H264Enc enc(1920, 1080, 30, 2'000'000);
//   std::vector<uint8_t> pkt;
//   if (enc.encode(uyvy_data, uyvy_size, pkt)) { /* pkt 를 내보낸다 */ }
//
// 🔴 이게 navi 안에 있어야 하는 이유:
//    UVC 카메라는 **한 프로세스만 연다**(실측: 두 번째 프로세스는 EBUSY).
//    인코딩을 gstreamer 같은 별도 프로세스로 빼면 navi 가 카메라를 놓아야 하고,
//    그러면 IR·열화상·ToF 를 계속 보내라는 요구를 지킬 수 없다.
//    navi 가 카메라를 쥔 채로 직접 인코딩해야 둘 다 된다.
//
// 입력은 카메라가 주는 UYVY 를 그대로 넣는다 — 색공간 변환(CPU)이 끼지 않는다.
// VPU 가 처리하므로 1080p30 에서도 CPU 부하는 미미하다.
#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <rockchip/rk_mpi.h>

namespace navi {

class H264Enc {
public:
    H264Enc(int width, int height, int fps, int bps, int gop = 0)
        : w_(width), h_(height) {
        if (gop <= 0) gop = fps;                 // 1초에 한 번 IDR
        // UYVY 는 픽셀당 2바이트다. stride 는 하드웨어 정렬을 맞춘다.
        hor_stride_ = align(w_ * 2, 16);
        ver_stride_ = align(h_, 16);
        frame_size_ = static_cast<size_t>(hor_stride_) * ver_stride_;

        if (mpp_create(&ctx_, &mpi_) != MPP_OK) fail("mpp_create 실패");
        if (mpp_init(ctx_, MPP_CTX_ENC, MPP_VIDEO_CodingAVC) != MPP_OK) fail("mpp_init 실패");

        MppEncCfg cfg = nullptr;
        if (mpp_enc_cfg_init(&cfg) != MPP_OK) fail("enc_cfg_init 실패");

        mpp_enc_cfg_set_s32(cfg, "prep:width", w_);
        mpp_enc_cfg_set_s32(cfg, "prep:height", h_);
        mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", hor_stride_);
        mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", ver_stride_);
        mpp_enc_cfg_set_s32(cfg, "prep:format", MPP_FMT_YUV422_UYVY);

        // CBR — 무선으로도 보내야 하므로 비트레이트가 튀지 않는 쪽이 낫다.
        mpp_enc_cfg_set_s32(cfg, "rc:mode", MPP_ENC_RC_MODE_CBR);
        mpp_enc_cfg_set_s32(cfg, "rc:bps_target", bps);
        mpp_enc_cfg_set_s32(cfg, "rc:bps_max", bps * 17 / 16);
        mpp_enc_cfg_set_s32(cfg, "rc:bps_min", bps * 15 / 16);
        mpp_enc_cfg_set_s32(cfg, "rc:fps_in_flex", 0);
        mpp_enc_cfg_set_s32(cfg, "rc:fps_in_num", fps);
        mpp_enc_cfg_set_s32(cfg, "rc:fps_in_denorm", 1);
        mpp_enc_cfg_set_s32(cfg, "rc:fps_out_flex", 0);
        mpp_enc_cfg_set_s32(cfg, "rc:fps_out_num", fps);
        mpp_enc_cfg_set_s32(cfg, "rc:fps_out_denorm", 1);
        mpp_enc_cfg_set_s32(cfg, "rc:gop", gop);

        mpp_enc_cfg_set_s32(cfg, "codec:type", MPP_VIDEO_CodingAVC);
        mpp_enc_cfg_set_s32(cfg, "h264:profile", 100);      // High
        mpp_enc_cfg_set_s32(cfg, "h264:level", 40);         // 1080p@30
        mpp_enc_cfg_set_s32(cfg, "h264:cabac_en", 1);
        mpp_enc_cfg_set_s32(cfg, "h264:cabac_idc", 0);
        mpp_enc_cfg_set_s32(cfg, "h264:trans8x8", 1);

        const MPP_RET rc = mpi_->control(ctx_, MPP_ENC_SET_CFG, cfg);
        mpp_enc_cfg_deinit(cfg);
        if (rc != MPP_OK) fail("MPP_ENC_SET_CFG 실패");

        // SPS/PPS 를 미리 뽑아 둔다 — 중간에 붙는 클라이언트에게 먼저 보내야 그림이 나온다.
        MppPacket hdr = nullptr;
        mpp_packet_init(&hdr, hdr_buf_, sizeof hdr_buf_);
        mpp_packet_set_length(hdr, 0);
        if (mpi_->control(ctx_, MPP_ENC_GET_HDR_SYNC, hdr) == MPP_OK) {
            const auto* p = static_cast<const uint8_t*>(mpp_packet_get_pos(hdr));
            header_.assign(p, p + mpp_packet_get_length(hdr));
        }
        mpp_packet_deinit(&hdr);

        if (mpp_buffer_group_get_internal(&grp_, MPP_BUFFER_TYPE_DRM) != MPP_OK)
            fail("buffer_group 실패");
        if (mpp_buffer_get(grp_, &buf_, frame_size_) != MPP_OK)
            fail("입력 버퍼 확보 실패");
    }

    ~H264Enc() {
        if (buf_) mpp_buffer_put(buf_);
        if (grp_) mpp_buffer_group_put(grp_);
        if (ctx_) { mpi_->reset(ctx_); mpp_destroy(ctx_); }
    }
    H264Enc(const H264Enc&) = delete;
    H264Enc& operator=(const H264Enc&) = delete;

    // SPS/PPS. 새 클라이언트에게 이걸 먼저 보낸다.
    const std::vector<uint8_t>& header() const { return header_; }

    int width() const { return w_; }
    int height() const { return h_; }

    // UYVY 한 장을 넣고 H.264 패킷을 받는다. 패킷이 없으면 false (드문 경우다).
    // out 은 매번 덮어쓴다.
    bool encode(const void* uyvy, size_t size, std::vector<uint8_t>& out, bool* is_key = nullptr) {
        if (!uyvy || size == 0) return false;

        // 카메라 stride 와 인코더 stride 가 다르면 줄 단위로 옮긴다.
        auto* dst = static_cast<uint8_t*>(mpp_buffer_get_ptr(buf_));
        const auto* src = static_cast<const uint8_t*>(uyvy);
        const int src_stride = w_ * 2;
        if (src_stride == hor_stride_) {
            std::memcpy(dst, src, std::min(size, frame_size_));
        } else {
            const int rows = static_cast<int>(std::min<size_t>(h_, size / src_stride));
            for (int y = 0; y < rows; ++y)
                std::memcpy(dst + static_cast<size_t>(y) * hor_stride_,
                            src + static_cast<size_t>(y) * src_stride, src_stride);
        }

        MppFrame frame = nullptr;
        if (mpp_frame_init(&frame) != MPP_OK) return false;
        mpp_frame_set_width(frame, w_);
        mpp_frame_set_height(frame, h_);
        mpp_frame_set_hor_stride(frame, hor_stride_);
        mpp_frame_set_ver_stride(frame, ver_stride_);
        mpp_frame_set_fmt(frame, MPP_FMT_YUV422_UYVY);
        mpp_frame_set_buffer(frame, buf_);
        mpp_frame_set_eos(frame, 0);

        if (mpi_->encode_put_frame(ctx_, frame) != MPP_OK) {
            mpp_frame_deinit(&frame);
            return false;
        }

        MppPacket packet = nullptr;
        const MPP_RET rc = mpi_->encode_get_packet(ctx_, &packet);
        mpp_frame_deinit(&frame);
        if (rc != MPP_OK || !packet) return false;

        const auto* p = static_cast<const uint8_t*>(mpp_packet_get_pos(packet));
        const size_t len = mpp_packet_get_length(packet);
        out.assign(p, p + len);
        // 키프레임 플래그 상수가 MPP 버전마다 달라서 NAL 로 직접 본다 (타입 5 = IDR).
        if (is_key) *is_key = hasIdr(p, len);
        mpp_packet_deinit(&packet);
        return len > 0;
    }

    // byte-stream 안에 IDR(NAL 타입 5)이 있는가. 시작코드는 00 00 01 또는 00 00 00 01.
    static bool hasIdr(const uint8_t* p, size_t n) {
        for (size_t i = 0; i + 3 < n; ++i)
            if (p[i] == 0 && p[i + 1] == 0 && p[i + 2] == 1 && (p[i + 3] & 0x1F) == 5)
                return true;
        return false;
    }

private:
    static int align(int v, int a) { return (v + a - 1) & ~(a - 1); }
    [[noreturn]] void fail(const char* msg) {
        if (ctx_) { mpp_destroy(ctx_); ctx_ = nullptr; }
        throw std::runtime_error(std::string("H264Enc: ") + msg);
    }

    int w_, h_, hor_stride_ = 0, ver_stride_ = 0;
    size_t frame_size_ = 0;
    MppCtx ctx_ = nullptr;
    MppApi* mpi_ = nullptr;
    MppBufferGroup grp_ = nullptr;
    MppBuffer buf_ = nullptr;
    std::vector<uint8_t> header_;
    uint8_t hdr_buf_[1024]{};
};

}  // namespace navi
