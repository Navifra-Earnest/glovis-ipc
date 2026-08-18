#!/usr/bin/env python3
"""Glovis 화재진압로봇 IPC 콘솔 — IR 영상 + 상태/알람 + e-stop/reset.

GTK3 + GStreamer(gtksink) + paho-mqtt. 전부 IPC 에 이미 있는 것만 쓴다.

  python3 navi_console.py                 # 창 모드 (개발용)
  python3 navi_console.py --fullscreen    # 터치스크린 운용 모드. F11 토글, Esc 로 해제
  python3 navi_console.py --selftest      # 상태 파싱 자체검증 (하드웨어 불필요)

⚠️ 조이스틱 주행은 이 프로세스가 아니라 joy_teleop.py 가 담당한다. 일부러 분리했다 —
   구동 명령은 350ms 케이던스를 지켜야 하는데 GUI 렌더링에 막히면 워치독이 로봇을 세운다.
"""
import argparse, json, os, sys, threading, time

import mqtt_link

PORT, PREFIX = 1883, "navi"
VIDEO_PORT = 5000
RECONNECT_S = 2          # 영상 파이프라인 오류 후 재시도 간격
VIDEO_STALL_S = 4        # 이 시간 동안 프레임이 없으면 파이프라인을 다시 세운다


def fmt_state(d):
    """state JSON → 화면에 뿌릴 (구동, 센서, 경고) 문자열 3개. 없는 필드는 '-' 로 둔다."""
    wheels = d.get("wheels") or []
    rpm = " ".join(f"{w.get('label', '?')}{w.get('rpm', 0):+.0f}" for w in wheels) or "축 없음"
    drive = (f"구동 {'OK' if d.get('drive_ok') else 'X'} · "
             f"{d.get('wheels_alive', 0)}축 · {rpm}")

    tof = d.get("tof") or {}
    th = d.get("thermal") or {}
    cm, sig = tof.get("dist_cm"), tof.get("strength")
    if not tof.get("present"):
        dist = "거리 센서없음"
    elif not tof.get("valid"):
        # valid 는 신호강도 판정이다 — false 면 거리값 자체가 쓰레기이므로 숫자를 아예 안 띄운다
        dist = f"거리 무효(강도 {sig})"
    else:
        dist = f"거리 {cm / 100:.2f} m ({cm}cm, 강도 {sig})"
    sensor = (f"{dist}   ·   열화상 {th.get('lo', '-')}~{th.get('hi', '-')}℃ "
              f"중앙 {th.get('center', '-')} ({th.get('fps', '-')}fps)")

    warn = []
    if d.get("estop"):
        warn.append(f"E-STOP 래치: {d.get('estop_reason') or '사유 없음'} — 해제 버튼으로만 풀린다")
    if d.get("watchdog"):
        warn.append("워치독 정지")
    if not d.get("drive_ok"):
        warn.append(d.get("drive_error") or "구동계 없음")
    if tof.get("present") and not tof.get("valid"):
        warn.append("거리값 신뢰 불가 — 장애물 판단에 쓰지 말 것")
    return drive, sensor, " / ".join(warn)


# 열화상 가짜색 팔레트. 센서는 Y16 흑백이고 색은 원래 소프트웨어가 입히는 것이다.
# navi 는 컬러맵을 TmSDK 경로에서만 입히는데 그 SDK 가 고장나 use_sdk=0 이라
# 흑백 BMP 가 온다(thermal.hpp:244) → IPC 에서 입힌다. navi.conf 의 thermal_colormap 은 죽은 설정.
# ⚠️ navi 는 매 프레임 lo~hi 로 자동 정규화한다(오토게인). 그래서 전 구간에 색을 쓰는
# 팔레트(ironbow·fire·jet)는 상온 장면도 화염처럼 그린다 — 실측으로 확인했다.
# redhot 은 장면을 흑백으로 두고 상단 구간에만 색을 써서 그 오독을 피한다(소방 TIC 방식).
PALETTES = {
    "redhot":  [(0.00, 15, 15, 15), (0.55, 150, 150, 150), (0.70, 210, 205, 190),
                (0.78, 235, 120, 40), (0.88, 255, 40, 20), (1.00, 255, 240, 120)],
    "fire":    [(0.00, 0, 0, 0), (0.25, 120, 0, 0), (0.50, 220, 40, 0), (0.72, 255, 140, 0),
                (0.88, 255, 220, 60), (1.00, 255, 255, 255)],
    "ironbow": [(0.00, 0, 0, 0), (0.15, 20, 0, 80), (0.30, 100, 0, 130), (0.45, 190, 30, 90),
                (0.60, 240, 90, 20), (0.80, 255, 180, 0), (1.00, 255, 255, 255)],
    "jet":     [(0.00, 0, 0, 131), (0.125, 0, 0, 255), (0.375, 0, 255, 255),
                (0.625, 255, 255, 0), (0.875, 255, 0, 0), (1.00, 128, 0, 0)],
    "grey":    [(0.00, 0, 0, 0), (1.00, 255, 255, 255)],
}


def build_lut(stops):
    """제어점 사이를 선형보간해 256단계 R/G/B 변환표 3개를 만든다."""
    out = [bytearray(256) for _ in range(3)]
    for i in range(256):
        t = i / 255.0
        lo = max([s for s in stops if s[0] <= t], key=lambda s: s[0])
        hi = min([s for s in stops if s[0] >= t], key=lambda s: s[0])
        f = 0.0 if hi[0] == lo[0] else (t - lo[0]) / (hi[0] - lo[0])
        for c in range(3):
            out[c][i] = round(lo[c + 1] + (hi[c + 1] - lo[c + 1]) * f)
    return [bytes(b) for b in out]


def abs_lut(lo_c, hi_c, hot_c, band_c=80.0, grey_lo=30, grey_hi=225):
    """절대 온도 기준 LUT — hot_c ℃ 미만은 회색, 이상만 색.

    navi 는 프레임마다 lo~hi 로 정규화해 흑백 BMP 를 보낸다(thermal.hpp:246):
        g = (raw - lo) / (hi - lo) * 255
    lo·hi 는 ℃ 로 state.thermal 에 같이 실려 오므로 역변환이 된다:
        T(g) = lo_c + (hi_c - lo_c) * g / 255
    덕분에 "상대적으로 제일 뜨거운 곳"이 아니라 "진짜 60℃ 넘는 곳"만 칠할 수 있다.
    화면에 hot_c 를 넘는 게 없으면 전체가 회색으로 남는다 — 사람은 안 빨개진다.
    """
    span = (hi_c - lo_c) if hi_c > lo_c else 1.0
    hot = [(0.0, 255, 120, 0), (0.45, 255, 30, 10), (1.0, 255, 245, 150)]  # 주황→빨강→백열
    hot_luts = build_lut(hot)
    # 회색은 lo ~ min(hot, hi) 구간에 꽉 펼친다. 양쪽 실패를 다 피하려면 이 상한이어야 한다:
    #   lo~hi 로 깔면  → 불이 들어온 순간 상온부가 몇 단계에 갇혀 새카매진다
    #   lo~hot 로 깔면 → 불이 없을 때 상온 2℃ 폭이 램프의 6%만 써서 역시 새카매진다
    cool_hi = min(hot_c, hi_c)
    cool = (cool_hi - lo_c) if cool_hi > lo_c else 1.0
    out = [bytearray(256) for _ in range(3)]
    for i in range(256):
        t_c = lo_c + span * i / 255.0
        if t_c < hot_c:
            f = min(1.0, max(0.0, (t_c - lo_c) / cool))
            v = grey_lo + round((grey_hi - grey_lo) * f)
            out[0][i] = out[1][i] = out[2][i] = v
        else:
            k = min(255, round((t_c - hot_c) / band_c * 255))
            for c in range(3):
                out[c][i] = hot_luts[c][k]
    return [bytes(b) for b in out]


def thermal_stalled(th, seen, now, hold=8.0):
    """열화상이 멈췄나 — frames 가 hold 초 동안 안 늘면 정지로 본다. seen 은 호출자가 들고 있는 dict.

    navi 의 thermal_stalled 알람과 fps·valid 를 믿을 수 없어서 콘솔이 직접 센다.
    실측(2026-08-14): frames 가 완전히 고정됐는데도 fps=6.8 · valid=True 로 보고했고
    alarm/thermal_stalled 는 active:false 였다. 유일하게 정직한 값이 frames 다.
    """
    if not th.get("present"):
        return False
    fr = th.get("frames")
    if fr != seen.get("frames"):
        seen["frames"], seen["t"] = fr, now
        return False
    return now - seen.get("t", now) > hold


def selftest():
    d, s, w = fmt_state({"drive_ok": True, "wheels_alive": 4, "estop": False,
                         "wheels": [{"label": "FL", "rpm": 3.0}, {"label": "FR", "rpm": -0.0}],
                         "tof": {"dist_cm": 531, "strength": 477, "valid": True, "present": True},
                         "thermal": {"lo": 22.0, "hi": 26.5, "center": 24.4, "fps": 7.8}})
    assert "구동 OK" in d and "4축" in d and "FL+3" in d, d
    assert "5.31 m" in s and "531cm" in s and "강도 477" in s, s
    assert "22.0~26.5" in s and "7.8fps" in s, s
    assert w == "", w

    d, s, w = fmt_state({"drive_ok": False, "estop": True, "estop_reason": "과전류 6.2A",
                         "drive_error": "휠 FR 초기화 실패",
                         "tof": {"dist_cm": 999, "strength": 12, "valid": False, "present": True}})
    assert "구동 X" in d and "축 없음" in d, d
    assert "무효" in s and "999" not in s, s    # valid:false → 거리 숫자를 아예 안 띄운다
    assert "과전류 6.2A" in w and "휠 FR" in w and "신뢰 불가" in w, w

    _, s, w = fmt_state({"tof": {"present": False}})
    assert "센서없음" in s and "신뢰 불가" not in w, (s, w)   # 없는 센서로 경고를 띄우진 않는다

    assert fmt_state({})[0] == "구동 X · 0축 · 축 없음"      # 빈 state 로도 안 죽는다

    for name, stops in PALETTES.items():
        r, g, b = build_lut(stops)
        assert len(r) == len(g) == len(b) == 256, name
        assert (r[0], g[0], b[0]) == tuple(stops[0][1:]), name       # 양 끝이 제어점과 일치
        assert (r[255], g[255], b[255]) == tuple(stops[-1][1:]), name
    r, g, b = build_lut(PALETTES["grey"])
    assert all(r[i] == g[i] == b[i] == i for i in range(256))        # grey 는 항등변환
    assert build_lut(PALETTES["ironbow"])[0][128] > 0                # 중간이 검정이 아니다

    # 절대 온도 기준 — 상온 장면(29~31℃)엔 색이 하나도 없어야 한다
    r, g, b = abs_lut(29.0, 31.0, hot_c=60.0)
    assert all(r[i] == g[i] == b[i] for i in range(256)), "상온인데 색이 칠해졌다"
    # 불이 없으면 회색 램프를 꽉 써야 한다 — 안 그러면 화면이 통째로 어두워진다
    assert r[0] < 40 and r[255] > 210, ("상온 대비가 죽었다", r[0], r[255])

    # 불이 있는 장면(20~300℃): 60℃ 경계 아래는 회색, 위는 색
    r, g, b = abs_lut(20.0, 300.0, hot_c=60.0)
    cut = round((60.0 - 20.0) / (300.0 - 20.0) * 255)     # 60℃ 에 해당하는 픽셀값
    assert r[cut - 3] == g[cut - 3] == b[cut - 3], "경계 아래가 회색이 아니다"
    assert r[cut + 3] > g[cut + 3], "경계 위가 붉지 않다"
    assert r[255] > 200 and g[255] > 200, "최상단이 백열이 아니다"

    # hot_c 가 hi 보다 높으면 전부 회색 (불이 없으면 아무 색도 없다)
    r, g, b = abs_lut(20.0, 55.0, hot_c=60.0)
    assert all(r[i] == g[i] == b[i] for i in range(256))

    seen = {}
    ok = {"present": True, "frames": 100}
    assert not thermal_stalled(ok, seen, 0)                       # 첫 관측 — 판단 보류
    assert not thermal_stalled({"present": True, "frames": 101}, seen, 5)   # 늘고 있다
    assert not thermal_stalled({"present": True, "frames": 101}, seen, 10)  # 고정 5초 — 아직
    assert thermal_stalled({"present": True, "frames": 101}, seen, 20)      # 고정 15초 — 정지
    assert not thermal_stalled({"present": True, "frames": 102}, seen, 21)  # 다시 늘면 해제
    assert not thermal_stalled({"present": False}, seen, 999)     # 없는 장치는 경고 안 냄
    print("selftest OK")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hosts", default=",".join(mqtt_link.HOSTS),
                    help="브로커 후보. 쉼표 구분이고 **앞이 우선순위**다 (유선 → 무선)")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--prefix", default=PREFIX)
    ap.add_argument("--fullscreen", action="store_true")
    ap.add_argument("--rotate", default="rotate-180",
                    choices=["none", "rotate-180", "clockwise", "counterclockwise",
                             "horizontal-flip", "vertical-flip"],
                    help="영상 회전/반전 (기본 rotate-180 — 카메라가 뒤집혀 장착됨)")
    ap.add_argument("--pip-pct", type=float, default=16,
                    help="열화상 PiP 높이를 화면 높이의 %%로 (기본 16). 고정 px 로 잡으면 "
                         "화면이 작을 때 영상을 다 가린다")
    ap.add_argument("--font-pt", type=int, default=11,
                    help="하단 바 글자 크기 pt (기본 11). 바 전체 높이가 이 값에 딸려 간다")
    ap.add_argument("--hot-c", type=float, default=60.0,
                    help="이 온도(℃) 이상만 색으로 칠한다. 미만은 회색 — 사람(30℃대)은 안 빨개진다. "
                         "0 이면 절대기준을 끄고 --palette 를 그대로 쓴다")
    ap.add_argument("--hot-band", type=float, default=80.0,
                    help="hot-c 부터 몇 ℃ 폭으로 색을 펼칠지 (기본 80 → hot-c+80℃ 에서 백열)")
    ap.add_argument("--palette", default="redhot", choices=sorted(PALETTES),
                    help="열화상 가짜색 팔레트 (기본 redhot — 장면은 흑백, 뜨거운 곳만 색). "
                         "로봇은 흑백만 보내온다")
    ap.add_argument("--logo-pct", type=float, default=4.5,
                    help="좌상단 로고 높이를 화면 높이의 %%로 (기본 4.5). 0 이면 안 띄운다")
    ap.add_argument("--logo-dir", default=None,
                    help="로고 폴더 (기본: 스크립트 옆 assets/)")
    ap.add_argument("--enable-topic", default="ipc/drive_enable",
                    help="구동 허용 토글 토픽. navi 접두사 밖이라 로봇은 구독하지 않는다")
    ap.add_argument("--dump-layout", action="store_true",
                    help="4초 뒤 위젯 할당 크기를 찍고 종료 (원격에서 비율 확인용)")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()

    import gi
    gi.require_version("Gtk", "3.0")
    gi.require_version("Gst", "1.0")
    from gi.repository import Gtk, Gst, GLib, Gdk, Pango, GdkPixbuf

    Gst.init(None)

    # ---------- 영상 ----------
    # 영상도 MQTT 와 같은 경로를 타야 한다. 시작 시점엔 아직 Link 가 없으므로 직접 고른다
    # (여기서 잠깐 블로킹해도 GUI 루프 시작 전이라 무해하다).
    hosts = a.hosts.split(",")
    vhost = mqtt_link.pick(hosts, a.port) or hosts[0]

    if Gst.ElementFactory.find("gtksink") is None:
        sys.exit("gtksink 가 없다: sudo apt install gstreamer1.0-gtk3")
    # 카메라가 뒤집혀 장착돼 있어 기본 180도 회전. 장착이 바뀌면 --rotate 로 조정한다.
    flip = "" if a.rotate == "none" else f"videoflip method={a.rotate} ! "
    pipe = Gst.parse_launch(                         # sync=false → 지연 누적 대신 최신 프레임 우선
        f"tcpclientsrc name=vsrc host={vhost} port={VIDEO_PORT} ! "
        f"h264parse ! avdec_h264 ! {flip}videoconvert ! gtksink name=vsink sync=false")
    sink, vsrc = pipe.get_by_name("vsink"), pipe.get_by_name("vsrc")

    # request_video_restart 는 아래에서 정의된다(전방참조). 버스 메시지는 파이프라인이
    # PLAYING 으로 간 뒤에야 오므로 그때는 이미 정의돼 있다.
    def on_bus(_bus, msg):
        # navi 는 보는 사람이 없으면 인코딩도 안 한다 → 연결이 끊기면 조용히 다시 붙는다
        if msg.type in (Gst.MessageType.ERROR, Gst.MessageType.EOS):
            request_video_restart(RECONNECT_S)      # 워커 스레드에서 처리 (아래 주석 참고)
    bus = pipe.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_bus)

    # 🔴 영상 워치독 — ERROR/EOS 만 기다리면 안 된다.
    #    navi 가 재시작되면 로봇 쪽 소켓만 사라지고 IPC 쪽 TCP 는 ESTABLISHED 로 남는다
    #    (half-open). 데이터만 끊기는데 GStreamer 는 오류를 내지 않아 위 on_bus 가
    #    영영 안 불리고 **검은 화면 그대로 방치된다.**
    #    실측(2026-08-18): bytes_received 가 5초간 42,080,478 에서 미동도 없었고
    #    로봇은 clients=0, 콘솔 로그엔 아무 오류도 없었다. 사람이 재시작해야 풀렸다.
    #    → 프레임 도착을 직접 세서 멈추면 다시 세운다. MQTT 쪽(mqtt_link)과 같은 처방이다.
    vid = {"n": 0, "seen": -1, "busy": False, "up": True}

    def _count_frame(_pad, _info):
        vid["n"] += 1
        return Gst.PadProbeReturn.OK

    vsrc.get_static_pad("src").add_probe(Gst.PadProbeType.BUFFER, _count_frame)

    def _video_worker(delay=0.0):
        """🔴 파이프라인 상태 전환은 **반드시 이 스레드에서** 한다.

        `set_state(NULL)` 은 스트리밍 스레드가 끝날 때까지 동기적으로 기다린다.
        죽은 소켓에 묶여 있으면 그대로 블로킹되고, 메인 스레드에서 부르면
        GTK 메인루프가 멈춰 **"navi_console.py is not responding"** 이 뜬다
        (실측 2026-08-18: navi 를 끈 상태에서 워치독이 4초마다 부르다가 굳었다).
        """
        try:
            if delay:
                time.sleep(delay)
            # 포트가 안 열려 있으면 파이프라인을 아예 건드리지 않는다 — 헛된 재시작이
            # 위 블로킹을 계속 유발한다. navi 가 돌아오면 그때 붙는다.
            if mqtt_link.reachable(cli.host, VIDEO_PORT, 0.5):
                vid["up"] = True
                pipe.set_state(Gst.State.NULL)
                pipe.set_state(Gst.State.PLAYING)
            else:
                vid["up"] = False
        finally:
            vid["busy"] = False

    def request_video_restart(delay=0.0):
        """중복 요청을 막는다 — 재시작이 진행 중이면 무시한다."""
        if vid["busy"]:
            return
        vid["busy"] = True
        threading.Thread(target=_video_worker, args=(delay,), daemon=True).start()

    def video_watchdog():
        stalled = vid["n"] == vid["seen"]
        vid["seen"] = vid["n"]
        if stalled:
            request_video_restart()
        return True

    GLib.timeout_add_seconds(VIDEO_STALL_S, video_watchdog)

    # ---------- 화면 ----------
    win = Gtk.Window(title="Glovis 화재진압로봇")
    win.set_default_size(1280, 800)
    box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
    win.add(box)

    video = sink.props.widget
    video.set_hexpand(True)
    video.set_vexpand(True)

    # 열화상 PiP — IR 영상 위에 중앙하단으로 얹는다. 프레임이 안 오면 그냥 안 보인다.
    overlay = Gtk.Overlay()
    overlay.add(video)
    pip = Gtk.Image()
    pip.set_name("pip")
    pip.set_halign(Gtk.Align.CENTER)
    pip.set_valign(Gtk.Align.END)
    pip.set_margin_bottom(10)
    overlay.add_overlay(pip)

    # 협업 로고 — 좌상단 HYUNDAI GLOVIS · 우상단 navifra. 양쪽 끝에 떨어뜨려 배치한다.
    # assets/*.png 는 흰 배경을 누끼로 딴 뒤 화이트 리버스로 뽑은 것이다(패널 없이 얹는다).
    # 원본은 짙은 잉크라 그대로 투명화하면 어두운 영상 위에서 안 보인다.
    logo_dir = a.logo_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
    if a.logo_pct > 0:
        lh = max(16, round(Gdk.Screen.get_default().get_height() * a.logo_pct / 100))
        for nm, align in (("glovis", Gtk.Align.START), ("navifra", Gtk.Align.END)):
            f = os.path.join(logo_dir, nm + ".png")
            if not os.path.exists(f):
                print(f"[로고] 없음: {f}", file=sys.stderr)
                continue
            img = Gtk.Image.new_from_pixbuf(
                GdkPixbuf.Pixbuf.new_from_file_at_scale(f, -1, lh, True))
            img.set_halign(align)
            img.set_valign(Gtk.Align.START)
            img.set_margin_top(10)
            img.set_margin_start(12)
            img.set_margin_end(12)
            overlay.add_overlay(img)
    box.pack_start(overlay, True, True, 0)

    # 하단 바: 상태는 세 줄로 나눠 넉넉히 두고(말줄임 없이 줄바꿈), 버튼은 오른쪽에서
    # 바 높이를 그대로 채운다. 한 줄에 우겨넣으면 긴 e-stop 사유가 잘린다.
    bar = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
    bar.set_margin_top(6)
    bar.set_margin_bottom(6)
    bar.set_margin_start(8)
    box.pack_start(bar, False, False, 0)

    stat = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
    stat.set_hexpand(True)
    bar.pack_start(stat, True, True, 0)

    lbl_link = Gtk.Label(xalign=0)
    lbl_drive = Gtk.Label(xalign=0)
    lbl_sensor = Gtk.Label(xalign=0)
    lbl_warn = Gtk.Label(xalign=0)
    for l in (lbl_link, lbl_drive, lbl_sensor, lbl_warn):
        l.set_line_wrap(True)                        # 자르지 않고 접는다
        l.set_line_wrap_mode(Pango.WrapMode.WORD_CHAR)
        l.set_xalign(0)
        stat.pack_start(l, False, False, 0)

    btns = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
    bar.pack_end(btns, False, False, 0)

    css = Gtk.CssProvider()
    css.load_from_data(f"""
        label {{ font-size: {a.font_pt}pt; padding: 0 4px; }}
        #warn {{ color: #d00; font-weight: bold; }}
        button {{ font-size: {a.font_pt * 2}pt; font-weight: bold;
                  padding: 0 {a.font_pt}px; margin: 0; }}
        #estop {{ background-image: none; background-color: #c00; color: #fff; }}
        #drive_off {{ background-image: none; background-color: #555; color: #ddd; }}
        #drive_on  {{ background-image: none; background-color: #1a7f37; color: #fff; }}
        #pip {{ border: 2px solid rgba(255,255,255,0.7); background-color: #000; }}
    """.encode())
    scr = Gdk.Screen.get_default()
    Gtk.StyleContext.add_provider_for_screen(scr, css, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
    pip_h = max(60, round(scr.get_height() * a.pip_pct / 100))   # 화면 비례. 1024x768 → 123px
    lbl_warn.set_name("warn")

    lut = list(build_lut(PALETTES[a.palette]))   # 절대기준이 켜지면 통째로 교체된다
    lut_key = {}                    # 절대기준 LUT 재계산용 (lo,hi 가 바뀔 때만)

    def colorize(pb):
        """흑백 픽스버프에 팔레트를 입힌다. 회색이라 R 채널만 읽어 3채널로 펼친다."""
        w, h, rs = pb.get_width(), pb.get_height(), pb.get_rowstride()
        if pb.get_n_channels() != 3 or rs != w * 3:   # 예상 밖 포맷이면 원본 그대로
            return pb
        src = pb.get_pixels()
        g = src[0::3]
        out = bytearray(len(src))
        out[0::3], out[1::3], out[2::3] = (g.translate(lut[0]), g.translate(lut[1]),
                                           g.translate(lut[2]))
        return GdkPixbuf.Pixbuf.new_from_bytes(GLib.Bytes.new(bytes(out)),
                                               GdkPixbuf.Colorspace.RGB, False, 8, w, h, rs)


    def send(topic, payload="{}"):
        cli.publish(f"{a.prefix}/{topic}", payload, qos=1)

    # cmd/stop 버튼은 뺐다(사용자 요청). 평상시 감속 정지는 joy_teleop 이 조이스틱을 놓을 때
    # 알아서 보낸다 — 화면에서는 비상 차단과 그 해제만 다룬다.
    # 구동 허용 토글 — 기본 잠김. 이 상태를 소유하는 건 콘솔이고 joy_teleop 이 따라간다.
    # 로봇에는 모터 enable 토픽이 없다(navi 구독 8종에 없음) → IPC 측 인터록이다.
    tgl = Gtk.ToggleButton(label="구동 잠김")
    tgl.set_name("drive_off")
    tgl.set_vexpand(True)

    def on_toggle(b):
        on = b.get_active()
        b.set_label("구동 허용" if on else "구동 잠김")
        b.set_name("drive_on" if on else "drive_off")
        cli.publish(a.enable_topic, json.dumps({"on": on}), qos=1, retain=True)
        if not on:
            send("cmd/stop")                     # 잠그는 즉시 감속 정지
    tgl.connect("toggled", on_toggle)
    btns.pack_start(tgl, True, True, 0)

    for label, name, topic, payload in (
            ("■ E-STOP", "estop", "cmd/estop", '{"reason":"콘솔 버튼"}'),
            ("E-STOP 해제", None, "cmd/reset", "{}")):
        b = Gtk.Button(label=label)
        b.set_vexpand(True)                          # 바 높이를 그대로 채운다
        if name:
            b.set_name(name)
        b.connect("clicked", lambda _w, t=topic, p=payload: send(t, p))
        btns.pack_start(b, True, True, 0)

    # ---------- MQTT ----------
    alarms = {}
    th_seen = {}                  # 열화상 frames 감시용 (thermal_stalled 참조)
    notice = {"txt": ""}          # 최근 event / 접속 상태. state 줄 끝에 얹는다

    def on_connect(c, _u, _f, rc):
        c.subscribe([(f"{a.prefix}/state", 0), (f"{a.prefix}/event", 1),
                     (f"{a.prefix}/alarm/#", 1), (f"{a.prefix}/state/online", 1),
                     (f"{a.prefix}/frame/thermal", 0)])
        # 프레임 발행은 기본 꺼져 있다 — 켜야 frame/thermal 이 온다
        c.publish(f"{a.prefix}/cmd/stream", json.dumps({"on": True, "fps": 10}), qos=1)
        # 접속할 때마다 무조건 잠금부터 발행한다 — 기본값이 안전이어야 한다
        c.publish(a.enable_topic, '{"on":false}', qos=1, retain=True)
        GLib.idle_add(tgl.set_active, False)
        notice["txt"] = ""
        GLib.idle_add(lbl_drive.set_text, f"MQTT 연결 (rc={rc}) — 상태 수신 대기")

    def on_disconnect(_c, _u, rc):
        notice["txt"] = f"⚠ MQTT 끊김(rc={rc}) 재접속 중"
        GLib.idle_add(lbl_warn.set_text, notice["txt"])

    def on_message(_c, _u, msg):
        sub = msg.topic[len(a.prefix) + 1:]
        if sub == "frame/thermal":                   # BMP 바이너리 — json 파싱 전에 걸러야 한다
            try:
                ld = GdkPixbuf.PixbufLoader.new_with_type("bmp")
                ld.write(msg.payload)
                ld.close()
                pb = ld.get_pixbuf()
                pb = colorize(pb)            # 흑백 → 가짜색. 확대 전에 입혀야 rowstride 가 단순하다
                h = pip_h
                pb = pb.scale_simple(round(h * pb.get_width() / pb.get_height()), h,
                                     GdkPixbuf.InterpType.BILINEAR)
            except GLib.Error:
                return
            GLib.idle_add(pip.set_from_pixbuf, pb)
            return
        try:
            d = json.loads(msg.payload)
        except ValueError:
            return
        if sub == "state":
            drive, sensor, warn = fmt_state(d)
            act = " · ".join(f"{k}:{v}" for k, v in sorted(alarms.items()))
            GLib.idle_add(lbl_drive.set_text,
                          drive + (f"   │   알람 {act}" if act else "   │   알람 없음"))
            GLib.idle_add(lbl_sensor.set_text,
                          sensor + (f"   │   {notice['txt']}" if notice["txt"] else ""))
            th = d.get("thermal") or {}
            if a.hot_c > 0 and th.get("valid"):
                # lo·hi 가 바뀔 때만 LUT 을 다시 만든다. 프레임마다 하면 낭비다.
                key = (round(th.get("lo", 0), 1), round(th.get("hi", 0), 1))
                if key != lut_key.get("k") and key[1] > key[0]:
                    lut_key["k"] = key
                    lut[:] = abs_lut(key[0], key[1], a.hot_c, a.hot_band)
            if thermal_stalled(d.get("thermal") or {}, th_seen, time.monotonic()):
                warn = (warn + " / " if warn else "") + \
                    "열화상 정지 — frames 고정(로봇에서 systemctl restart navi 필요)"
            GLib.idle_add(lbl_warn.set_text, warn)
        elif sub.startswith("alarm/"):
            key = d.get("key", sub[6:])
            if d.get("active"):
                alarms[key] = d.get("severity", "?")
            else:
                alarms.pop(key, None)
        elif sub == "state/online":
            if d.get("online"):
                # navi 가 재시작하면 프레임 발행이 기본값(꺼짐)으로 돌아간다. 브로커는 그대로라
                # on_connect 가 다시 안 불리므로 여기서 켜줘야 열화상 PiP 가 살아난다.
                _c.publish(f"{a.prefix}/cmd/stream", json.dumps({"on": True, "fps": 10}), qos=1)
            else:
                GLib.idle_add(lbl_warn.set_text, "⚠ 로봇 오프라인 (navi 정지 또는 통신 두절)")
        elif sub == "event":
            notice["txt"] = f"event {d.get('event')} {d.get('detail', '')}".strip()[:100]

    def on_switch(old, new):
        # 경로가 바뀌면 영상도 따라가야 한다 — 기존 재접속 경로를 그대로 쓴다
        vsrc.set_property("host", new)
        request_video_restart()          # 메인 스레드에서 set_state 를 부르면 GUI 가 굳는다

    # 유선 우선 · 무선 폴백 (mqtt_link 참고). 콘솔이 죽으면 브로커가 대신 잠금을 발행한다
    # — 조종 화면 없이 구동되는 상태를 막는다.
    cli = mqtt_link.Link(hosts=hosts, port=a.port,
                         on_connect=on_connect, on_message=on_message,
                         on_disconnect=on_disconnect, on_switch=on_switch,
                         will=(a.enable_topic, '{"on":false}', 1, True),
                         log=lambda m: print(m, flush=True))

    def link_tick():
        cli.tick()
        if not cli.connected:
            lbl_link.set_text("⚠ 링크 없음 — 재접속 중")
        else:
            if cli.host == hosts[0]:
                txt = f"🔗 유선 {cli.host}"
            else:
                w = mqtt_link.wifi_signal()
                sig = f"  {w[2]:.0f} dBm ({w[1]}%)" if w else ""
                txt = f"📶 무선 {cli.host}{sig}"
            lbl_link.set_text(txt if vid["up"] else txt + "   ⚠ 영상 없음")
        return True
    link_tick()
    GLib.timeout_add(1500, link_tick)

    # ---------- 조작 ----------
    def on_key(_w, ev):
        if ev.keyval == Gdk.KEY_F11:
            (win.unfullscreen if win.get_window().get_state() & Gdk.WindowState.FULLSCREEN
             else win.fullscreen)()
        elif ev.keyval == Gdk.KEY_Escape:
            win.unfullscreen()
    win.connect("key-press-event", on_key)
    win.connect("destroy", Gtk.main_quit)

    win.show_all()
    if a.fullscreen:
        win.fullscreen()

    if a.dump_layout:                                # 원격에서 화면을 못 볼 때 비율을 숫자로 확인
        def dump():
            for nm, w in (("window", win), ("video", overlay), ("bar", bar),
                          ("stat", stat), ("btns", btns), ("drive", lbl_drive),
                          ("sensor", lbl_sensor), ("warn", lbl_warn)):
                r = w.get_allocation()
                print(f"  {nm:8s} {r.width:5d} x {r.height:4d}  @({r.x},{r.y})")
            print(f"  bar/window 높이비 = "
                  f"{bar.get_allocation().height / max(1, win.get_allocation().height):.1%}")
            Gtk.main_quit()
            return False
        GLib.timeout_add_seconds(4, dump)
    pipe.set_state(Gst.State.PLAYING)
    try:
        Gtk.main()
    finally:
        cli.publish(f"{a.prefix}/cmd/stream", '{"on":false}', qos=1)  # 프레임 발행 끄고 나간다
        time.sleep(0.2)
        pipe.set_state(Gst.State.NULL)
        cli.loop_stop()


if __name__ == "__main__":
    main()
