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

# ── 차폭 예측선 (1920x1080 프레임 좌표) ───────────────────────────────
# 2026-08-18 사용자가 바닥 실측해서 그은 선을 픽셀에서 추출한 값이다.
# 🔴 카메라가 **우측에 치우쳐 장착**돼 있어 좌우가 비대칭인 게 정상이다.
#    보기 좋게 대칭으로 "정리"하면 실제 폭과 틀어진다. 우측 선이 더 급한 것도
#    카메라에 더 가깝기 때문이고, 두 선이 하단이 아니라 좌·우 측면으로 빠져나가는 것도
#    로봇 폭이 근거리 화각보다 넓다는 뜻이다.
GUIDE_WIDTH = (((0, 579), (661, 509)),        # 좌
               ((1035, 506), (1919, 792)))    # 우
# 거리 눈금 — **수평이다.** y 하나만 두고 좌·우 x 는 차폭선에서 계산한다.
#   좌표 2개를 따로 박아두면 실측 오차 때문에 좌우 높이가 어긋난다(실제로 512 vs 521 로
#   어긋나 지적받았다). 여기 이렇게 두면 차폭선을 나중에 고쳐도 눈금은 항상 수평이다.
GUIDE_DIST  = ((517, "1.0 m"),)      # (프레임 y, 라벨)
TICK_HALF   = 45                     # 눈금 반길이 (프레임 px)
LABEL_RATIO = 0.015                  # 거리 라벨 크기 = 영상 높이 × 이 값 (2026-08-18: 1/2 로 축소)
COL_WIDTH   = (1.00, 0.15, 0.15)     # 차폭선 = 빨강
COL_DIST    = (0.40, 0.75, 0.05)     # 거리 눈금 = 진한 연두 (2026-08-18 사용자 지정, 하늘색에서 변경)
FRAME_W, FRAME_H = 1920, 1080


def guide_tick_xs(y):
    """거리선 높이 y 에서 좌·우 차폭선의 x. 수평이므로 y 하나로 양쪽이 정해진다."""
    return [x0 + (y - y0) * (x1 - x0) / (y1 - y0)
            for (x0, y0), (x1, y1) in GUIDE_WIDTH]


# ── 차량 카운터 ───────────────────────────────────────────────────────────
# 이 로봇은 **차량 아래로 기어들어간다.** 차체가 ToF 시야를 막으면 차 밑, 트이면 차 사이다.
# (센서 장착 방향은 확인하지 않았다 — 임계값만 사용자가 지정했다. 방향이 다르면
#  `--under-cm` 만 바꾸면 되고 판정 구조는 그대로다.)
# 판정 기준은 사용자 지정(2026-09-02):
#
#   0.2 m 이하가 3초 유지   → +1대
#   그 이상이 1초 이상 유지 → 차 사이 통과 중 = 다음 대를 셀 준비
#
# 🔴 `under` 은 **valid 를 반드시 본다.** `valid` 는 신호강도 판정이고 false 면 거리값이
#    쓰레기라 작은 값이 튀어나올 수 있다(fmt_state 주석 참고) — 그걸 "차 밑" 으로 세면
#    허깨비 차량이 생긴다. 그리고 물리적으로도 **무효 = 위에 반사할 것이 없음 = 차 사이**다.
#    즉 판정은 하나로 정리된다: `valid 하고 임계 이하일 때만 차 밑`, 나머지는 전부 차 사이.
UNDER_CM, UNDER_HOLD_S, GAP_HOLD_S = 20.0, 3.0, 1.0   # 0.2 m (2026-09-02 사용자 변경)
ICONS = "🚗 🚗"       # 첫 줄 고정. 대수·상태와 무관하게 그대로 둔다(사용자 지정)


class VehicleCounter:
    """지나간 차량 수. 히스테리시스(진입 3초 / 이탈 1초)로 튐과 이중계수를 막는다.

    이탈에 1초를 요구하는 이유: 차 밑에서 거리값이 순간 튀어도(배선·요철) 차에서
    나온 것으로 보지 않는다. 그래서 한 대를 두 번 세지 않는다.
    """

    def __init__(self, under_cm=UNDER_CM, under_hold=UNDER_HOLD_S, gap_hold=GAP_HOLD_S):
        self.under_cm, self.under_hold, self.gap_hold = under_cm, under_hold, gap_hold
        self.n, self.under, self.raw, self.since = 0, False, None, 0.0

    def reset(self):
        """카운터만 0 으로. 현재 차 밑인지 여부는 유지한다 —
        리셋했다고 지금 밑에 있는 차를 다시 세면 안 된다."""
        self.n = 0

    def feed(self, tof, now):
        """state.tof 한 샘플. 반환: 이번 호출로 +1 됐나."""
        cm = tof.get("dist_cm")
        raw = bool(tof.get("present") and tof.get("valid")
                   and cm is not None and cm <= self.under_cm)
        if raw != self.raw:                       # 원시 판정이 바뀌면 타이머를 다시 잡는다
            self.raw, self.since = raw, now
        held = now - self.since
        if raw and not self.under and held >= self.under_hold:
            self.under, self.n = True, self.n + 1
            return True
        if not raw and self.under and held >= self.gap_hold:
            self.under = False
        return False

    def text(self):
        """세 줄로 준다 — 아이콘 / 대수 / 상태.

        차 밑에서 힐끗 보는 값이라 셋을 겹쳐 읽지 않게 나눴다.
        첫 줄은 **고정**이다(대수만큼 늘리지 않는다 — 사용자 지정). 그래서 줄 수와
        폭이 항상 같고, 하단 바 높이가 들썩여 영상 영역이 흔들리는 일이 없다.
        """
        return (f"{ICONS}\n{self.n}대 통과\n"
                f"{'(차 밑)' if self.under else '(차 사이)'}")


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
    # ── 차량 카운터 ──
    def tof(cm, valid=True, present=True):
        return {"dist_cm": cm, "valid": valid, "present": present}

    c = VehicleCounter()
    assert c.feed(tof(80), 0.0) is False and c.n == 0          # 차 사이
    assert c.feed(tof(10), 1.0) is False and c.n == 0          # 진입 — 아직 3초 안 됐다
    assert c.feed(tof(10), 3.9) is False and c.n == 0
    assert c.feed(tof(10), 4.0) is True and c.n == 1           # 3초 유지 → +1
    assert c.feed(tof(10), 9.0) is False and c.n == 1, "유지 중에 또 세면 안 된다"
    # 차 밑에서 값이 순간 튀어도 1초를 못 넘기면 이탈이 아니다 → 이중계수 없음
    assert c.feed(tof(80), 9.5) is False and c.under is True
    assert c.feed(tof(10), 10.0) is False and c.n == 1
    assert c.feed(tof(10), 20.0) is False and c.n == 1, "재진입으로 세면 안 된다"
    # 1초 이상 벗어나면 차 사이 → 다음 대를 셀 준비
    assert c.feed(tof(80), 21.0) is False and c.under is True  # 아직 1초 안 됨
    assert c.feed(tof(80), 22.1) is False and c.under is False
    assert c.feed(tof(10), 23.0) is False and c.n == 1
    assert c.feed(tof(10), 26.0) is True and c.n == 2, "두 번째 차"

    # 🔴 무효값은 절대 차 밑으로 세지 않는다 — false 면 작은 값이 튀어나올 수 있다
    c2 = VehicleCounter()
    for t in (0.0, 1.0, 2.0, 3.0, 4.0, 5.0):
        assert c2.feed(tof(5, valid=False), t) is False
    assert c2.n == 0, "무효 거리값으로 허깨비 차량이 생겼다"
    for t in (0.0, 1.0, 2.0, 3.0, 4.0, 5.0):
        assert c2.feed(tof(5, present=False), t) is False
    assert c2.n == 0, "센서 없음도 차 밑이 아니다"
    assert c2.feed(tof(None), 6.0) is False

    # 경계값: 임계값 자체는 "이하" 라서 포함이다
    c3 = VehicleCounter()
    c3.feed(tof(UNDER_CM), 0.0)
    assert c3.feed(tof(UNDER_CM), 3.0) is True, "임계값은 이하 = 차 밑"
    c4 = VehicleCounter()
    c4.feed(tof(UNDER_CM + 1), 0.0)
    assert c4.feed(tof(UNDER_CM + 1), 5.0) is False and c4.n == 0
    assert UNDER_CM == 20.0, "사용자 지정 0.2 m"
    # 세 줄이어야 한다(아이콘 / 대수 / 상태). 줄 수가 흔들리면 하단 바 높이가 들썩인다
    for cc in (c3, c4, VehicleCounter()):
        assert cc.text().count("\n") == 2, cc.text()
    assert "차 밑" in c3.text() and "차 사이" in c4.text()
    # 첫 줄은 **항상 고정** — 대수·상태와 무관해야 한다(폭이 변하면 바가 흔들린다)
    c5 = VehicleCounter()
    for n, under in ((0, False), (3, False), (3, True), (99, True)):
        c5.n, c5.under = n, under
        lines = c5.text().split("\n")
        assert lines[0] == ICONS, (n, under, lines[0])
        assert lines[1] == f"{n}대 통과"
        assert lines[2] == ("(차 밑)" if under else "(차 사이)")

    # 리셋은 카운터만 — 지금 차 밑인 사실은 유지한다(리셋 후 재계수 금지)
    c.reset()
    assert c.n == 0 and c.under is True
    assert c.feed(tof(10), 30.0) is False and c.n == 0

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
    ap.add_argument("--lift-block-topic", default="ipc/lift_blocked",
                    help="crevis-io 가 발행하는 리프트 막힘 알람 (retain)")
    ap.add_argument("--under-cm", type=float, default=UNDER_CM,
                    help="이 거리 이하 = 차 밑 (기본 16cm = 0.16m)")
    ap.add_argument("--under-hold", type=float, default=UNDER_HOLD_S,
                    help="차 밑 판정 유지시간 s → 이만큼 지나면 +1대")
    ap.add_argument("--gap-hold", type=float, default=GAP_HOLD_S,
                    help="차 사이 판정 유지시간 s → 다음 대를 셀 준비")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()

    import cairo                    # 라벨 굵기 지정(FONT_WEIGHT_BOLD)에 필요
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
            # 로그는 **전환 시점만** 남긴다 — 4초마다 찍으면 저널이 쓰레기가 되고,
            # 아무것도 안 찍으면 원격에서 상태를 확인할 방법이 없다(실제로 겪었다).
            # 🔴 영상 host 는 **항상 현재 MQTT 링크에 맞춘다.**
            #    시작 시 pick() 이 None 이면 vhost 가 hosts[0](유선)로 떨어지는데,
            #    Link 의 첫 접속은 on_switch 를 부르지 않아(이전 host 없음) 영상만
            #    유선에 남는다. 유선이 빠져 있으면 라우트가 없어 SYN 도 못 나가고
            #    2초마다 재시작만 반복한다 — 실측 2026-08-18, 이걸로 카메라가 죽었다.
            host = cli.host or hosts[0]
            if mqtt_link.reachable(host, VIDEO_PORT, 0.5):
                print(f"[video] {host}:{VIDEO_PORT} 재접속 — 파이프라인 재시작", flush=True)
                vid["up"] = True
                pipe.set_state(Gst.State.NULL)
                vsrc.set_property("host", host)     # NULL 상태에서 바꿔야 확실히 먹는다
                pipe.set_state(Gst.State.PLAYING)
            else:
                if vid["up"]:
                    print(f"[video] {host}:{VIDEO_PORT} 닫힘 — 파이프라인 대기"
                          " (navi 정지?)", flush=True)
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

    # 차폭 예측선 — 영상 위에 cairo 로 얹는다. gtksink 는 종횡비를 유지해 레터박스를
    # 넣으므로 **영상이 실제로 그려진 사각형**을 구해 그 안에서 정규화 좌표를 쓴다
    # (창 크기가 바뀌어도 선이 영상에 붙어 있어야 한다).
    guide = {"on": True}

    def on_draw_guide(area, cr):
        if not guide.get("logged"):
            guide["logged"] = True
            print(f"[guide] draw 호출됨 alloc={area.get_allocated_width()}x"
                  f"{area.get_allocated_height()} on={guide['on']}", flush=True)
        if not guide["on"]:
            return False
        aw, ah = area.get_allocated_width(), area.get_allocated_height()
        if ah <= 0 or aw <= 0:
            return False
        ar = FRAME_W / FRAME_H
        if aw / ah > ar:                       # 위젯이 더 넓다 → 좌우 레터박스
            vw, vh, ox, oy = ah * ar, ah, (aw - ah * ar) / 2, 0
        else:                                  # 위젯이 더 높다 → 상하 레터박스
            vw, vh, ox, oy = aw, aw / ar, 0, (ah - aw / ar) / 2

        def P(x, y):
            return ox + x / FRAME_W * vw, oy + y / FRAME_H * vh

        cr.set_line_width(max(2.0, vh * 0.006))
        cr.set_source_rgba(*COL_WIDTH, 0.92)               # 차폭선
        for (x0, y0), (x1, y1) in GUIDE_WIDTH:
            cr.move_to(*P(x0, y0))
            cr.line_to(*P(x1, y1))
        cr.stroke()

        cr.set_source_rgba(*COL_DIST, 0.95)                # 거리 눈금(수평)
        for ty, _ in GUIDE_DIST:
            for tx in guide_tick_xs(ty):
                cr.move_to(*P(tx - TICK_HALF, ty))
                cr.line_to(*P(tx + TICK_HALF, ty))
        cr.stroke()

        # PIL 미리보기는 DejaVuSans-Bold 였는데 cairo 는 굵기를 안 줘서 화면만 얇았다
        cr.select_font_face("Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
        cr.set_font_size(max(5.5, vh * LABEL_RATIO))
        for ty, lab in GUIDE_DIST:
            cr.move_to(*P(guide_tick_xs(ty)[-1] + TICK_HALF + 12, ty + 12))
            cr.show_text(lab)
        return False

    guide_area = Gtk.DrawingArea()
    guide_area.connect("draw", on_draw_guide)
    overlay.add_overlay(guide_area)
    overlay.set_overlay_pass_through(guide_area, True)     # 터치가 통과해야 한다
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
    # 차량 카운터는 차 밑에서 눈으로 확인하는 값이라 크게 띄운다.
    # pack_end 는 **나중에 부른 것이 왼쪽**이다 → btns 를 먼저 팩해야 e-stop·리셋
    # 버튼이 오른쪽 끝을 유지한다. 터치 조작에서 안전 버튼 위치가 바뀌면 안 된다.
    lbl_count = Gtk.Label(xalign=0)
    lbl_count.set_name("count")
    for l in (lbl_link, lbl_drive, lbl_sensor, lbl_warn):
        l.set_line_wrap(True)                        # 자르지 않고 접는다
        l.set_line_wrap_mode(Pango.WrapMode.WORD_CHAR)
        l.set_xalign(0)
        stat.pack_start(l, False, False, 0)

    btns = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
    bar.pack_end(btns, False, False, 0)
    bar.pack_end(lbl_count, False, False, 0)      # btns 다음 = 버튼 왼쪽에 놓인다

    css = Gtk.CssProvider()
    css.load_from_data(f"""
        label {{ font-size: {a.font_pt}pt; padding: 0 4px; }}
        #warn {{ color: #d00; font-weight: bold; }}
        /* 세 줄이라 2배로 두면 하단 바가 커져 영상이 줄어든다 → 1.5배 */
        #count {{ font-size: {round(a.font_pt * 1.5)}pt; font-weight: bold; color: #ffd400;
                  padding: 0 14px; }}
        #count_under {{ font-size: {round(a.font_pt * 1.5)}pt; font-weight: bold;
                        color: #2ecc40; padding: 0 14px; }}
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

    lift_block = {"txt": ""}        # crevis-io 가 알려주는 리프트 막힘 사유
    warn_state = {"txt": ""}        # 로봇 state 에서 온 경고
    counter = VehicleCounter(a.under_cm, a.under_hold, a.gap_hold)
    prev_estop = {"v": None}        # e-stop 해제(True→False) 를 잡기 위한 직전값

    def show_count():
        """차 밑이면 초록(#count_under), 차 사이면 노랑(#count). 이름을 바꿔 끼운다."""
        lbl_count.set_text(counter.text())
        lbl_count.set_name("count_under" if counter.under else "count")

    show_count()

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


    def warn_text(base=None):
        """경고줄 조립. 로봇 state 경고와 IPC 리프트 막힘을 합친다.

        따로 set_text 하면 나중에 온 쪽이 앞의 것을 지운다 — 실제로 그렇게 짰다가
        리프트 알람이 state 한 줄에 덮여 사라졌다.
        """
        if base is not None:
            warn_state["txt"] = base
        parts = [x for x in (lift_block["txt"], warn_state["txt"]) if x]
        return " / ".join(parts)

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

    def on_reset_clicked(_w):
        """E-STOP 해제 버튼. 차량 카운터도 여기서 0 으로 만든다.

        🔴 state 의 estop True→False 전이만 보면 **e-stop 이 안 걸려 있을 때는
           카운터를 리셋할 방법이 없다**(전이가 없으니까). 사용자 의도는 "리셋 조작으로
           0 을 만든다" 이므로 버튼 자체에도 건다. 두 경로 다 조종자의 명시적 조작이다.
        """
        send("cmd/reset")
        counter.reset()
        GLib.idle_add(show_count)
        print("[차량] 리셋 버튼 — 카운터 0 으로", flush=True)

    for label, name, topic, payload in (
            ("■ E-STOP", "estop", "cmd/estop", '{"reason":"콘솔 버튼"}'),
            ("E-STOP 해제", None, None, None)):
        b = Gtk.Button(label=label)
        b.set_vexpand(True)                          # 바 높이를 그대로 채운다
        if name:
            b.set_name(name)
        if topic is None:
            b.connect("clicked", on_reset_clicked)
        else:
            b.connect("clicked", lambda _w, t=topic, p=payload: send(t, p))
        btns.pack_start(b, True, True, 0)

    # ---------- MQTT ----------
    alarms = {}
    th_seen = {}                  # 열화상 frames 감시용 (thermal_stalled 참조)
    notice = {"txt": ""}          # 최근 event / 접속 상태. state 줄 끝에 얹는다

    def on_connect(c, _u, _f, rc):
        c.subscribe([(f"{a.prefix}/state", 0), (f"{a.prefix}/event", 1),
                     (f"{a.prefix}/alarm/#", 1), (f"{a.prefix}/state/online", 1),
                     (f"{a.prefix}/frame/thermal", 0), (a.lift_block_topic, 1)])
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
        if msg.topic == a.lift_block_topic:      # prefix 밖 토픽 → 먼저 처리한다
            try:
                d = json.loads(msg.payload)
            except ValueError:
                return
            lift_block["txt"] = (f"⬆ 리프트 {d.get('reason', '?')}"
                                 f" — 버튼을 뗐다 다시 누르면 재시도")\
                if d.get("on") else ""
            GLib.idle_add(lbl_warn.set_text, warn_text())
            return
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
            # ── 차량 카운터. 리셋은 **e-stop 해제**다(사용자 지정) — 즉 조종자가
            #    의도적으로 RESET 을 눌렀을 때만 0 이 된다. 접속·재연결로는 안 바뀐다.
            es = bool(d.get("estop"))
            if prev_estop["v"] is True and es is False:
                counter.reset()
                print("[차량] e-stop 해제 — 카운터 0 으로", flush=True)
            prev_estop["v"] = es
            if counter.feed(d.get("tof") or {}, time.monotonic()):
                print(f"[차량] {counter.n}대 통과", flush=True)
            GLib.idle_add(show_count)

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
            GLib.idle_add(lbl_warn.set_text, warn_text(warn))
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
        # 경로가 바뀌면 영상도 따라간다. host 는 워커가 cli.host 로 맞추므로 여기서 안 만진다
        # (메인 스레드에서 set_state 를 부르면 GUI 가 굳는다 — 4.12 참고).
        request_video_restart()

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
