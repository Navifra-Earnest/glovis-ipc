#!/usr/bin/env python3
"""상용 아날로그 게임패드 → 메카넘 → navi/cmd/wheel. 기존 joy-teleop 과 **동시 운용**.

기존 조이스틱(디지털 4방향, js1)은 그대로 둔다. 두 프로세스가 같은 브로커에
발행하지만 서로 안 싸운다 — 양쪽 다 **움직일 때만 발행하고, 놓으면 `cmd/stop`
한 번 보낸 뒤 침묵**하는 계약을 지키기 때문이다(joy_teleop 의 발행 분기와 동일).
둘을 **동시에** 꺾으면 마지막에 도착한 명령이 이긴다 — 그건 어쩔 수 없다.

  조작 (사용자가 정한 규칙, 2026-09-02)
    LB + RB 홀드   데드맨. **둘 다** 눌러야 움직이고, 떼면 즉시 정지
    왼쪽 스틱 ↑↓   전진 / 후진 (기울인 만큼 비례)
    X 홀드 + ←→    좌 / 우 횡이동. 이 동안 **회전은 죽는다**
    오른쪽 스틱 ←→ 좌 / 우회전 (비례)
    D패드 ↑↓       주행 최대속도 ± (--vmax-step)
    D패드 →←       회전 최대속도 ± (--wmax-step-deg)

🔴 **콘솔 구동허용이 OFF 일 때만 동작한다** — 기존 조이스틱은 ON 에서만 움직이므로
   토글이 조종기 선택 스위치가 되고 이중 명령이 불가능해진다 (allowed() 주석 참고).

⚠️ 이 패드는 커널이 js 노드를 안 만든다(hid_nintendo) → **evdev 직독**이다.
   버튼·축 번호는 실측했다(2026-09-02, joy2_map.py): 아래 상수 주석 참고.

  python3 joy2_teleop.py
  python3 joy2_teleop.py --prefix navitest   # 로봇 안 움직이는 발행 확인용
  python3 joy2_teleop.py --selftest          # 판정·기구학 검증 (하드웨어 불필요)
"""
import argparse
import glob
import json
import math
import os
import struct
import sys
import time

import mqtt_link
# 🔴 기구학·상한·발행 판정은 **기존 파일에서 가져온다.** 부호표(MIX)가 두 곳에 있으면
#    한쪽만 고쳐서 두 조이스틱이 다르게 도는 사고가 난다.
from joy_teleop import MAX_RPM, RAMP, RPM_PER_MS, due, mecanum_rpm

# struct input_event: timeval(long sec, long usec) + u16 type + u16 code + s32 value
EV_FMT, EV_SIZE = "llHHi", 24
EV_KEY, EV_ABS = 0x01, 0x03

# ── 실측 매핑 (2026-09-02, joy2_map.py) ───────────────────────────────────
BTN_DEADMAN = (310, 311)   # BTN_TL / BTN_TR = LB / RB (위쪽 어깨). 아래쪽 ZL·ZR 은 312/313
BTN_STRAFE = 307           # BTN_NORTH = 물리 X 버튼
AX_FWD, AX_STRAFE, AX_YAW = 1, 0, 3   # ABS_Y, ABS_X(좌스틱), ABS_RX(우스틱 좌우)
AX_MAX = 32767.0           # 위·왼쪽이 **음수**다 (실측)
HAT_X, HAT_Y = 16, 17      # D패드. -1/0/+1 로 온다 (아날로그 아님)

# ── 장착 보정: 로봇의 물리적 전진은 조종자 기준 **우측**이다 ──────────────
# 기존 조이스틱은 패널을 -90° 돌려 꽂아서 **기구적으로** 보정했다. 이 패드는 손에
# 드는 물건이라 그럴 수 없으니 소프트웨어에서 축을 바꾼다:
#
#     사용자 "전진"(스틱 ↑)  → 차체 vy   (게걸음 축)
#     사용자 "우횡이동"       → 차체 vx   (전후진 축)
#
# 각속도는 그대로다 — 좌표계를 90° 돌려도 yaw 의 부호는 바뀌지 않는다(사용자 질문 확인).
#
# 🔴 아래 세 부호는 **실기에서 확정한다.** 반대로 가면 해당 줄 하나만 뒤집는다.
#    (한 번에 하나씩 확인할 것 — 두 개를 같이 뒤집으면 어느 쪽이 문제였는지 못 가린다)
SIGN_FWD = +1.0      # 사용자 전진 → vy 부호
SIGN_STRAFE = +1.0   # 사용자 우횡이동 → vx 부호
SIGN_YAW = +1.0      # 우스틱 오른쪽 → wz 부호 (+wz = CCW = 좌회전)

# 회전 지령을 °/s 로 받기 위한 환산 팔길이. mecanum_rpm 의 wz 는 vx·vy 와 같은
# 단위(선속도)로 들어가므로 (lx+ly)/2 가 필요하다. 차체 치수 미실측(kinematics:false)이라
# 추정값이다 → **제자리 360° 회전 시간을 재서 보정한다.** 방향에는 영향 없다.
ROT_ARM_M = 0.30

POLL = 0.02          # 아날로그라 기존(0.05)보다 촘촘히 본다. 발행 주기와는 무관
DEAD = 0.12          # 정규화 데드존. 스틱 중립 드리프트가 주행으로 새는 걸 막는다
VMAX_HW = MAX_RPM / RPM_PER_MS      # 축 상한(20 RPM)이 정하는 물리 최고속 ≈ 0.159 m/s


def allowed(enabled):
    """서브 패드는 콘솔 구동허용이 **꺼져 있을 때만** 움직인다 (사용자 결정 2026-09-02).

    기존 조이스틱은 구동허용 **ON** 에서만 움직인다 → 반대로 걸면 두 조종기가
    **동시에 명령할 수 없다.** 이중 명령이 구조적으로 불가능해진다. 즉 콘솔의
    구동허용 토글이 사실상 **조종기 선택 스위치**다:

        허용 ON   → 기존 조이스틱 (콘솔이 안전 책임)
        허용 OFF  → 이 서브 패드  (데드맨 LB+RB 가 안전 책임)

    🔴 None(아직 못 받음)은 **막는다.** 기본값을 False 로 두면 시작 직후 상태를
       모르는 채로 열려서, 마침 허용 ON 이었을 때 이중 명령이 난다. retain 된
       `ipc/drive_enable` 이 접속 즉시 오므로 실사용에서 기다림은 없다.
    """
    return enabled is False


def axis_norm(raw, dead=DEAD):
    """원시 축값 → -1.0~+1.0. 데드존 **바깥부터 0 에서 다시 시작**한다.

    단순히 데드존 안을 0 으로 만들면 데드존을 넘는 순간 속도가 튄다.
    """
    v = max(-1.0, min(1.0, raw / AX_MAX))
    if abs(v) < dead:
        return 0.0
    return math.copysign((abs(v) - dead) / (1.0 - dead), v)


def to_body(fwd, strafe, yaw):
    """사용자 의도(전진, 우횡이동, 좌회전) → 차체 (vx, vy, wz).

    **여기가 -90° 장착 보정의 전부다.** fwd 가 vy 로, strafe 가 vx 로 간다.
    """
    return SIGN_STRAFE * strafe, SIGN_FWD * fwd, SIGN_YAW * yaw


def resolve(btn, ax, vmax, wz_max):
    """입력 상태 → (vx, vy, wz). 데드맨이 안 잡혀 있으면 전부 0 이다.

    X 홀드 중에는 횡이동이 살고 **회전이 죽는다**(사용자 규칙). 반대로 X 를 놓으면
    좌우 스틱은 아무 것도 아니다 — 횡이동은 X 모드에서만 나온다.
    """
    if not all(b in btn for b in BTN_DEADMAN):
        return 0.0, 0.0, 0.0
    fwd = -axis_norm(ax.get(AX_FWD, 0)) * vmax          # 위가 음수 → 뒤집어 전진 +
    if BTN_STRAFE in btn:
        return to_body(fwd, axis_norm(ax.get(AX_STRAFE, 0)) * vmax, 0.0)
    return to_body(fwd, 0.0, -axis_norm(ax.get(AX_YAW, 0)) * wz_max)


def step_limits(hat_edges, vmax, wdeg, a):
    """D패드 엣지 → (주행 최대속도, 회전 최대속도°/s). 눌린 순간에만 한 칸 움직인다."""
    for code, val in hat_edges:
        if code == HAT_Y:                     # 위=-1 → 증속
            vmax += -val * a.vmax_step
        elif code == HAT_X:                   # 오른쪽=+1 → 회전 증속
            wdeg += val * a.wmax_step_deg
    return (min(a.vmax_hi, max(a.vmax_lo, round(vmax, 4))),
            min(a.wmax_hi, max(a.wmax_lo, round(wdeg, 2))))


class Pad:
    """evdev 논블로킹 리더. 버튼 집합 · 축 값 · D패드 상승엣지만 들고 있는다."""

    def __init__(self, path):
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.btn, self.ax, self.edges = set(), {}, []

    def poll(self):
        """읽을 게 없을 때까지 비운다. edges 는 호출자가 소비하고 비운다."""
        self.edges = []
        while True:
            try:
                data = os.read(self.fd, EV_SIZE)
            except BlockingIOError:
                return
            except OSError:                    # 동글이 빠졌다
                raise SystemExit("\n패드 연결이 끊겼다 (USB 재삽입 후 서비스 재시작)")
            if len(data) < EV_SIZE:
                return
            _s, _us, etype, code, val = struct.unpack(EV_FMT, data)
            if etype == EV_KEY:
                (self.btn.add if val else self.btn.discard)(code)
            elif etype == EV_ABS:
                if code in (HAT_X, HAT_Y) and val:
                    self.edges.append((code, val))     # 0 복귀는 무시 → 한 번만 센다
                self.ax[code] = val


def find_pad():
    """by-id 로 찾는다 — event 번호는 재연결마다 바뀐다.
    IMU(`-event-if00`)는 자세 데이터를 초당 수백 개 뿜으므로 반드시 제외한다."""
    for p in sorted(glob.glob("/dev/input/by-id/*-event-joystick")):
        if "Microsoft" not in p:               # 기존 디지털 조이스틱은 joy-teleop 담당
            return p
    raise SystemExit("새 패드를 못 찾았다: /dev/input/by-id/*-event-joystick 없음")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default=None)
    ap.add_argument("--hosts", default=",".join(mqtt_link.HOSTS),
                    help="브로커 후보. **앞이 우선순위**(유선 → 무선)")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--prefix", default="navi")
    ap.add_argument("--enable-topic", default="ipc/drive_enable")
    ap.add_argument("--period", type=float, default=0.35,
                    help="반복 발행 주기 s. 350ms 인 이유는 README 함정 1번")
    ap.add_argument("--vmax", type=float, default=0.0239,
                    help="시작 주행 최대속도 m/s (기본 = 기존 조이스틱과 같은 3 RPM)")
    # 사용자 규격은 0.1 m/s 스텝인데, 축 상한 20 RPM = 0.159 m/s 라 한 칸만 올려도
    # 천장이다. 그래서 기본은 0.02(≈2.5 RPM)로 두고 규격값은 인자로 열어 둔다.
    ap.add_argument("--vmax-step", type=float, default=0.02)
    ap.add_argument("--vmax-lo", type=float, default=0.02)
    ap.add_argument("--vmax-hi", type=float, default=round(VMAX_HW, 4))
    ap.add_argument("--wmax-deg", type=float, default=2.0, help="시작 회전 최대속도 °/s")
    ap.add_argument("--wmax-step-deg", type=float, default=2.0)
    ap.add_argument("--wmax-lo", type=float, default=2.0)
    ap.add_argument("--wmax-hi", type=float, default=30.0)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()

    pad = Pad(a.dev or find_pad())
    st = {}

    def on_connect(c, u, flags, rc):
        c.subscribe([(a.prefix + "/state", 0), (a.prefix + "/event", 1),
                     (a.enable_topic, 1)])
        print(f"[mqtt] rc={rc}  구동잠김(콘솔에서 허용해야 움직인다)", flush=True)

    def on_message(c, u, msg):
        try:
            d = json.loads(msg.payload)
        except ValueError:
            return
        if msg.topic == a.enable_topic:
            was, st["enabled"] = st.get("enabled"), bool(d.get("on"))
            if was != st["enabled"]:
                print(f"\n[구동허용] {st['enabled']} → 이 서브 패드는 "
                      f"{'대기(기존 조이스틱 차례)' if st['enabled'] else '**활성**'}",
                      flush=True)
            return
        if msg.topic.endswith("/event"):
            print(f"\n[event] {d}", flush=True)
            return
        if d.get("estop") and not st.get("estop"):
            print("\n⚠ e-stop 래치 — cmd/reset 만 해제된다", flush=True)
        st["estop"] = bool(d.get("estop"))

    cli = mqtt_link.Link(hosts=a.hosts.split(","), port=a.port,
                         on_connect=on_connect, on_message=on_message)

    vmax, wdeg = a.vmax, a.wmax_deg
    moving, last_rpm, last_pub = False, None, 0.0
    print(f"장치: {a.dev or '자동'}  주행 {vmax:.3f} m/s · 회전 {wdeg:.1f}°/s\n"
          f"조건: 콘솔 구동허용 **OFF** + LB+RB 홀드 (허용 ON 이면 기존 조이스틱 차례)",
          flush=True)
    try:
        while True:
            pad.poll()
            cli.tick()
            if pad.edges:
                vmax, wdeg = step_limits(pad.edges, vmax, wdeg, a)
                print(f"\n[속도] 주행 {vmax:.3f} m/s · 회전 {wdeg:.1f}°/s", flush=True)

            vx, vy, wz = resolve(pad.btn, pad.ax, vmax,
                                 math.radians(wdeg) * ROT_ARM_M)
            if not allowed(st.get("enabled")):
                vx = vy = wz = 0.0            # 기존 조이스틱 차례 = 손을 뗀 것과 동일 취급
            rpm = mecanum_rpm(vx, vy, wz, cap=vmax * RPM_PER_MS)
            now = time.monotonic()
            if any(rpm) and due(rpm, last_rpm, now, last_pub, a.period):
                cli.publish(a.prefix + "/cmd/wheel",
                            json.dumps({"rpm": rpm, "ramp": RAMP}), qos=1)
                moving, last_pub = True, now
                mode = "횡이동" if BTN_STRAFE in pad.btn else "주행  "
                print(f"\r[{mode}] vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f} rpm={rpm}   ",
                      end="", flush=True)
            elif not any(rpm) and moving:
                # rpm=[0,0,0,0] 은 절대 보내지 않는다 — 여자 유지로 과전류 e-stop 위험
                cli.publish(a.prefix + "/cmd/stop", "{}", qos=1)
                moving = False
                why = ("대기 — 콘솔 구동허용을 **끄면** 이 패드 차례"
                       if st.get("enabled") is True
                       else "대기 — 구동허용 상태 수신 전"
                       if st.get("enabled") is None
                       else "정지 (데드맨 해제)"
                       if not all(b in pad.btn for b in BTN_DEADMAN) else "정지")
                print(f"\r{why}{' ' * 28}", end="", flush=True)
            last_rpm = rpm
            time.sleep(POLL)
    except KeyboardInterrupt:
        pass
    finally:
        cli.publish(a.prefix + "/cmd/stop", "{}", qos=1)
        time.sleep(0.2)
        cli.stop()
        print("\n정지 명령 발행 후 종료")


def selftest():
    both = set(BTN_DEADMAN)

    # ── 데드맨: 하나만 눌러서는 절대 안 움직인다 (가장 중요한 성질) ──
    full = {AX_FWD: -32767}
    assert resolve(set(), full, 0.1, 0.1) == (0.0, 0.0, 0.0)
    assert resolve({BTN_DEADMAN[0]}, full, 0.1, 0.1) == (0.0, 0.0, 0.0)
    assert resolve({BTN_DEADMAN[1]}, full, 0.1, 0.1) == (0.0, 0.0, 0.0)
    assert resolve(both, full, 0.1, 0.1) != (0.0, 0.0, 0.0)

    # ── 장착 보정: 사용자 "전진" 은 **vy** 로 나가야 한다 (vx 로 가면 90° 틀린다) ──
    vx, vy, wz = resolve(both, {AX_FWD: -32767}, 0.1, 0.1)
    assert vx == 0.0 and abs(vy - 0.1) < 1e-9 and wz == 0.0, (vx, vy, wz)
    back = resolve(both, {AX_FWD: 32767}, 0.1, 0.1)
    assert back[1] < 0 and back[0] == 0.0, back

    # ── X 홀드: 횡이동이 **vx** 로 나가고 회전은 죽는다 ──
    right = resolve(both | {BTN_STRAFE}, {AX_STRAFE: 32767, AX_YAW: 32767}, 0.1, 0.1)
    assert abs(right[0] - 0.1) < 1e-9 and right[2] == 0.0, right
    left = resolve(both | {BTN_STRAFE}, {AX_STRAFE: -32767}, 0.1, 0.1)
    assert left[0] < 0, left
    # X 를 놓으면 좌스틱 좌우는 아무것도 아니다
    assert resolve(both, {AX_STRAFE: 32767}, 0.1, 0.1) == (0.0, 0.0, 0.0)

    # ── 회전: 우스틱 오른쪽 = 우회전(-wz), 왼쪽 = 좌회전(+wz) ──
    assert resolve(both, {AX_YAW: 32767}, 0.1, 0.5)[2] < 0
    assert resolve(both, {AX_YAW: -32767}, 0.1, 0.5)[2] > 0

    # ── 데드존: 중립 드리프트는 0, 데드존 바깥은 0 에서 다시 시작 ──
    assert axis_norm(int(AX_MAX * 0.05)) == 0.0
    assert axis_norm(int(AX_MAX * DEAD * 0.99)) == 0.0
    assert 0.0 < axis_norm(int(AX_MAX * (DEAD + 0.01))) < 0.05
    assert abs(axis_norm(32767) - 1.0) < 1e-6
    assert abs(axis_norm(-32767) + 1.0) < 1e-6
    assert abs(axis_norm(99999)) <= 1.0, "범위를 넘는 값도 ±1 로 잘려야 한다"

    # ── 비례 제어: 반쯤 꺾으면 반쯤 속도 (데드존 보정 후) ──
    half = resolve(both, {AX_FWD: -int(AX_MAX * (DEAD + (1 - DEAD) * 0.5))}, 0.1, 0.1)
    assert abs(half[1] - 0.05) < 0.002, half

    # ── D패드 속도 조절: 눌린 순간 한 칸, 상·하한에서 멈춘다 ──
    class A:
        vmax_step, wmax_step_deg = 0.02, 2.0
        vmax_lo, vmax_hi, wmax_lo, wmax_hi = 0.02, 0.16, 2.0, 30.0
    assert step_limits([(HAT_Y, -1)], 0.10, 10.0, A) == (0.12, 10.0)   # 위 = 증속
    assert step_limits([(HAT_Y, +1)], 0.10, 10.0, A) == (0.08, 10.0)   # 아래 = 감속
    assert step_limits([(HAT_X, +1)], 0.10, 10.0, A) == (0.10, 12.0)   # 오른쪽 = 회전 증속
    assert step_limits([(HAT_X, -1)], 0.10, 10.0, A) == (0.10, 8.0)
    assert step_limits([(HAT_Y, +1)], 0.02, 2.0, A) == (0.02, 2.0), "하한에서 더 안 내려간다"
    assert step_limits([(HAT_Y, -1)], 0.16, 30.0, A) == (0.16, 30.0), "상한에서 더 안 올라간다"
    assert step_limits([], 0.10, 10.0, A) == (0.10, 10.0)

    # ── 상한: 어떤 조합도 축 상한을 넘지 않는다 (혼합 지령 포함) ──
    v = 0.05
    mixed = resolve(both | {BTN_STRAFE}, {AX_FWD: -32767, AX_STRAFE: 32767}, v, 0.1)
    assert max(abs(r) for r in mecanum_rpm(*mixed, cap=v * RPM_PER_MS)) \
        <= v * RPM_PER_MS + 0.02
    assert max(abs(r) for r in mecanum_rpm(*resolve(
        both, {AX_FWD: -32767, AX_YAW: 32767}, VMAX_HW, 0.5))) <= MAX_RPM + 1e-6

    # ── 조종기 선택: 허용 OFF 에서만 활성, **모르는 상태는 막힌다** ──
    assert allowed(False) is True, "허용 OFF = 서브 패드 차례"
    assert allowed(True) is False, "허용 ON 이면 기존 조이스틱 차례 — 이중 명령 금지"
    assert allowed(None) is False, "수신 전에는 막아야 한다(기본값 열림이면 이중 명령 위험)"

    # ── 두 조이스틱 공존의 근거: 정지 상태에서는 rpm 이 전부 0 → 발행 안 함 ──
    assert not any(mecanum_rpm(*resolve(set(), full, 0.1, 0.1)))
    print("selftest OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
