#!/usr/bin/env python3
"""상용 아날로그 게임패드 → 메카넘 → navi/cmd/wheel. 기존 joy-teleop 과 **동시 운용**.

기존 조이스틱(디지털 4방향, js1)은 그대로 둔다. 두 프로세스가 같은 브로커에
발행하지만 서로 안 싸운다 — 양쪽 다 **움직일 때만 발행하고, 놓으면 `cmd/stop`
한 번 보낸 뒤 침묵**하는 계약을 지키기 때문이다(joy_teleop 의 발행 분기와 동일).
둘을 **동시에** 꺾으면 마지막에 도착한 명령이 이긴다 — 그건 어쩔 수 없다.

  조작 (사용자가 정한 규칙, 2026-09-02)
    LB + RB 홀드   데드맨. **둘 다** 눌러야 움직이고, 떼면 즉시 정지
    왼쪽 스틱 ↑↓   전진 / 후진 (기울인 만큼 비례)
    X 홀드 + ←→    좌 / 우 횡이동(=차체 전후진 명령). 이 동안 **회전은 죽는다**
    오른쪽 스틱 ←→ 좌 / 우회전 (비례)
    D패드 ↑↓       주행 최대속도 ± (--vmax-step)
    D패드 →←       회전 최대속도 ± (--wmax-step-deg)

🔴 **콘솔 구동허용이 OFF 일 때만 동작한다** — 기존 조이스틱은 ON 에서만 움직이므로
   토글이 조종기 선택 스위치가 되고 이중 명령이 불가능해진다 (allowed() 주석 참고).

⚠️ 연사(터보)가 X(308)에 걸리면 홀드가 100 Hz 로 떨려 횡이동/회전이 요동친다.
   패드 펌웨어 기능이라 리눅스에서 못 끈다 — TURBO(또는 HOME)+해당 버튼으로 해제한다.

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
from joy_teleop import (GHOST_COOLDOWN, MAIN_JS_MATCH, MAX_RPM, RAMP, RPM_PER_MS,
                        due, ghost, mecanum_rpm)

# struct input_event: timeval(long sec, long usec) + u16 type + u16 code + s32 value
EV_FMT, EV_SIZE = "llHHi", 24
EV_KEY, EV_ABS = 0x01, 0x03

# ── 실측 매핑 (2026-09-02, joy2_map.py) ───────────────────────────────────
BTN_DEADMAN = (310, 311)   # BTN_TL / BTN_TR = LB / RB (위쪽 어깨). 아래쪽 ZL·ZR 은 312/313
# 🔴 **이 동글은 모드가 바뀐다** — 그러면 물리 X 버튼의 코드도 바뀐다(실측 2026-09-02):
#
#      Switch 모드  057e:2009 → hid_nintendo → **위치**로 코드 부여 → 물리 X = 308
#      X-input 모드 3537:103e → xpad        → **글자**로 코드 부여 → 물리 X = 307
#
#    리눅스 코드 이름이 위치와 어긋나 있어서 그렇다: BTN_X=307 인데 위치는 NORTH(위),
#    BTN_Y=308 인데 위치는 WEST(왼쪽). 재부팅으로 모드가 바뀌자 "Y 를 눌러야 횡이동이
#    되는" 상태가 됐다.
#
#    → 드라이버를 판별해 고르는 대신 **두 코드를 모두 받는다.** 나머지 면 버튼(A·B)에는
#      아무 기능이 없으므로 잃는 게 없고, 모드가 바뀌어도 조작이 안 바뀐다.
#      데드맨 LB/RB(310/311)는 두 모드에서 같다 — 실측 확인.
BTN_STRAFE = (307, 308)    # 물리 X. 모드에 따라 둘 중 하나로 온다
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
# 🔴 부호 조정은 **이 세 줄이 전부다.** 물리적 질문 하나당 상수 하나 —
#    코드 다른 곳에 `-` 를 두지 않는다(두 곳에 있으면 "한 줄만 뒤집어라" 가 거짓말이 된다).
#    각 상수는 (스틱 원시 극성 × 장착 보정)을 **합친** 값이다. 스틱은 위·왼쪽이 음수다.
#    실기에서 **한 번에 하나씩** 확인할 것 — 둘을 같이 뒤집으면 원인을 못 가린다.
SIGN_FWD = -1.0      # 좌스틱 **위** → 물리적 전진 (위가 음수라 기본이 -1)
SIGN_STRAFE = -1.0   # 좌스틱 **오른쪽** → 물리적 우횡이동 (2026-09-02 실기에서 반전)
SIGN_YAW = -1.0      # 우스틱 **오른쪽** → 우회전(CW). +wz 는 CCW 라 기본이 -1

# 회전 지령을 °/s 로 받기 위한 환산 팔길이. mecanum_rpm 의 wz 는 vx·vy 와 같은
# 단위(선속도)로 들어가므로 (lx+ly)/2 가 필요하다. 차체 치수 미실측(kinematics:false)이라
# 추정값이고, **회전 속도의 게인이 곧 이 값이다.**
#
# 0.30 → 0.10 (2026-09-02): 실기에서 회전이 너무 빨라 사용자가 1/3 을 요청했다.
# 라벨(°/s)은 규격대로 두고 이 값만 줄였다 — 즉 "0.30 이 3배 과대추정이었다" 로 본다.
# 제자리 360° 회전 시간을 재면 정확한 값이 나온다. 방향에는 영향 없다.
ROT_ARM_M = 0.10

POLL = 0.02          # 아날로그라 기존(0.05)보다 촘촘히 본다. 발행 주기와는 무관

# 지령 하한은 **두지 않는다.** 한때 크리프(손만 얹혀 있을 때 조금씩 도는 것)를 막으려고
# 넣었는데 두 번 잘못됐다:
#   1) 출력 rpm 에 걸었더니 각속도를 1/3 로 줄인 직후 **제자리 회전이 통째로 죽었다**
#      (회전 지령이 축당 0.44 RPM < 하한 0.5). 크리프와 "의도한 느린 회전" 은
#      rpm 크기로 구분되지 않는다.
#   2) 입력 쪽으로 옮겨도 **문턱이 두 개**(데드존 + 하한)가 되어 조작 감각이 계단이 된다.
# 그리고 애초에 **데드맨(LB+RB)이 있다** — 손을 놓으면 즉시 멈춘다(사용자 지적 2026-09-02).
# 크리프가 실제로 거슬리면 개념이 하나인 **DEAD 를 키운다.** 문턱을 새로 만들지 않는다.
#
# ⚠️ 남는 한 가지: 데드맨을 잡은 채 스틱이 살짝 꺾여 있으면 아주 작은 지령이 350ms 마다
#    계속 나가고, 그러면 "명령이 곧 생존 신호" 인 로봇 워치독이 안 걸린다. 조종자가
#    붙어 있는 상태라 위험은 낮다. 실제로 관측된 "명령 끝났는데 도는" 현상은 이게
#    아니었다 — 발행이 끊기고 워치독까지 걸린 뒤에도 RL 이 -0.4 RPM 을 보고했다(MQTT 하류).

# 🔴 무선 패드가 **끊겨도 마지막 입력 상태가 그대로 남는다** — 버튼 집합과 축 값이
#    메모리에 있으니 데드맨이 눌린 채로 굳고, 로봇은 마지막 명령으로 계속 간다.
#    (동글을 뽑으면 read 가 죽어서 서비스가 재시작되지만, **컨트롤러만 꺼지면**
#     노드는 살아있어서 아무 일도 안 일어난다 — 이쪽이 위험하다.)
#    그래서 입력이 이 시간 이상 없으면 데드맨을 놓은 것으로 본다.
#
# 관측(2026-09-02 실주행): 이 타임아웃이 두 번 걸렸는데 **둘 다 유휴 구간**이었다
#    (데드맨을 놓은 뒤, 또는 패드를 안 만지는 동안) → 주행을 끊지 않았다. 유휴 중에는
#    어차피 rpm 이 0 이라 무해하다.
# ⚠️ 다만 **스틱을 끝까지 밀어 붙인 상태**는 아직 검증 못 했다. evdev 는 값이 바뀔 때만
#    이벤트를 주므로 32767 에 포화되면 이벤트가 끊길 수 있다(EV_SYN 도 빈 sync 는
#    전달되지 않는 것을 로그로 확인). 그 상태로 2초가 넘으면 주행이 끊긴다.
#    로그에 주행 중 "패드 입력 끊김" 이 뜨면 --input-timeout 을 올리거나 0 으로 끈다.
#    오동작 방향이 **정지**라서 켠 채로 쓴다.
INPUT_TIMEOUT = 2.0
DEAD = 0.12          # 정규화 데드존. 스틱 중립 드리프트가 주행으로 새는 걸 막는다
VMAX_HW = MAX_RPM / RPM_PER_MS      # 축 상한(20 RPM)이 정하는 물리 최고속 ≈ 0.159 m/s

# D패드 속도 조절 범위 (사용자 지정 2026-09-02). 흩어진 리터럴 대신 여기 모아두고
# selftest 가 정합성(하한 ≤ 시작 ≤ 상한, 상한 ≤ 물리 최고속)을 검사한다.
#
#   주행  0.01 ─ 0.05 m/s   =  1.3 ─  6.3 RPM   (한 칸 0.01 = 1.26 RPM)
#   회전  1.0  ─ 6.0  °/s   = 0.22 ─ 1.32 RPM   (한 칸 2.0  = 0.44 RPM)
#
# 회전 상한 6°/s 는 1/3 감속 **전**의 기본 회전 속도와 같다(0.30 팔길이 × 2°/s = 1.32 RPM).
# 하한을 0.01 / 1.0 으로 내린 것은 사용자 지정(2026-09-02) — 기본값·상한은 그대로.
#
# ⚠️ 하한이 스텝의 배수가 아니라 **내려갔다 올라오면 격자가 달라진다.** 회전은
#    2 → (한 칸 내림) 1 → 3 → 5 → 6 이 된다(2·4 로는 다시 안 돌아온다). 주행도
#    0.0239 에서 시작하니 격자가 0.0039 만큼 어긋나 있다. 조작에는 문제 없고,
#    딱 떨어지는 값이 필요하면 START 를 격자에 맞추면 된다.
VMAX_START, VMAX_STEP, VMAX_LO, VMAX_HI = 0.0239, 0.01, 0.01, 0.05
WDEG_START, WDEG_STEP, WDEG_LO, WDEG_HI = 2.0, 2.0, 1.0, 6.0


def status(msg, force=False, _s={"t": 0.0}):
    """tty 면 한 줄 갱신(\r), 서비스로 돌 때는 **개행해서 journald 에 남긴다**.

    🔴 \r 만 쓰면 journald 에 한 줄도 안 남는다(개행이 없으니 커밋되지 않는다).
       2026-09-02 실주행에서 "횡이동이 안 된다" 를 로그로 확인할 방법이 없었다.
       발행 주기(~3회/s)로 다 남기면 저널이 넘치므로 1초에 한 줄로 줄인다.
       force=True 는 상태 전이(정지 등) — 빈도와 무관하게 항상 남긴다.
    """
    if sys.stdout.isatty():
        print("\r" + msg, end="", flush=True)
        return
    now = time.monotonic()
    if force or now - _s["t"] >= 1.0:
        _s["t"] = now
        print(msg, flush=True)


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
    문턱은 이 데드존 **하나뿐이다** — 위 주석 참고.
    """
    v = max(-1.0, min(1.0, raw / AX_MAX))
    if abs(v) < dead:
        return 0.0
    return math.copysign((abs(v) - dead) / (1.0 - dead), v)


def to_body(ly, lx, rx):
    """스틱 **원시 극성** 값 → 차체 (vx, vy, wz). 부호 보정이 일어나는 유일한 곳이다.

    **여기가 -90° 장착 보정의 전부다**: 좌스틱 상하(ly)가 vy 로, 좌우(lx)가 vx 로 간다.
    로봇의 물리적 전진이 조종자 기준 우측이기 때문이다(파일 상단 주석).
    """
    return SIGN_STRAFE * lx, SIGN_FWD * ly, SIGN_YAW * rx


def resolve(btn, ax, vmax, wz_max):
    """입력 상태 → (vx, vy, wz). 데드맨이 안 잡혀 있으면 전부 0 이다.

    X 홀드 중에는 횡이동이 살고 **회전이 죽는다**(사용자 규칙). 반대로 X 를 놓으면
    좌우 스틱은 아무 것도 아니다 — 횡이동은 X 모드에서만 나온다.
    """
    if not all(b in btn for b in BTN_DEADMAN):
        return 0.0, 0.0, 0.0
    # 여기서는 **부호를 만지지 않는다** — 전부 to_body 의 SIGN_* 가 소유한다
    ly = axis_norm(ax.get(AX_FWD, 0)) * vmax
    if btn & set(BTN_STRAFE):
        return to_body(ly, axis_norm(ax.get(AX_STRAFE, 0)) * vmax, 0.0)
    return to_body(ly, 0.0, axis_norm(ax.get(AX_YAW, 0)) * wz_max)


def step_limits(hat_edges, vmax, wdeg, a):
    """D패드 엣지 → (주행 최대속도, 회전 최대속도°/s). 눌린 순간에만 한 칸 움직인다."""
    for code, val in hat_edges:
        if code == HAT_Y:                     # 위=-1 → 증속
            vmax += -val * a.vmax_step
        elif code == HAT_X:                   # 오른쪽=+1 → 회전 증속
            wdeg += val * a.wmax_step_deg
    return (min(a.vmax_hi, max(a.vmax_lo, round(vmax, 4))),
            min(a.wmax_hi, max(a.wmax_lo, round(wdeg, 2))))


class PadGone(Exception):
    """동글이 빠졌거나 노드가 사라졌다 → 재오픈이 필요하다.

    프로세스를 죽이지 않는 이유: 죽이면 systemd 가 3초마다 재시작하는데, 동글을
    빼놓고 두면 **재시작 카운터가 1396까지 올라가고**(2026-09-02 실측) 유닛 상태가
    `activating` 으로 남아 "고장난 것" 처럼 보인다. crevis_io 가 Modbus 끊김을
    다루는 방식과 같게 **안에서 기다린다.** 동글을 다시 꽂으면(event 번호가 바뀌어도)
    by-id 로 다시 찾아 붙는다.
    """


class Pad:
    """evdev 논블로킹 리더. 버튼 집합 · 축 값 · D패드 상승엣지만 들고 있는다."""

    def __init__(self, path):
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.btn, self.ax, self.edges = set(), {}, []

    def poll(self):
        """읽을 게 없을 때까지 비운다. **읽은 이벤트 수**를 돌려준다(입력 끊김 감시용).
        edges 는 호출자가 소비하고 비운다."""
        self.edges, n = [], 0
        while True:
            try:
                data = os.read(self.fd, EV_SIZE)
            except BlockingIOError:
                return n
            except OSError:                    # 동글이 빠졌다
                raise PadGone
            if len(data) < EV_SIZE:
                return n
            n += 1
            _s, _us, etype, code, val = struct.unpack(EV_FMT, data)
            if etype == EV_KEY:
                (self.btn.add if val else self.btn.discard)(code)
            elif etype == EV_ABS:
                if code in (HAT_X, HAT_Y) and val:
                    self.edges.append((code, val))     # 0 복귀는 무시 → 한 번만 센다
                self.ax[code] = val


def find_pad(exclude=MAIN_JS_MATCH, paths=None):
    """by-id 로 찾는다 — event 번호는 재연결·재부팅마다 바뀐다.

    **기존 조이스틱만 배제**하고 나머지를 잡는다. 동글 이름이 모드에 따라
    `Pro_Controller` ↔ `GameSir-Dongle` 로 바뀌므로 이름을 못 박을 수 없다.
    joy_teleop 은 반대로 그 이름을 **포함 매칭**하므로 둘이 겹치지 않는다.

    IMU(`-event-if00`)는 자세 데이터를 초당 수백 개 뿜는데 `-event-joystick` 만
    보므로 자동으로 걸러진다.
    """
    cands = paths if paths is not None else sorted(
        glob.glob("/dev/input/by-id/*-event-joystick"))
    for p in cands:
        if exclude not in p:
            return p
    return None                        # 없으면 호출자가 기다린다 (PadGone 주석 참고)


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
    ap.add_argument("--vmax", type=float, default=VMAX_START,
                    help="시작 주행 최대속도 m/s (기본 = 기존 조이스틱과 같은 3 RPM)")
    ap.add_argument("--vmax-step", type=float, default=VMAX_STEP)
    ap.add_argument("--vmax-lo", type=float, default=VMAX_LO)
    ap.add_argument("--vmax-hi", type=float, default=VMAX_HI)
    ap.add_argument("--wmax-deg", type=float, default=WDEG_START, help="시작 회전 최대속도 °/s")
    ap.add_argument("--wmax-step-deg", type=float, default=WDEG_STEP)
    ap.add_argument("--wmax-lo", type=float, default=WDEG_LO)
    ap.add_argument("--wmax-hi", type=float, default=WDEG_HI)
    ap.add_argument("--input-timeout", type=float, default=INPUT_TIMEOUT,
                    help="패드 입력이 이 시간 없으면 데드맨을 놓은 것으로 본다")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()

    pad, st = None, {}

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
        st["rpm_fb"] = tuple(w.get("rpm") for w in d.get("wheels", []))
        cur = (d.get("drive_ok"), d.get("wheels_alive"), bool(d.get("estop")))
        if cur != st.get("last"):
            st["last"] = cur
            print(f"\n[state] drive_ok={cur[0]} alive={cur[1]} estop={cur[2]}", flush=True)
            if cur[2]:
                print("        ⚠ e-stop 래치 — cmd/reset 만 해제된다", flush=True)

    cli = mqtt_link.Link(hosts=a.hosts.split(","), port=a.port,
                         on_connect=on_connect, on_message=on_message)

    vmax, wdeg = a.vmax, a.wmax_deg
    moving, last_rpm, last_pub = False, None, 0.0
    last_input, stale, last_ghost = time.monotonic(), False, 0.0
    print(f"주행 {vmax:.3f} m/s ({vmax * RPM_PER_MS:.1f} RPM) · 회전 {wdeg:.1f}°/s\n"
          f"조건: 콘솔 구동허용 **OFF** + LB+RB 홀드 (허용 ON 이면 기존 조이스틱 차례)",
          flush=True)
    last_wait_log = 0.0

    def stop_now(why):
        """발행을 끊는 것만으로는 부족하다 — 명시적으로 세운다."""
        cli.publish(a.prefix + "/cmd/stop", "{}", qos=1)
        print(f"\n[정지] {why}", flush=True)

    try:
        while True:
            # ── 패드가 없으면 **안에서 기다린다** (죽지 않는다 — PadGone 주석 참고) ──
            if pad is None:
                path = a.dev or find_pad()
                now = time.monotonic()
                if path is None:
                    if moving:
                        moving = False
                        stop_now("패드 없음")
                    if now - last_wait_log >= 10.0:
                        last_wait_log = now
                        print("[패드] 대기 중 — 동글/컨트롤러 전원 확인", flush=True)
                    cli.tick()
                    time.sleep(1.0)
                    continue
                pad = Pad(path)
                last_input, stale = now, False
                print(f"[패드] 연결: {path}", flush=True)

            try:
                got = pad.poll()
            except PadGone:
                pad = None
                if moving:
                    moving = False
                    stop_now("패드 연결 끊김")
                else:
                    print("\n[패드] 연결 끊김 — 재연결 대기", flush=True)
                continue
            cli.tick()
            now = time.monotonic()
            if got:
                last_input = now
                if stale:
                    stale = False
                    print("\n[입력] 패드 복구", flush=True)
            elif a.input_timeout and not stale and now - last_input > a.input_timeout:
                stale = True
                print(f"\n⚠ [입력] 패드에서 {a.input_timeout:.1f}초간 아무것도 안 온다 — "
                      f"정지한다 (컨트롤러 전원·배터리 확인)", flush=True)
            if pad.edges:
                vmax, wdeg = step_limits(pad.edges, vmax, wdeg, a)
                print(f"\n[속도] 주행 {vmax:.3f} m/s · 회전 {wdeg:.1f}°/s", flush=True)

            vx, vy, wz = resolve(pad.btn, pad.ax, vmax,
                                 math.radians(wdeg) * ROT_ARM_M)
            if stale or not allowed(st.get("enabled")):
                vx = vy = wz = 0.0            # 손을 뗀 것과 동일 취급
            rpm = mecanum_rpm(vx, vy, wz, cap=vmax * RPM_PER_MS)
            if any(rpm) and due(rpm, last_rpm, now, last_pub, a.period):
                cli.publish(a.prefix + "/cmd/wheel",
                            json.dumps({"rpm": rpm, "ramp": RAMP}), qos=1)
                moving, last_pub = True, now
                mode = "횡이동" if pad.btn & set(BTN_STRAFE) else "주행  "
                status(f"[{mode}] vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f} rpm={rpm}"
                       f"  btn={sorted(pad.btn)}")
            elif not any(rpm) and moving:
                # rpm=[0,0,0,0] 은 절대 보내지 않는다 — 여자 유지로 과전류 e-stop 위험
                cli.publish(a.prefix + "/cmd/stop", "{}", qos=1)
                moving = False
                why = ("정지 — 패드 입력 끊김" if stale
                       else "대기 — 콘솔 구동허용을 **끄면** 이 패드 차례"
                       if st.get("enabled") is True
                       else "대기 — 구동허용 상태 수신 전"
                       if st.get("enabled") is None
                       else "정지 (데드맨 해제)"
                       if not all(b in pad.btn for b in BTN_DEADMAN) else "정지")
                status(why, force=True)
            # ── 유령 구동 감시: 내가 활성 조종기이고 명령을 안 보내는 중인데 모터가 돈다 ──
            #    (활성일 때만 본다 — 기존 조이스틱이 주행 중일 때 정지를 쏘면 안 된다)
            if (not moving and allowed(st.get("enabled"))
                    and ghost(st.get("rpm_fb"), now - last_pub)
                    and now - last_ghost >= GHOST_COOLDOWN):
                last_ghost = now
                cli.publish(a.prefix + "/cmd/stop", "{}", qos=1)
                print(f"\n⚠ [유령] 명령이 없는데 모터가 돈다 {st.get('rpm_fb')} — 정지 발행",
                      flush=True)
            last_rpm = rpm
            time.sleep(POLL)
    except KeyboardInterrupt:
        pass
    finally:
        # 🔴 프로세스가 죽는 순간에도 반드시 세운다
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
    #
    # 🔴 여기서 **부호는 검사하지 않는다.** SIGN_* 는 실기 캘리브레이션 값이라
    #    뒤집힐 수 있고, 테스트가 그걸 고정하면 부호를 고칠 때마다 테스트가 깨진다
    #    (2026-09-02 SIGN_STRAFE 반전에서 실제로 그랬다). 검사할 것은 로직이다:
    #    **어느 축으로 가는가 · 반대 입력이 반대로 나오는가 · 크기가 vmax 인가.**
    up = resolve(both, {AX_FWD: -32767}, 0.1, 0.1)
    down = resolve(both, {AX_FWD: 32767}, 0.1, 0.1)
    assert up[0] == 0.0 and up[2] == 0.0, ("전진이 vy 아닌 축으로 샜다", up)
    assert abs(abs(up[1]) - 0.1) < 1e-9, up
    assert up[1] == -down[1] and down[1] != 0.0, ("전/후진이 대칭이 아니다", up, down)

    # ── X 홀드: 횡이동이 **vx** 로 나가고 회전은 죽는다 ──
    right = resolve(both | set(BTN_STRAFE), {AX_STRAFE: 32767, AX_YAW: 32767}, 0.1, 0.1)
    left = resolve(both | set(BTN_STRAFE), {AX_STRAFE: -32767}, 0.1, 0.1)
    assert abs(abs(right[0]) - 0.1) < 1e-9, right
    assert right[2] == 0.0, ("X 홀드 중에는 회전이 죽어야 한다", right)
    assert right[0] == -left[0] and left[0] != 0.0, ("좌/우 횡이동이 대칭이 아니다", right, left)
    # X 를 놓으면 좌스틱 좌우는 아무것도 아니다
    assert resolve(both, {AX_STRAFE: 32767}, 0.1, 0.1) == (0.0, 0.0, 0.0)

    # ── 회전: 좌우가 반대이고 크기가 상한이다 ──
    cw = resolve(both, {AX_YAW: 32767}, 0.1, 0.5)
    ccw = resolve(both, {AX_YAW: -32767}, 0.1, 0.5)
    assert cw[2] == -ccw[2] and cw[2] != 0.0, (cw, ccw)
    assert abs(abs(cw[2]) - 0.5) < 1e-9, cw

    # ── 데드존: 중립 드리프트는 0, 데드존 바깥은 0 에서 다시 시작 ──
    assert axis_norm(int(AX_MAX * 0.05)) == 0.0
    assert axis_norm(int(AX_MAX * DEAD * 0.99)) == 0.0
    assert 0.0 < axis_norm(int(AX_MAX * (DEAD + 0.01))) < 0.05, "데드존 밖은 0 에서 시작"
    assert abs(axis_norm(32767) - 1.0) < 1e-6
    assert abs(axis_norm(-32767) + 1.0) < 1e-6
    assert abs(axis_norm(99999)) <= 1.0, "범위를 넘는 값도 ±1 로 잘려야 한다"

    # ── 비례 제어: 반쯤 꺾으면 반쯤 속도 (데드존 보정 후) ──
    half = resolve(both, {AX_FWD: -int(AX_MAX * (DEAD + (1 - DEAD) * 0.5))}, 0.1, 0.1)
    assert abs(abs(half[1]) - 0.05) < 0.002, half

    # ── D패드 속도 조절: 눌린 순간 한 칸, 상·하한에서 멈춘다 ──
    # 실제 설정값을 그대로 쓴다 — 이 테스트가 설정 문서 역할도 한다
    class A:
        vmax_step, wmax_step_deg = VMAX_STEP, WDEG_STEP
        vmax_lo, vmax_hi, wmax_lo, wmax_hi = VMAX_LO, VMAX_HI, WDEG_LO, WDEG_HI
    assert step_limits([(HAT_Y, -1)], 0.03, 4.0, A) == (0.04, 4.0)   # 위 = 증속
    assert step_limits([(HAT_Y, +1)], 0.03, 4.0, A) == (0.02, 4.0)   # 아래 = 감속
    assert step_limits([(HAT_X, +1)], 0.03, 4.0, A) == (0.03, 6.0)   # 오른쪽 = 회전 증속
    assert step_limits([(HAT_X, -1)], 0.03, 4.0, A) == (0.03, 2.0)
    assert step_limits([(HAT_Y, +1)], VMAX_LO, WDEG_LO, A) == (VMAX_LO, WDEG_LO), \
        "하한에서 더 안 내려간다"
    assert step_limits([(HAT_Y, -1)], VMAX_HI, WDEG_HI, A) == (VMAX_HI, WDEG_HI), \
        "상한에서 더 안 올라간다"
    assert step_limits([(HAT_X, -1)], 0.03, WDEG_LO, A) == (0.03, WDEG_LO)
    assert step_limits([], 0.03, 4.0, A) == (0.03, 4.0)

    # 설정 정합성 — 한 곳만 고쳐서 범위가 깨지는 걸 막는다
    assert VMAX_LO <= VMAX_START <= VMAX_HI, (VMAX_LO, VMAX_START, VMAX_HI)
    assert WDEG_LO <= WDEG_START <= WDEG_HI, (WDEG_LO, WDEG_START, WDEG_HI)
    assert 0 < VMAX_STEP <= VMAX_HI - VMAX_LO, "한 칸이 전체 범위보다 크면 조절이 안 된다"
    assert 0 < WDEG_STEP <= WDEG_HI - WDEG_LO
    assert VMAX_HI <= VMAX_HW + 1e-9, f"주행 상한이 축 상한({VMAX_HW:.4f} m/s)을 넘는다"

    # ── 상한: 어떤 조합도 축 상한을 넘지 않는다 (혼합 지령 포함) ──
    v = 0.05
    mixed = resolve(both | set(BTN_STRAFE), {AX_FWD: -32767, AX_STRAFE: 32767}, v, 0.1)
    assert max(abs(r) for r in mecanum_rpm(*mixed, cap=v * RPM_PER_MS)) \
        <= v * RPM_PER_MS + 0.02
    assert max(abs(r) for r in mecanum_rpm(*resolve(
        both, {AX_FWD: -32767, AX_YAW: 32767}, VMAX_HW, 0.5))) <= MAX_RPM + 1e-6

    # ── 부호 소유권: 보정은 to_body 한 곳에만 있어야 한다 ──
    #    resolve 가 부호를 또 만지면 SIGN_* 를 뒤집어도 결과가 안 바뀌거나 두 번 바뀐다
    import inspect
    body = inspect.getsource(resolve)
    assert "-axis_norm" not in body, "resolve 에 부호 보정이 남아 있다 (SIGN_* 가 유일해야 한다)"
    assert to_body(1.0, 0.0, 0.0) == (0.0, SIGN_FWD, 0.0)
    assert to_body(0.0, 1.0, 0.0) == (SIGN_STRAFE, 0.0, 0.0)
    assert to_body(0.0, 0.0, 1.0) == (0.0, 0.0, SIGN_YAW)

    # 회전 게인: 라벨 °/s → 차체 wz. 1/3 감속 후에도 방향은 유지돼야 한다
    assert ROT_ARM_M > 0
    assert math.radians(6.0) * ROT_ARM_M < math.radians(6.0) * 0.30, "게인이 안 줄었다"

    # 🔴 회귀 방지: **현재 설정으로 제자리 회전이 실제 지령을 만들어야 한다.**
    #    회전 게인(ROT_ARM_M)이나 기본 각속도를 낮추다가 0 으로 깎이면 회전이 죽는다.
    #    2026-09-02 에 각속도 1/3 + 출력 하한 조합으로 실제로 죽었다.
    spin = resolve(both, {AX_YAW: 32767}, 0.0239, math.radians(2.0) * ROT_ARM_M)
    assert any(mecanum_rpm(*spin, cap=0.0239 * RPM_PER_MS)), \
        f"현재 회전 게인으로 최대 꺾음이 0 이다 — 회전이 죽는다: {spin}"

    # ── 모드가 바뀌어도 횡이동이 되어야 한다 (동글이 글자↔코드를 바꾼다) ──
    for code in BTN_STRAFE:
        m = resolve(both | {code}, {AX_STRAFE: 32767}, 0.1, 0.1)
        assert m[0] != 0.0 and m[2] == 0.0, (code, m)

    # ── 장치 선택: 두 서비스가 같은 장치를 잡으면 안 된다 ──
    real = ["/dev/input/by-id/usb-_GameSir-Dongle_5F25F811-event-joystick",
            "/dev/input/by-id/usb-©Microsoft_Corporation_Controller_0D9C01C-event-joystick"]
    assert "GameSir" in find_pad(paths=real)
    # 동글이 없으면 **None** 이어야 한다 — 예외로 죽으면 systemd 재시작 루프가 된다
    assert find_pad(paths=[real[1]]) is None
    assert find_pad(paths=[]) is None
    from joy_teleop import find_joystick
    assert find_pad(paths=real) != find_joystick(paths=[
        p.replace("-event-joystick", "-joystick") for p in real])

    # ── 유령 감시는 활성 조종기만 — 아니면 서로를 죽인다 ──
    assert ghost((0.0, 0.0, -0.4, 0.0), 5.0)          # 판정 자체는 joy_teleop 에서 검증
    assert allowed(True) is False, "허용 ON 이면 유령 감시도 돌지 않아야 한다"

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
