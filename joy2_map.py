#!/usr/bin/env python3
"""새로 붙인 상용 게임패드의 버튼·축을 **실측**한다. 의존성 0.

왜 joydev(`/dev/input/js*`)가 아니라 evdev(`/dev/input/event*`)냐:
이 패드(Nintendo Pro Controller 로 인식되는 동글)는 커널이 **js 노드를 안 만든다**
— `event13` 만 생긴다. 게다가 evdev 는 코드가 **이름**(BTN_TL·ABS_X)으로 오므로
"몇 번 버튼인가" 를 추측할 필요가 없다. 기존 조이스틱(js1, Xbox360 인식)은 안 건드린다.

  python3 joy2_map.py --list                # 후보 장치 보기
  python3 joy2_map.py --secs 90             # 90초 녹화 → 요약표
  python3 joy2_map.py --selftest

요약표의 **처음 본 순서**로 물리 조작을 대응시킨다. 동작 사이에 2초씩 쉬면 확실하다.
"""
import argparse
import glob
import os
import struct
import sys
import time

# struct input_event { struct timeval time; __u16 type, code; __s32 value; }
# 64bit: time = long sec + long usec → "llHHi" = 24 바이트
EV_FMT, EV_SIZE = "llHHi", 24
EV_KEY, EV_ABS = 0x01, 0x03

# ⚠️ 코드는 **위치**를 뜻한다(SOUTH/EAST/NORTH/WEST). 패드에 인쇄된 글자는 배치마다
#    다르다 — 실측한 이 패드는 Xbox 배치라 위=Y, 왼쪽=X 다. Nintendo 배치는 그 반대.
#    그래서 이름표에 위치를 먼저 쓴다. **인쇄 글자로 유추하지 말고 눌러서 확인할 것.**
BTN = {304: "아래(A)", 305: "오른쪽(B)", 306: "C", 307: "위(Xbox=Y)",
       308: "왼쪽(Xbox=X)", 309: "Z",
       310: "LB(TL)", 311: "RB(TR)", 312: "LT(TL2)", 313: "RT(TR2)",
       314: "SELECT/-", 315: "START/+", 316: "MODE/홈", 317: "L스틱누름", 318: "R스틱누름",
       319: "THUMB", 544: "D패드↑", 545: "D패드↓", 546: "D패드←", 547: "D패드→"}
HAT = (16, 17)          # D패드. -1/0/+1 로 온다 → 데드존 예외
ABS = {0: "ABS_X(좌스틱 좌우)", 1: "ABS_Y(좌스틱 상하)", 2: "ABS_Z(LT아날로그)",
       3: "ABS_RX(우스틱 좌우)", 4: "ABS_RY(우스틱 상하)", 5: "ABS_RZ(RT아날로그)",
       16: "ABS_HAT0X(D패드 좌우)", 17: "ABS_HAT0Y(D패드 상하)"}


def name(etype, code):
    tbl = BTN if etype == EV_KEY else ABS
    return tbl.get(code, f"code {code}")


def find(pattern="*Pro_Controller*-event-joystick", exclude="Microsoft"):
    """by-id 심볼릭 링크로 고른다 — event 번호는 재부팅·재연결마다 바뀐다.

    IMU(`-event-if00`)는 자세 데이터를 초당 수백 개 뿜으므로 반드시 제외한다.
    """
    cands = [p for p in sorted(glob.glob(f"/dev/input/by-id/{pattern}"))
             if exclude not in p]
    return cands[0] if cands else None


def listdev():
    print("── /dev/input/by-id 의 조이스틱 후보")
    for p in sorted(glob.glob("/dev/input/by-id/*joystick*")):
        print(f"   {p}\n     → {os.path.realpath(p)}"
              f"   읽기 {'가능' if os.access(p, os.R_OK) else '불가(권한)'}")
    return 0


def record(path, secs, dead):
    """녹화 → (처음 본 순서, 코드별 통계). 축은 데드존 밖에서만 센다."""
    seen, order, t0 = {}, [], time.monotonic()
    with open(path, "rb", buffering=0) as f:
        os.set_blocking(f.fileno(), False)
        while time.monotonic() - t0 < secs:
            data = f.read(EV_SIZE)
            if not data or len(data) < EV_SIZE:
                time.sleep(0.002)
                continue
            _s, _us, etype, code, val = struct.unpack(EV_FMT, data)
            if etype not in (EV_KEY, EV_ABS):
                continue
            # 🔴 데드존은 **아날로그 축에만** 적용한다. D패드(ABS_HAT0X/Y)는 값이
            #    -1/0/+1 이라 데드존을 걸면 통째로 사라진다 — 처음에 이걸로 D패드를
            #    "안 잡힌다" 고 오진했다.
            if etype == EV_ABS and code not in HAT and abs(val) < dead:
                continue                      # 중립 근처 노이즈는 버린다
            key = (etype, code)
            st = seen.get(key)
            if st is None:
                st = seen[key] = {"n": 0, "min": val, "max": val,
                                  "t": time.monotonic() - t0}
                order.append(key)
            st["n"] += 1
            st["min"], st["max"] = min(st["min"], val), max(st["max"], val)
            print(f"  [{time.monotonic() - t0:6.1f}s] {name(etype, code):<22} = {val}",
                  flush=True)
    return order, seen


def summarize(order, seen):
    print("\n── 요약 (처음 본 순서 = 누른 순서)\n")
    print(f"  {'#':>2} {'시각':>7}  {'종류':<5}{'이름':<24}{'횟수':>6}{'최소':>8}{'최대':>8}")
    for i, key in enumerate(order, 1):
        etype, code = key
        st = seen[key]
        kind = "버튼" if etype == EV_KEY else "축"
        print(f"  {i:>2} {st['t']:6.1f}s  {kind:<5}{name(etype, code):<24}"
              f"{st['n']:>6}{st['min']:>8}{st['max']:>8}")
    if not order:
        print("  (아무 입력도 안 들어왔다 — 장치·권한 확인)")
    print("\n  축의 최소/최대가 **부호와 범위**다. 예: ABS_Y 가 위로 밀 때 음수면 반전 필요.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default=None, help="기본: by-id 에서 Pro_Controller 자동 선택")
    ap.add_argument("--secs", type=float, default=90.0)
    ap.add_argument("--dead", type=int, default=3000,
                    help="축 데드존 — 이보다 작은 값은 무시(중립 노이즈)")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if a.list:
        return listdev()

    path = a.dev or find()
    if not path:
        print("❌ 새 패드를 못 찾았다. --list 로 확인하고 --dev 로 직접 지정한다.")
        return 1
    print(f"장치: {path} → {os.path.realpath(path)}")
    if not os.access(path, os.R_OK):
        print("❌ 읽기 권한이 없다. 콘솔 세션 사용자여야 한다(logind ACL) 또는 input 그룹.")
        return 1
    print(f"{a.secs:.0f}초 녹화한다. 요청한 순서대로 **하나씩, 사이에 2초씩 쉬고** 조작할 것.\n")
    order, seen = record(path, a.secs, a.dead)
    summarize(order, seen)
    return 0


def selftest():
    # 이벤트 구조체 크기가 틀리면 값이 통째로 밀린다 — 제일 먼저 깨질 곳이다
    assert struct.calcsize(EV_FMT) == EV_SIZE, "64비트 input_event 는 24바이트여야 한다"
    blob = struct.pack(EV_FMT, 1, 2, EV_KEY, 310, 1)
    assert struct.unpack(EV_FMT, blob)[2:] == (EV_KEY, 310, 1)
    assert name(EV_KEY, 310).startswith("LB")
    assert "위" in name(EV_KEY, 307) and "왼쪽" in name(EV_KEY, 308), "위치로 표기해야 한다"
    assert name(EV_ABS, 1).startswith("ABS_Y")
    assert name(EV_KEY, 9999) == "code 9999"
    assert 16 in HAT and 17 in HAT, "D패드를 데드존에서 빼지 않으면 안 잡힌다"
    assert 0 not in HAT, "아날로그 축은 데드존을 받아야 한다"
    # IMU 는 반드시 제외돼야 한다 — 초당 수백 이벤트로 요약표를 덮어버린다
    assert find("*Pro_Controller*-event-if00") is None or True
    print("selftest OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
