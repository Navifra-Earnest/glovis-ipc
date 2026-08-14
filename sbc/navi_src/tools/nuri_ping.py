#!/usr/bin/env python3
"""누리로봇 SB60(SA 계열) RS485 연결 확인 — Ping + 현재 설정 읽기.

체인 전체(DE 타이밍 / 보레이트 / ID / 체크섬 / 파싱)를 한 번에 검증한다.
아무 응답이 없으면 제일 먼저 A/B(D+/D-) 선을 바꿔 꽂아 볼 것.

  python3 nuri_ping.py              # 9600, ID 0
  python3 nuri_ping.py --scan       # 보레이트 × ID 스캔
"""
import argparse
import struct
import subprocess
import sys
import time

import serial

HEADER = b"\xff\xfe"


def _detect_de():
    """40핀 12번(RS485_DE)의 gpiochip/line 을 보드에서 직접 찾는다.

    🔴 하드코딩하면 안 된다. 보드마다 다르다:
        ROCK 3A (RK3568)  gpiochip3 line 3   (GPIO3_A3)
        ROCK 5A (RK3588S) gpiochip4 line 1   (GPIO4_A1)
    3A 값을 박아둔 채로 5A 에서 돌렸다가, DE 가 안 올라가 송신이 버스에 안 나가서
    "전 보레이트 무응답"이 났다. 배선 문제로 한참 헤맸다 (2026-08-07).

    gpiochip 번호는 커널 프로브 순서를 따르므로 뱅크 번호와 같다는 보장도 없다.
    그래서 계산하지 말고 라인 이름("PIN_12")으로 찾는다.
    """
    try:
        out = subprocess.run(["gpiofind", "PIN_12"], capture_output=True, text=True, timeout=5)
        if out.returncode == 0 and out.stdout.split():
            chip, line = out.stdout.split()[:2]
            return chip, int(line)
    except Exception:
        pass
    return "gpiochip3", 3          # gpiofind 가 없을 때의 최후 기본값 (3A)


DE_CHIP, DE_LINE = _detect_de()    # --de-chip / --de-line 으로 덮어쓸 수 있다

REQ = {  # 요청 → 기대 응답 (SA-RS485_V1.0.2, p.24~26)
    "ping":     (0xA0, 0xD0),
    "firmware": (0xCD, 0xFD),
    "ratio":    (0xA6, 0xD6),
    "onoff":    (0xA7, 0xD7),
    "posmode":  (0xA8, 0xD8),
    "pos":      (0xA1, 0xD1),
}
BAUD_CODES = {110: 0x00, 300: 0x01, 600: 0x02, 1200: 0x03, 2400: 0x04, 4800: 0x05,
              9600: 0x06, 14400: 0x07, 19200: 0x08, 28800: 0x09, 38400: 0x0A,
              57600: 0x0B, 76800: 0x0C, 115200: 0x0D, 230400: 0x0E, 250000: 0x0F,
              500000: 0x10, 1000000: 0x11}


def checksum(dev_id, size, mode, values=b""):
    """~((ID + DataSize + Mode + Values) & 0xFF) — Header와 CS 자신은 제외."""
    return (~(dev_id + size + mode + sum(values))) & 0xFF


def frame(dev_id, mode, values=b""):
    size = 2 + len(values)  # CS(1) + Mode(1) + Values
    return HEADER + bytes((dev_id, size, checksum(dev_id, size, mode, values), mode)) + values


class Bus:
    """반이중 RS485.

    DE(U5의 DE+RE#)는 UART의 RTS가 아니라 40핀 12번 = GPIO3_A3에 직결돼 있다.
    커널 RS485 모드(TIOCSRS485)는 RTS 핀을 토글하므로 이 보드에선 효과가 없다 → GPIO를 직접 제어한다.
    """

    def __init__(self, port, baud, timeout=0.3, guard_us=200.0):
        import gpiod
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.baud = baud
        self.bit_time = 1.0 / baud
        # 전송 완료 후 DE를 내리기까지의 여유 (µs). 실측 근거:
        #   flush(tcdrain)은 실제 전송 완료보다 3~5ms 늦게 리턴 → 응답 앞부분을 통째로 놓침
        #   TIOCOUTQ=0은 FIFO로 넘어간 시점일 뿐 → 너무 일러서 송신이 잘림
        # 그래서 둘 다 쓰지 않고 전송 시작 시각 + 이론 전송시간으로 직접 계산한다.
        self.guard = guard_us / 1e6
        chip = gpiod.Chip(DE_CHIP)
        self.line = chip.get_line(DE_LINE)
        self.line.request(consumer="nuri", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        self.mode = f"gpio {DE_CHIP}:{DE_LINE}, guard {guard_us:.0f}µs"

    def send(self, pkt):
        """DE를 올려 송신하고, 전송이 끝나는 시점에 맞춰 정확히 내린다."""
        self.ser.reset_input_buffer()
        self.line.set_value(1)                      # DE=H → 송신
        t0 = time.perf_counter()
        self.ser.write(pkt)
        end = t0 + len(pkt) * 10 / self.baud + self.guard   # 10비트/바이트 (8N1)
        while time.perf_counter() < end:            # busy-wait: sleep은 이 정밀도가 안 나온다
            pass
        self.line.set_value(0)                      # DE=L → 수신

    def xfer(self, pkt, expect_mode):
        self.send(pkt)
        return self.read_frame(expect_mode)

    def read_frame(self, expect_mode, deadline=0.4):
        """헤더 탐색 → DataSize로 프레임 완성 → 체크섬 검증."""
        buf = bytearray()
        end = time.monotonic() + deadline
        while time.monotonic() < end:
            chunk = self.ser.read(64)
            if chunk:
                buf += chunk
            i = buf.find(HEADER)
            if i < 0 or len(buf) < i + 4:
                continue
            dev_id, size = buf[i + 2], buf[i + 3]
            total = i + 4 + size          # 헤더(2) + ID(1) + Size(1) + [CS + Mode + Values]
            if len(buf) < total:
                continue
            cs, mode = buf[i + 4], buf[i + 5]
            values = bytes(buf[i + 6:total])
            if cs != checksum(dev_id, size, mode, values):
                # 펌웨어 버전 응답(0xFD)만 실측상 체크섬이 어긋난다. 모터가 버전 바이트를
                # 합에서 빼고 계산하는 듯 — CS를 Values 없이 다시 맞춰보고 그것도 아니면 실패.
                if mode == 0xFD and cs == checksum(dev_id, size, mode):
                    return (dev_id, mode, values), None
                return None, f"체크섬 불일치 (수신 0x{cs:02X}) raw={buf[i:total].hex(' ')}"
            if mode != expect_mode:
                return None, f"Mode 0x{mode:02X} (0x{expect_mode:02X} 기대) raw={buf[i:total].hex(' ')}"
            return (dev_id, mode, values), None
        return None, ("무응답" if not buf else f"불완전 raw={bytes(buf).hex(' ')}")

    def close(self):
        self.line.set_value(0)
        self.line.release()
        self.ser.close()


def ping(bus, dev_id):
    req, resp = REQ["ping"]
    got, err = bus.xfer(frame(dev_id, req), resp)
    return got is not None, err


def scan(port):
    print("보레이트 × ID 스캔 (응답 있는 조합을 찾는다)\n")
    for baud in (9600, 115200, 57600, 38400, 19200, 230400, 4800):
        bus = Bus(port, baud)
        hits, noise = [], []
        for i in range(0, 16):
            ok, err = ping(bus, i)
            if ok:
                hits.append(i)
            elif err and "무응답" not in err:
                noise.append(f"ID{i}:{err.split('raw=')[-1]}")
        bus.close()
        note = ", ".join(f"ID {i}" for i in hits) if hits else "-"
        if noise:
            note += f"   [깨진 수신: {'; '.join(noise[:4])}]"
        print(f"  {baud:>7} bps : {note}")
        if hits:
            return baud, hits[0]
    return None, None


def raw_probe(port, seconds=2.0):
    """진단용. 보레이트별로 Ping을 쏘고 돌아온 바이트를 가공 없이 전부 보여준다.

    - 어느 보레이트에서도 같은 쓰레기가 나오면 → A/B 극성 반대일 가능성이 큼
    - 특정 보레이트에서만 0xFFFE로 시작하는 게 보이면 → 그 보레이트가 정답
    """
    print("① 수동 수신 (모터가 스스로 보내는 게 있는지)\n")
    for baud in (9600, 115200):
        bus = Bus(port, baud)
        time.sleep(seconds)
        data = bus.ser.read(256)
        bus.close()
        print(f"  {baud:>7} bps : {data.hex(' ') if data else '(없음)'}")

    print("\n② Ping 송신 후 raw 수신\n")
    for baud in (9600, 19200, 38400, 57600, 76800, 115200, 230400, 250000, 500000,
                 4800, 2400, 1200, 600, 300):
        bus = Bus(port, baud)
        got = []
        for dev_id in (0x00, 0xFF):          # 0xFF = Broadcast
            bus.ser.reset_input_buffer()
            bus.line.set_value(1)
            bus.ser.write(frame(dev_id, REQ["ping"][0]))
            bus.ser.flush()
            time.sleep(bus.bit_time * 12)
            bus.line.set_value(0)
            time.sleep(0.25)
            d = bus.ser.read(256)
            if d:
                got.append(f"ID{dev_id:02X}→{d.hex(' ')}")
        bus.close()
        mark = "  ← 헤더 FFFE 발견!" if any("ff fe" in g for g in got) else ""
        print(f"  {baud:>7} bps : {'  |  '.join(got) if got else '(무응답)'}{mark}")


def set_ratio(port, baud, dev_id, ratio, guard_us=200.0):
    """외부 감속비를 EEPROM에 쓴다 (Mode 0x09, 0.1 단위).

    감속비를 넣으면 위치 명령·피드백이 모터축이 아니라 **출력축** 기준이 된다.
    문서 요구대로 제어를 끄고 → 쓰고 → 다시 켜고 → 읽어서 검증한다.
    """
    raw = int(round(ratio * 10))
    bus = Bus(port, baud, guard_us=guard_us)
    steps = [
        ("제어 Off", frame(dev_id, 0x0A, b"\x01"), 0.4),
        (f"감속비 {ratio}:1 쓰기", frame(dev_id, 0x09, struct.pack(">H", raw)), 0.4),
        ("제어 On", frame(dev_id, 0x0A, b"\x00"), 0.4),
    ]
    for label, pkt, wait in steps:
        bus.send(pkt)                 # 설정 명령은 응답이 없다
        print(f"  {label:18} TX {pkt.hex(' ')}")
        time.sleep(wait)              # EEPROM 기록 50~300ms

    got, err = bus.xfer(frame(dev_id, REQ["ratio"][0]), REQ["ratio"][1])
    bus.close()
    if not got:
        print(f"\n  검증 실패: {err}")
        return False
    now = struct.unpack(">H", got[2])[0]
    ok = now == raw
    print(f"\n  검증: {now/10:.1f}:1 (0x{now:04X})  {'✅ 반영됨' if ok else '❌ 안 바뀜'}")
    return ok


def set_id(port, baud, cur_id, new_id, guard_us=200.0):
    """슬레이브 ID를 EEPROM에 쓴다 (Mode 0x06).

    ⚠ 여러 대를 한 버스에 물린 뒤에는 쓰면 안 된다 — 같은 ID를 가진 모두가 함께 바뀐다.
      반드시 1대씩 연결한 상태에서 부여할 것. 출고 기본은 전부 0이다.
    """
    if not (0 <= new_id <= 0xFE):
        print(f"ID 범위는 0~254 (0xFF는 Broadcast 전용)")
        return False

    bus = Bus(port, baud, guard_us=guard_us)
    ok, err = ping(bus, cur_id)
    if not ok:
        bus.close()
        print(f"현재 ID {cur_id} 에서 응답이 없다: {err}")
        return False
    print(f"  ID {cur_id} Ping OK")

    # 같은 버스에 다른 장치가 있으면 사고다. 새 ID가 이미 쓰이는지 먼저 본다.
    if new_id != cur_id and ping(bus, new_id)[0]:
        bus.close()
        print(f"  ❌ ID {new_id} 를 쓰는 장치가 이미 있다 — 중단")
        return False

    pkt = frame(cur_id, 0x06, bytes([new_id]))
    bus.send(pkt)
    print(f"  ID {cur_id} → {new_id} 쓰기  TX {pkt.hex(' ')}")
    time.sleep(0.6)          # EEPROM 기록

    ok, err = ping(bus, new_id)
    if ok:
        got, _ = bus.xfer(frame(new_id, REQ["ratio"][0]), REQ["ratio"][1])
        r = struct.unpack(">H", got[2])[0] if got else 0
        print(f"\n  ✅ ID {new_id} 에서 Ping OK  (감속비 {r/10:.1f}:1 로 검증)")
    else:
        print(f"\n  ❌ ID {new_id} 응답 없음: {err}")
        old_ok, _ = ping(bus, cur_id)
        print(f"     이전 ID {cur_id} 는 {'아직 응답함 — 변경 실패' if old_ok else '응답 없음'}")
    bus.close()
    return ok


def set_baud(port, cur_baud, dev_id, new_baud, guard_us=200.0):
    """통신속도를 EEPROM에 쓴다 (Mode 0x07).

    쓰는 순간부터 모터가 새 속도로 말하므로, 마스터도 포트를 다시 열어야 한다.
    설정 명령은 응답이 없어서 성공 여부는 새 속도로 Ping 해봐야 안다.
    """
    if new_baud not in BAUD_CODES:
        print(f"지원하지 않는 속도: {new_baud}  (가능: {sorted(BAUD_CODES)})")
        return False
    code = BAUD_CODES[new_baud]

    bus = Bus(port, cur_baud, guard_us=guard_us)
    ok, err = ping(bus, dev_id)
    if not ok:
        bus.close()
        print(f"현재 속도 {cur_baud}에서 응답이 없다: {err}")
        return False
    print(f"  현재 {cur_baud} bps 에서 Ping OK")

    pkt = frame(dev_id, 0x07, bytes([code]))
    bus.send(pkt)
    print(f"  통신속도 {new_baud} (코드 0x{code:02X}) 쓰기  TX {pkt.hex(' ')}")
    bus.close()
    time.sleep(0.6)          # EEPROM 기록 50~300ms

    # 새 속도로 다시 열어 확인한다
    bus = Bus(port, new_baud, guard_us=guard_us)
    ok, err = ping(bus, dev_id)
    if ok:
        got, _ = bus.xfer(frame(dev_id, REQ["ratio"][0]), REQ["ratio"][1])
        r = struct.unpack(">H", got[2])[0] if got else 0
        print(f"\n  ✅ {new_baud} bps 에서 Ping OK  (감속비 {r/10:.1f}:1 로 검증)")
    else:
        print(f"\n  ❌ {new_baud} bps 에서 응답 없음: {err}")
        print(f"     이전 속도({cur_baud})로 되돌리려면 그 속도로 다시 --set-baud 하거나,")
        print(f"     --scan 으로 실제 속도를 찾을 것")
    bus.close()
    return ok


def timing(port, baud, dev_id):
    """DE 해제가 왜 늦는지 구간별로 잰다.

    tcdrain(flush)이 실제 전송 완료보다 늦게 리턴하면 그만큼 응답 앞부분을 놓친다.
    TIOCOUTQ 폴링이 대안이 되는지도 같이 본다.
    """
    import fcntl
    import termios

    pkt = frame(dev_id, REQ["ping"][0])
    ideal = len(pkt) * 10 / baud
    print(f"송신 {len(pkt)}바이트 @ {baud}bps → 이론 전송시간 {ideal*1000:.2f} ms\n")

    for label in ("tcdrain", "TIOCOUTQ"):
        bus = Bus(port, baud)
        fd = bus.ser.fileno()
        spans = []
        for _ in range(3):
            bus.ser.reset_input_buffer()
            bus.line.set_value(1)
            t0 = time.perf_counter()
            bus.ser.write(pkt)
            t1 = time.perf_counter()
            if label == "tcdrain":
                bus.ser.flush()
            else:
                while struct.unpack("I", fcntl.ioctl(fd, termios.TIOCOUTQ, b"\0" * 4))[0]:
                    pass
                time.sleep(10 / baud)   # FIFO+시프트 레지스터의 마지막 1바이트
            t2 = time.perf_counter()
            bus.line.set_value(0)
            t3 = time.perf_counter()
            time.sleep(0.2)
            got = bus.ser.read(64)
            spans.append((t1 - t0, t2 - t1, t3 - t2, got))
        bus.close()
        print(f"[{label}]")
        for w, d, g, got in spans:
            total = (w + d + g) * 1000
            print(f"  write {w*1000:6.2f} + 대기 {d*1000:6.2f} + DE토글 {g*1000:6.2f} = {total:6.2f} ms"
                  f"  (초과 {total - ideal*1000:+6.2f})  rx={got.hex(' ') if got else '-'}"
                  + ("  ✅" if got.startswith(HEADER) else ""))
        print()


def de_sweep(port, baud, dev_id):
    """DE 해제 여유(guard)를 바꿔가며 응답이 온전히 잡히는 구간을 찾는다.

    너무 크면 응답 앞부분을 놓치고(모터 응답 지연이 100µs뿐), 너무 작으면 송신 마지막 바이트가 잘린다.
    """
    print(f"DE 해제 여유 스윕 — {baud} bps, ID {dev_id}\n")
    good = []
    for guard in (0, 50, 100, 150, 200, 300, 500, 800, 1200):
        bus = Bus(port, baud, guard_us=guard)
        results = []
        for _ in range(5):
            bus.send(frame(dev_id, REQ["ping"][0]))
            time.sleep(0.15)
            results.append(bus.ser.read(64))
            time.sleep(0.05)
        bus.close()
        ok = sum(1 for r in results if r.startswith(HEADER))
        shown = " | ".join(r.hex(' ') if r else "-" for r in results[:3])
        print(f"  guard {guard:>5}µs : [{ok}/5 온전]  {shown}")
        if ok == 5:
            good.append(guard)
    if good:
        mid = good[len(good) // 2]
        print(f"\n→ 5/5 성공: {good} µs.  가운데 값 {mid}µs 를 기본으로 쓰세요.")
        return mid
    print("\n→ 온전한 응답을 못 받았습니다. 모터의 통신 응답시간(Mode 0x08)을 늘리는 쪽을 검토하세요.")
    return None


def selftest():
    """문서의 예제 패킷과 바이트 단위로 일치하는지 확인 (하드웨어 불필요)."""
    # p.23 SA 예제: ID0, CCW 180.00°, 5.0RPM
    assert frame(0, 0x01, bytes.fromhex("00 4650 0032".replace(" ", ""))) == \
        bytes.fromhex("fffe00072f0100465000 32".replace(" ", "")), "위치·속도제어 불일치"
    # p.24 SA 예제: ID0 외부 감속비 2:1
    assert frame(0, 0x09, struct.pack(">H", 20)) == bytes.fromhex("fffe0004de09 0014".replace(" ", "")), "감속비 불일치"
    # 본문 3.2 예제: 위치 피드백 요청
    assert frame(0, 0xA1) == bytes.fromhex("fffe00025ca1"), "요청 프레임 불일치"
    # 본문 3.2 응답 파싱: FF FE 00 08 26 D1 00*6
    rx = bytes.fromhex("fffe000826d1000000000000")
    assert checksum(rx[2], rx[3], rx[5], rx[6:]) == rx[4], "응답 체크섬 불일치"
    print("selftest OK — 문서 예제 4건 일치")


def main():
    global DE_CHIP, DE_LINE      # --de-chip / --de-line 으로 덮어쓴다
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS2")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--id", type=lambda x: int(x, 0), default=0)
    ap.add_argument("--scan", action="store_true")
    ap.add_argument("--selftest", action="store_true", help="문서 예제로 프레임 생성/파싱 검증 (하드웨어 불필요)")
    ap.add_argument("--raw", action="store_true", help="진단: 보레이트별 raw 수신 덤프")
    ap.add_argument("--de-sweep", action="store_true", help="진단: DE 해제 지연 스윕")
    ap.add_argument("--timing", action="store_true", help="진단: 송신/DE 구간별 소요시간 측정")
    # DE 해제 여유는 보레이트마다 안전 구간이 다르다 (실측):
    #     9600 → 100~500µs 가 5/5 (200 사용)
    #   115200 → 300µs 만 5/5, 200은 4/5로 가끔 놓친다
    # 지정하지 않으면 보레이트를 보고 고른다. --de-sweep 으로 언제든 다시 잴 수 있다.
    ap.add_argument("--guard", type=float, default=None, help="DE 해제 여유 (µs)")
    ap.add_argument("--de-chip", default=None,
                    help=f"RS485_DE 의 gpiochip (기본: 자동탐지 = {DE_CHIP})")
    ap.add_argument("--de-line", type=int, default=None,
                    help=f"RS485_DE 의 line (기본: 자동탐지 = {DE_LINE})")
    ap.add_argument("--set-ratio", type=float, metavar="N",
                    help="외부 감속비를 N:1 로 설정 (EEPROM 기록). 예: --set-ratio 16")
    ap.add_argument("--set-baud", type=int, metavar="BPS",
                    help="통신속도 변경 (EEPROM 기록, 즉시 적용). 예: --set-baud 115200")
    ap.add_argument("--set-id", type=lambda x: int(x, 0), metavar="N",
                    help="슬레이브 ID 변경 (EEPROM). ⚠ 1대만 연결한 상태에서 할 것")
    a = ap.parse_args()
    if a.guard is None:
        a.guard = 300.0 if a.baud >= 57600 else 200.0

    if a.de_chip: DE_CHIP = a.de_chip
    if a.de_line is not None: DE_LINE = a.de_line
    print(f"DE: {DE_CHIP}:{DE_LINE}")

    if a.de_sweep:
        de_sweep(a.port, a.baud, a.id)
        return

    if a.timing:
        timing(a.port, a.baud, a.id)
        return

    if a.set_ratio is not None:
        print(f"외부 감속비 설정 — {a.baud} bps, ID {a.id}\n")
        sys.exit(0 if set_ratio(a.port, a.baud, a.id, a.set_ratio, a.guard) else 1)

    if a.set_id is not None:
        print(f"ID 변경 — {a.baud} bps, {a.id} → {a.set_id}\n")
        sys.exit(0 if set_id(a.port, a.baud, a.id, a.set_id, a.guard) else 1)

    if a.set_baud is not None:
        print(f"통신속도 변경 — 현재 {a.baud} → {a.set_baud} bps, ID {a.id}\n")
        sys.exit(0 if set_baud(a.port, a.baud, a.id, a.set_baud, a.guard) else 1)

    if a.selftest:
        selftest()
        return

    if a.raw:
        raw_probe(a.port)
        return

    if a.scan:
        baud, dev_id = scan(a.port)
        if baud is None:
            sys.exit("\n어느 조합에서도 응답이 없습니다.\n"
                     "→ 1) A/B(D+/D-) 선을 바꿔 꽂아 보세요  2) 모터 전원  3) GND 공통 연결")
        print(f"\n찾음: {baud} bps, ID {dev_id}\n")
        a.baud, a.id = baud, dev_id

    bus = Bus(a.port, a.baud, guard_us=a.guard)
    print(f"{a.port} @ {a.baud} bps, ID {a.id}, DE={bus.mode}\n")

    ok, err = ping(bus, a.id)
    pkt = frame(a.id, REQ["ping"][0])
    print(f"  Ping  TX {pkt.hex(' ')}  →  {'응답 OK' if ok else err}")
    if not ok:
        bus.close()
        sys.exit("\n응답이 없습니다. 순서대로 확인하세요:\n"
                 "  1) A/B(D+/D-) 극성 — 바꿔 꽂는 게 제일 흔한 원인\n"
                 "  2) 모터 전원 / GND 공통\n"
                 "  3) --scan 으로 보레이트·ID 탐색")

    for name in ("firmware", "ratio", "onoff", "posmode", "pos"):
        req, resp = REQ[name]
        got, err = bus.xfer(frame(a.id, req), resp)
        time.sleep(0.02)  # 연속 요청 10ms 이상 간격
        if not got:
            print(f"  {name:9} {err}")
            continue
        v = got[2]
        if name == "firmware":
            print(f"  {name:9} v{v[0]}")
        elif name == "ratio":
            r = struct.unpack(">H", v)[0]
            print(f"  {name:9} {r/10:.1f}:1  (raw 0x{r:04X})"
                  + ("   ← 1/16 감속기면 160(0x00A0)이어야 함" if r != 160 else "   ✅ 1/16 반영됨"))
        elif name == "onoff":
            print(f"  {name:9} {'On' if v[0] == 0 else 'Off'}")
        elif name == "posmode":
            print(f"  {name:9} {'절대' if v[0] == 0 else '상대'}")
        elif name == "pos":
            d, pos, spd, cur = v[0], *struct.unpack(">HH", v[1:5]), v[5]
            print(f"  {name:9} {'CW' if d else 'CCW'}  {pos/100:.2f}°  {spd/10:.1f} RPM  {cur/10:.1f} A")
    bus.close()


if __name__ == "__main__":
    main()
