#!/usr/bin/env python3
"""Crevis IO (MODBUS TCP) 입력 비트 탐색 — 읽기 전용. 버튼별 비트를 찾는다.

라이브러리 없이 순수 소켓으로 붙인다 (KU Polishing 의 crevis_io_driver 선례 —
납품 시 의존성 0). 레지스터 맵 근거: GN-9289 User Manual REV1.06 p.28
  입력 image REGISTER : 0x0000~  (func 3/4)
  출력 image BIT(coil): 0x1000~  (func 1/5/15)

  python3 crevis_probe.py                 # 20초간 변화 감시
  python3 crevis_probe.py --seconds 40
"""
import argparse
import socket
import struct
import sys
import time


class Modbus:
    def __init__(self, host, port=502, unit=1, timeout=1.0):
        self.unit, self.tid = unit, 0
        self.s = socket.create_connection((host, port), timeout)
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def _req(self, pdu):
        self.tid = (self.tid + 1) & 0xFFFF
        frame = struct.pack(">HHHB", self.tid, 0, len(pdu) + 1, self.unit) + pdu
        self.s.sendall(frame)
        hdr = self._recv(6)
        length = struct.unpack(">H", hdr[4:6])[0]
        body = self._recv(length)
        fn = body[1]
        if fn & 0x80:                       # 예외 응답
            raise IOError(f"modbus 예외 fn=0x{fn:02X} code={body[2]}")
        return body[2:]

    def _recv(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.s.recv(n - len(buf))
            if not chunk:
                raise IOError("연결이 끊겼다")
            buf += chunk
        return buf

    def read_words(self, fn, addr, count):
        """fn=3 holding / fn=4 input register"""
        data = self._req(struct.pack(">BHH", fn, addr, count))
        nbytes = data[0]
        return list(struct.unpack(">" + "H" * (nbytes // 2), data[1:1 + nbytes]))

    def read_bits(self, fn, addr, count):
        """fn=1 coil / fn=2 discrete input"""
        data = self._req(struct.pack(">BHH", fn, addr, count))
        nbytes = data[0]
        raw = data[1:1 + nbytes]
        return [(raw[i // 8] >> (i % 8)) & 1 for i in range(count)]


def fmt(words):
    return " ".join(f"{w:04X}" for w in words)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.100.100")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--unit", type=int, default=1)
    ap.add_argument("--count", type=int, default=4, help="읽을 워드 수")
    ap.add_argument("--seconds", type=float, default=20.0)
    a = ap.parse_args()

    m = Modbus(a.host, a.port, a.unit)

    # 어느 함수코드/주소가 응답하는지 먼저 확인한다
    print(f"=== {a.host}:{a.port} unit={a.unit} 접속 OK ===")
    works = []
    for label, fn, addr, kind in (("입력 REGISTER 0x0000 (fn4)", 4, 0x0000, "w"),
                                  ("입력 REGISTER 0x0000 (fn3)", 3, 0x0000, "w"),
                                  ("discrete input 0 (fn2)", 2, 0x0000, "b"),
                                  ("coil 0x1000 (fn1)", 1, 0x1000, "b")):
        try:
            if kind == "w":
                v = m.read_words(fn, addr, a.count)
                print(f"  {label:32} → {fmt(v)}")
            else:
                v = m.read_bits(fn, addr, 16)
                print(f"  {label:32} → {''.join(str(x) for x in v)}")
            works.append((label, fn, addr, kind))
        except Exception as e:
            print(f"  {label:32} → 실패: {e}")

    if not works:
        sys.exit("어느 방식으로도 못 읽었다 — unit id 나 주소를 확인할 것")

    label, fn, addr, kind = works[0]
    print(f"\n=== {label} 로 {a.seconds:.0f}초간 변화 감시 — 버튼을 하나씩 눌렀다 떼세요 ===")
    base = None
    end = time.time() + a.seconds
    seen = {}
    while time.time() < end:
        v = m.read_words(fn, addr, a.count) if kind == "w" else m.read_bits(fn, addr, 16)
        if base is None:
            base = v
            print(f"  기준(아무것도 안 누른 상태): {fmt(v) if kind == 'w' else ''.join(map(str, v))}")
        elif v != base:
            key = fmt(v) if kind == "w" else "".join(map(str, v))
            if key not in seen:
                seen[key] = 0
                # 기준과 달라진 비트를 뽑는다
                diff = []
                if kind == "w":
                    for i, (b, c) in enumerate(zip(base, v)):
                        x = b ^ c
                        for bit in range(16):
                            if x >> bit & 1:
                                diff.append(f"word{i}.bit{bit}({'ON' if c >> bit & 1 else 'OFF'})")
                else:
                    for i, (b, c) in enumerate(zip(base, v)):
                        if b != c:
                            diff.append(f"bit{i}({'ON' if c else 'OFF'})")
                print(f"  변화: {key}   ← {', '.join(diff)}")
            seen[key] += 1
        time.sleep(0.05)

    print(f"\n=== 요약: 서로 다른 상태 {len(seen)}종 관측 ===")
    for k, n in sorted(seen.items(), key=lambda kv: -kv[1]):
        print(f"  {k}  ({n}회)")


if __name__ == "__main__":
    main()
