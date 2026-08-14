#!/usr/bin/env python3
"""ipc_monitor.py — 상위(IPC) 역할로 로봇 브로커에 붙어 데이터를 받는다.

개발 PC 에서 돌린다. 로봇 연동을 붙이기 전에 토픽 규격이 실제로 맞는지,
데이터가 제대로 오는지 확인하는 용도다.

    ./ipc_monitor.py                        대시보드 (수신 전용)
    ./ipc_monitor.py --host 192.168.50.1    AP(EV-DL_AP) 로 붙었을 때
    ./ipc_monitor.py --raw                  들어오는 메시지를 그대로 흘려 본다
    ./ipc_monitor.py --save-frames ./frames 받은 프레임을 파일로 남긴다
    ./ipc_monitor.py --stream on            프레임 전송을 켜고 대시보드 진입

받는 것 — navi/state · navi/event · navi/alarm/<key> · navi/frame/<key> · navi/state/online

의존: pip install paho-mqtt
"""
import argparse
import json
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

try:
    import paho.mqtt.client as mqtt
except ImportError:
    sys.exit("paho-mqtt 가 없다:  pip install paho-mqtt")

DEFAULT_HOST = "192.168.0.64"   # 유선. AP 로 붙으면 192.168.50.1
DEFAULT_PORT = 1883
PREFIX = "navi"
STALE_SEC = 5.0                 # 이보다 오래 state 가 없으면 끊긴 것으로 본다
                                # (로봇 heartbeat 가 2초라 그보다 넉넉해야 한다)

# ANSI — 깜빡임 없이 갱신한다
CLS, HOME, HIDE, SHOW = "\033[2J", "\033[H", "\033[?25l", "\033[?25h"
def c(code, s): return f"\033[{code}m{s}\033[0m"
RED, GRN, YEL, CYA, DIM, BLD = "31", "32", "33", "36", "2", "1"


class Monitor:
    def __init__(self, args):
        self.prefix, self.raw = args.prefix, args.raw
        self.host, self.port = args.host, args.port
        self.save_dir = Path(args.save_frames) if args.save_frames else None
        if self.save_dir:
            self.save_dir.mkdir(parents=True, exist_ok=True)
        self.stream = args.stream

        self.state = {}
        self.online = None
        self.alarms = {}                     # key → payload
        self.events = deque(maxlen=6)
        self.frames = {}                     # key → [건수, 총 바이트, 마지막 시각]
        self.state_times = deque(maxlen=20)  # 수신 Hz 계산용
        self.connected = False
        self.last_err = ""

        self.cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.cl.on_connect = self._on_connect
        self.cl.on_disconnect = self._on_disconnect
        self.cl.on_message = self._on_message

    # ── MQTT ────────────────────────────────────────────────────────
    def _on_connect(self, cl, _u, _f, rc, _p=None):
        self.connected = (rc == 0 or str(rc) == "Success")
        # retain 알람이 접속 즉시 몰려온다 — 현재 상태를 바로 알 수 있다
        cl.subscribe(f"{self.prefix}/#", qos=1)
        if self.stream:
            on = self.stream == "on"
            cl.publish(f"{self.prefix}/cmd/stream", json.dumps({"on": on}), qos=1)

    def _on_disconnect(self, _cl, _u, _f, rc=None, _p=None):
        self.connected = False

    def _on_message(self, _cl, _u, m):
        sub = m.topic[len(self.prefix) + 1:] if m.topic.startswith(self.prefix + "/") else m.topic
        if self.raw:
            body = m.payload[:160].decode("utf-8", "replace") if len(m.payload) < 400 \
                   else f"<{len(m.payload)} bytes>"
            print(f"{datetime.now():%H:%M:%S.%f}"[:-3], c(CYA, sub), body, flush=True)
            return

        if sub == "state":
            try:
                self.state = json.loads(m.payload)
                self.state_times.append(time.time())
            except json.JSONDecodeError as e:
                self.last_err = f"state 파싱 실패: {e}"
        elif sub == "state/online":
            try:
                self.online = json.loads(m.payload).get("online")
            except json.JSONDecodeError:
                pass
        elif sub == "event":
            try:
                self.events.append((datetime.now(), json.loads(m.payload)))
            except json.JSONDecodeError:
                pass
        elif sub.startswith("alarm/"):
            try:
                self.alarms[sub[6:]] = json.loads(m.payload)
            except json.JSONDecodeError:
                pass
        elif sub.startswith("frame/"):
            key = sub[6:]
            f = self.frames.setdefault(key, [0, 0, 0.0])
            f[0] += 1; f[1] += len(m.payload); f[2] = time.time()
            if self.save_dir:
                # 열화상은 BMP, tof 는 JSON — 앞 2바이트로 가른다
                ext = "bmp" if m.payload[:2] == b"BM" else "json"
                (self.save_dir / f"{key}_{f[0]:05d}.{ext}").write_bytes(m.payload)

    # ── 화면 ────────────────────────────────────────────────────────
    def _hz(self):
        t = list(self.state_times)
        if len(t) < 2:
            return 0.0
        span = t[-1] - t[0]
        return (len(t) - 1) / span if span > 0 else 0.0

    def render(self):
        s, out = self.state, []
        w = out.append

        link = c(GRN, "연결됨") if self.connected else c(RED, "끊김")
        on = "—" if self.online is None else (c(GRN, "online") if self.online else c(RED, "offline"))
        age = time.time() - self.state_times[-1] if self.state_times else None
        # 로봇은 상태가 바뀔 때만 보내고, 안 바뀌면 2초 heartbeat 로만 보낸다.
        # 그래서 정지 중에는 1Hz 안팎이 정상이다 — 5Hz 가 안 나온다고 이상한 게 아니다.
        # 끊김 판정은 heartbeat(2초)보다 넉넉히 잡는다.
        fresh = c(DIM, "수신 없음") if age is None else \
                (f"{self._hz():.1f} Hz" if age < STALE_SEC else c(RED, f"{age:.0f}초째 끊김"))
        w(c(BLD, f"IPC 모니터  {self.host}:{self.port}  ") + f"{link} · {on} · {fresh}")
        w(c(DIM, "상태는 변화가 있을 때 발행한다 (최대 5Hz) · 무변화 시 2초 heartbeat"))
        w("─" * 74)

        if not s:
            w(c(DIM, "  상태 수신 대기 중…"))
            return "\n".join(out)

        # 구동
        ok = s.get("drive_ok")
        head = c(GRN, f"정상 {s.get('wheels_alive', 0)}축") if ok else c(RED, "구동계 없음")
        w(f"{c(BLD,'구동'):<14} {head}")
        if not ok and s.get("drive_error"):
            w(f"{'':<14} {c(DIM, s['drive_error'][:58])}")
        for wh in s.get("wheels", []):
            w(f"{'':<14} {wh.get('label','?'):<4} {wh.get('rpm',0):>7.1f} RPM  "
              f"{wh.get('amp',0):>5.2f} A  {c(DIM, wh.get('state',''))}")

        # 정지 상태
        flags = []
        if s.get("estop"):
            flags.append(c(RED, f"E-STOP: {s.get('estop_reason','')[:44]}"))
        if s.get("watchdog"):
            flags.append(c(YEL, "워치독 트립"))
        if flags:
            w(f"{c(BLD,'정지'):<14} " + " · ".join(flags))

        # 액추에이터
        a = s.get("actuator", {})
        if a.get("present"):
            w(f"{c(BLD,'액추에이터'):<14} {a.get('state','?')}  위치 {a.get('position',0)}  "
              f"{a.get('current',0):.2f} A")

        # 카메라
        for cam in s.get("cams", []):
            st = f"{cam.get('fps',0):.1f} fps  drops {cam.get('drops',0)}" \
                 if cam.get("open") else c(RED, (cam.get("error") or "닫힘")[:40])
            w(f"{c(BLD,'카메라'):<14} {cam.get('key','?'):<8} {st}")

        # 열화상
        t = s.get("thermal", {})
        if t.get("present"):
            w(f"{c(BLD,'열화상'):<14} {t.get('lo',0):.1f} ~ {t.get('hi',0):.1f}℃  "
              f"중앙 {t.get('center',0):.1f}℃  {t.get('fps',0):.1f} fps")

        # ToF
        f = s.get("tof", {})
        if f.get("present"):
            d = f"{f.get('dist_cm',0)} cm" if f.get("valid") else c(YEL, "신뢰도 낮음")
            w(f"{c(BLD,'ToF'):<14} {d}  강도 {f.get('strength',0)}  {f.get('fps',0):.1f} fps")

        w(f"{c(BLD,'tick'):<14} {s.get('tick_ms',0):.1f} / 최대 {s.get('tick_max_ms',0):.1f} ms"
          f"   운동학 {'열림' if s.get('kinematics') else c(DIM,'미설정')}")

        # 알람 — active 만 보이고 나머지는 개수로
        act = [(k, v) for k, v in sorted(self.alarms.items()) if v.get("active")]
        w("─" * 74)
        if act:
            for k, v in act:
                col = RED if v.get("severity") == "critical" else YEL
                w(f"{c(col, '● ' + k):<26} {v.get('message','')[:46]}")
        else:
            w(c(GRN, "● 활성 알람 없음") + c(DIM, f"  (수신 {len(self.alarms)}종)"))

        if self.frames:
            w("─" * 74)
            for k, (n, tot, last) in sorted(self.frames.items()):
                ago = time.time() - last
                w(f"{c(BLD,'프레임'):<14} {k:<8} {n}건  {tot/1024:.0f} KB  "
                  + (c(DIM, f"{ago:.0f}초 전") if ago > 2 else "수신 중"))

        if self.events:
            w("─" * 74)
            for ts, e in self.events:
                w(c(DIM, f"{ts:%H:%M:%S} ") + f"{e.get('event','?')}: {e.get('detail','')[:50]}")

        if self.last_err:
            w(c(RED, f"⚠ {self.last_err}"))
        return "\n".join(out)

    def run(self, seconds=None):
        try:
            self.cl.connect(self.host, self.port, keepalive=15)
        except OSError as e:
            sys.exit(f"접속 실패 {self.host}:{self.port} — {e}\n"
                     f"  · 보드가 켜져 있는지, 같은 망에 있는지 확인\n"
                     f"  · AP(EV-DL_AP) 로 붙었으면 --host 192.168.50.1")
        self.cl.loop_start()
        end = time.time() + seconds if seconds else None
        try:
            if self.raw:
                print(c(DIM, f"raw 모드 — {self.prefix}/# 구독. Ctrl+C 로 종료"))
                while not end or time.time() < end:
                    time.sleep(0.2)
                return
            print(HIDE + CLS, end="")
            while not end or time.time() < end:
                print(HOME + CLS + self.render(), end="", flush=True)
                time.sleep(0.3)
        finally:
            if not self.raw:
                print(SHOW, end="")
            self.cl.loop_stop()
            self.cl.disconnect()


def main():
    ap = argparse.ArgumentParser(description="IPC 역할 MQTT 수신 모니터")
    ap.add_argument("--host", default=DEFAULT_HOST, help=f"브로커 주소 (기본 {DEFAULT_HOST})")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--prefix", default=PREFIX)
    ap.add_argument("--raw", action="store_true", help="파싱 없이 그대로 출력")
    ap.add_argument("--save-frames", metavar="DIR", help="받은 프레임을 파일로 저장")
    ap.add_argument("--stream", choices=["on", "off"], help="접속 시 프레임 전송을 켜거나 끈다")
    ap.add_argument("--seconds", type=float, help="이 시간만 돌고 종료 (자동 점검용)")
    args = ap.parse_args()
    try:
        Monitor(args).run(args.seconds)
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
