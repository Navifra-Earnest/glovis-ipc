#!/usr/bin/env python3
"""브로커 이중 경로 — **유선 우선, 무선 폴백**. IPC 세 서비스가 공유한다.

브로커는 로봇에 있고 `listener 1883 0.0.0.0` 이라 유선·무선 주소 **양쪽으로 같은
브로커**가 열려 있다. 즉 경로 전환은 전송 경로만 바뀌는 것이고 상태가 갈라지지 않는다
(로봇 쪽은 손댈 게 없다).

  python3 mqtt_link.py --selftest      # 선택 로직 검증 (네트워크 불필요)

설계 이유 두 가지:

1. **tick() 은 절대 블로킹하지 않는다.** 제어 루프(50 ms)에서 불리기 때문이다.
   경로 탐색은 백그라운드 스레드가 하고, 접속은 `connect_async` 로 던진다.
   여기서 0.3 초만 멈춰도 350 ms 케이던스가 깨져 로봇 워치독(500 ms)이 걸린다.
2. **경로를 바꿀 때는 클라이언트를 새로 만든다.** paho 1.6 에서 접속된 클라이언트의
   호스트를 갈아끼우면 재접속 대상이 애매해진다. 새로 만들면 on_connect 가 다시 불려
   구독·초기발행이 자동으로 재실행된다 — 세 서비스 모두 on_connect 에서 구독한다.
"""
import argparse
import socket
import threading
import time

import paho.mqtt.client as mqtt

# 앞이 우선순위. 유선(전용 링크) → 무선(로봇 AP)
HOSTS = ("10.10.10.64", "192.168.50.1")
PORT = 1883
PROBE_TIMEOUT = 0.4      # TCP 접속 시도 상한. 백그라운드라 루프엔 영향 없다


def reachable(host, port=PORT, timeout=PROBE_TIMEOUT):
    """ping 이 아니라 **1883 TCP 접속**으로 본다 — 링크가 올라와 있어도
    브로커가 안 떠 있으면 제어가 안 되므로, 그 구분이 필요하다."""
    try:
        with socket.create_connection((host, port), timeout):
            return True
    except OSError:
        return False


def wifi_signal(iface=None):
    """(iface, 품질%, dBm) 또는 None. `/proc/net/wireless` 만 읽는다 — 의존성·권한 0.

    `iw`/`nmcli` 를 부르면 매번 프로세스를 띄워야 해서 GUI 에서 1초마다 갱신하기엔
    아깝다. 커널이 이 파일에 그대로 노출한다.
    ⚠️ 일부 드라이버는 level 을 u8 로 올려 양수로 보인다 → 그때는 dBm 이 아니다.
    """
    try:
        lines = open("/proc/net/wireless").read().splitlines()[2:]
    except OSError:
        return None
    for ln in lines:
        if ":" not in ln:
            continue
        name, rest = ln.split(":", 1)
        name, f = name.strip(), rest.split()
        if (iface and name != iface) or len(f) < 3:
            continue
        try:
            link, level = float(f[1].rstrip(".")), float(f[2].rstrip("."))
        except ValueError:
            continue
        return name, max(0, min(100, round(link / 70 * 100))), level    # 70 = 전형적 최대 quality
    return None


def pick(hosts=HOSTS, port=PORT, probe=reachable):
    """우선순위 순으로 처음 닿는 호스트. 아무것도 안 되면 None.
    probe 를 주입할 수 있게 뺀 건 selftest 때문이다."""
    for h in hosts:
        if probe(h, port):
            return h
    return None


class Link:
    """우선순위 재접속·승격을 처리하는 paho 래퍼.

    on_connect(client, userdata, flags, rc) 등 콜백 시그니처는 paho v1 그대로다.
    on_switch(old, new) 는 경로가 바뀔 때 불린다(영상 파이프라인 재시작용).
    """

    def __init__(self, hosts=HOSTS, port=PORT, keepalive=10,
                 on_connect=None, on_message=None, on_disconnect=None,
                 will=None, probe_every=2.0, fail_switch=2, on_switch=None,
                 log=print):
        self.hosts, self.port, self.keepalive = tuple(hosts), port, keepalive
        self._cb = (on_connect, on_message, on_disconnect)
        self.will = will                      # (topic, payload, qos, retain)
        self.probe_every, self.fail_switch = probe_every, fail_switch
        self.on_switch, self.log = on_switch, log
        self.host, self.cli, self.connected = None, None, False
        self._last_try = 0.0
        self._found = None                    # 탐색 스레드가 채운다
        threading.Thread(target=self._prober, daemon=True).start()

    # ── 백그라운드 경로 탐색 ────────────────────────────────────
    def _prober(self):
        """붙어 있어도 **현재 경로를 실제로 확인한다.**

        🔴 paho keepalive 만 믿으면 링크가 죽은 걸 최대 1.5×keepalive 동안 모른다.
           실측(2026-08-18): 유선을 iptables 로 막고 15초를 기다려도 세 서비스가
           죽은 소켓을 그대로 붙들고 있었다. 그래서 능동 프로브가 필요하다.
        """
        bad = 0                               # 현재 경로 연속 실패 횟수
        while True:
            best = pick(self.hosts, self.port)
            cur = self.host
            if not self.connected or cur is None:
                bad, self._found = 0, best
            elif best == cur:
                bad, self._found = 0, cur     # 이미 최선
            else:
                ok = reachable(cur, self.port)
                bad = 0 if ok else bad + 1
                if not ok and bad >= self.fail_switch:
                    self._found = best        # 현재 경로 사망 → 이동
                elif ok and best and self.hosts.index(best) < self.hosts.index(cur):
                    self._found = best        # 상위 경로 복귀 → 승격(유선은 흔들리지 않으니 즉시)
                else:
                    self._found = cur         # 프로브가 한 번 튄 것 — 버틴다
            time.sleep(self.probe_every)

    # ── 접속 ───────────────────────────────────────────────────
    def _open(self, host):
        old = self.host
        if self.cli is not None:
            try:
                self.cli.loop_stop()
                self.cli.disconnect()
            except Exception:
                pass
        on_connect, on_message, on_disconnect = self._cb
        c = mqtt.Client()                     # paho 1.6.1 = v1 콜백 API

        def _conn(cl, u, f, rc):
            self.connected = (rc == 0)
            if on_connect:
                on_connect(cl, u, f, rc)

        def _disc(cl, u, rc):
            self.connected = False
            if on_disconnect:
                on_disconnect(cl, u, rc)

        c.on_connect, c.on_disconnect = _conn, _disc
        if on_message:
            c.on_message = on_message
        if self.will:
            c.will_set(*self.will)
        self.cli, self.host = c, host
        c.connect_async(host, self.port, self.keepalive)   # 블로킹 안 한다
        c.loop_start()
        self.log(f"[link] {host}:{self.port} 접속 시도"
                 f"{'' if old is None else f' (이전 {old})'}"
                 f"{'  ← 유선 우선' if host == self.hosts[0] else '  ← 폴백'}")
        if old and old != host and self.on_switch:
            self.on_switch(old, host)

    def tick(self):
        """루프에서 주기적으로 부른다. 블로킹하지 않는다."""
        now = time.monotonic()
        want = self._found
        if not self.connected:
            if want and now - self._last_try >= self.probe_every:
                self._last_try = now
                self._open(want)
        elif want and want != self.host:
            self._open(want)                  # 경로 변경(승격 또는 현재 경로 사망)

    def publish(self, topic, payload="{}", qos=1, **kw):
        """끊겨 있으면 그냥 실패한다 — 구동 명령이 끊기면 로봇이 워치독으로 선다."""
        if self.cli is None:
            return None
        return self.cli.publish(topic, payload, qos=qos, **kw)

    def subscribe(self, *a, **kw):
        return self.cli.subscribe(*a, **kw) if self.cli else None

    def stop(self):
        if self.cli is not None:
            try:
                self.cli.loop_stop()
                self.cli.disconnect()
            except Exception:
                pass


def selftest():
    up = set()

    def probe(h, _p=PORT, _t=None):
        return h in up

    W, A = HOSTS                                  # 유선, 무선
    assert pick(HOSTS, probe=probe) is None       # 둘 다 죽음
    up = {A}
    assert pick(HOSTS, probe=probe) == A          # 무선만 → 폴백
    up = {W}
    assert pick(HOSTS, probe=probe) == W
    up = {W, A}
    assert pick(HOSTS, probe=probe) == W, "둘 다 살아 있으면 유선이 이겨야 한다"
    # 순서가 곧 우선순위다
    assert pick((A, W), probe=probe) == A
    print("selftest OK")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--probe", action="store_true", help="지금 어느 경로가 열려 있나")
    a = ap.parse_args()
    if a.selftest:
        selftest()
    else:
        for h in HOSTS:
            print(f"  {h}:{PORT}  {'OK' if reachable(h) else 'X'}")
        print(f"→ 선택: {pick()}")
