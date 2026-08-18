#!/usr/bin/env python3
"""Crevis IO(MODBUS TCP) 물리 버튼 → navi 리프트(액추에이터)·리셋 명령.

IPC 에서 실행한다. 조이스틱(joy_teleop)·콘솔(navi_console)과 **별도 프로세스**다 —
Modbus 가 멈춰도 주행이 죽지 않아야 한다.

  버튼 (Crevis discrete input, fn2 addr 0 / 실측 2026-08-14)
    bit0 = 리셋      → 로봇에서 `systemctl restart navi` (디바운스 상승엣지 1회, 쿨다운 15초)
                       cmd/reset 과 달리 drive_down 까지 복구된다. 콘솔 RESET 버튼은 즉발 래치해제.
    bit1 = 리프트 UP  → navi/cmd/actuator {"dir":"ret"}   누르고 있는 동안 반복
    bit2 = 리프트 DOWN → navi/cmd/actuator {"dir":"ext"}   〃

라이브러리 없이 순수 소켓 Modbus (crevis_probe.Modbus). 납품 시 의존성 0.

  python3 crevis_io.py
  python3 crevis_io.py --prefix navitest   # 로봇 안 움직이는 발행 확인용
  python3 crevis_io.py --selftest          # 판정 로직 검증 (하드웨어 불필요)
"""
import argparse
import json
import pathlib
import subprocess
import sys
import time

import mqtt_link

from crevis_probe import Modbus

POLL = 0.05          # Modbus 폴링 주기 (RTT 0.2ms 라 부담 없다)

# ⚠️⚠️ 액추에이터를 세우는 방법은 **duty:0 뿐이다.** 다음 두 개를 쓰면 안 된다:
#
#   1) `{"dir":"stop"}` — 운용 매뉴얼 §7.3 예제에 있지만 **틀렸다.**
#      mqtt.hpp:253 이 `dir == "ret"` 만 후진으로 보고 **나머지 전부를 ext(전진)로**
#      처리한다. 즉 "stop" 을 보내면 멈추는 게 아니라 전진한다.
#
#   2) `cmd/stop` — 액추에이터는 서지만 **구동계까지 감속 정지**시킨다.
#      조이스틱으로 주행하는 중에 버튼을 떼면 로봇이 같이 멈춘다.
#
# 그리고 "발행을 멈추는 것"도 정지가 아니다 — 워치독(500ms)은 `cmd_at_` 타임스탬프
# **하나**를 보고, 그건 joy_teleop 의 350ms 주행 명령이 계속 갱신한다.
# 즉 주행 중에는 액추에이터 워치독이 영영 안 걸린다. 반드시 명시적으로 세운다.
STOP = json.dumps({"dir": "ext", "duty": 0})

# 리프트 방향 → navi 의 액추에이터 dir.
# 🔴 실측(2026-08-14): **ret = 상승, ext = 하강**. 문서에는 ext/ret 의 물리 방향이
#    안 적혀 있고, 처음에 ext=상승으로 넣었다가 실기에서 반대로 움직였다.
#    또 뒤집힐 일이 생기면 **이 한 줄만** 고친다.
WIRE = {"up": "ret", "down": "ext"}

# duty 를 안 넣으면 보드의 navi.conf 설정값(actuator_duty)을 쓴다 — IPC 에 값을 중복하지 않는다.
JOG = {k: json.dumps({"dir": v}) for k, v in WIRE.items()}
LABEL = {"up": "리프트 상승", "down": "리프트 하강"}


def decide(up, down):
    """버튼 상태 → 리프트 방향("up"/"down"). None 이면 정지.

    둘 다 눌리면 정지한다 — 어느 쪽을 고르든 오조작이라 움직이지 않는 게 맞다.
    """
    if up and down:
        return None
    if up:
        return "up"
    if down:
        return "down"
    return None


class Debounce:
    """연속 n 샘플이 같을 때만 상태를 바꾼다. update() 는 상승엣지에서 True.

    리셋 버튼에만 쓴다 — 스퍼리어스 엣지 한 번으로 **안전 래치가 풀리면 안 된다.**
    조그 버튼은 자기교정된다(잘못된 엣지 = 350ms 펄스 한 번)므로 디바운스가 불필요하다.
    """

    def __init__(self, n=2):
        self.n, self.state, self.run, self.raw = n, False, 0, False

    def update(self, raw):
        self.run = self.run + 1 if raw == self.raw else 1
        self.raw = raw
        if self.run >= self.n and self.state != raw:
            self.state = raw
            return raw            # True = 상승엣지
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--io-host", default="192.168.100.100")
    ap.add_argument("--io-port", type=int, default=502)
    ap.add_argument("--unit", type=int, default=1)
    ap.add_argument("--hosts", default=",".join(mqtt_link.HOSTS),
                    help="브로커 후보. 쉼표 구분이고 **앞이 우선순위**다 (유선 → 무선)")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--prefix", default="navi")
    ap.add_argument("--period", type=float, default=0.35,
                    help="조그 반복 발행 주기 s (워치독 500ms 미만)")
    ap.add_argument("--restart-cmd",
                    default="ssh -o BatchMode=yes -o ConnectTimeout=5 "
                            "-o StrictHostKeyChecking=accept-new "
                            "radxa@{host} sudo -n systemctl restart navi",
                    help="리셋 버튼이 실행할 명령. `{host}` 는 **현재 붙어 있는 경로**로 치환된다"
                         " (유선/무선). 키 인증 + sudoers NOPASSWD 가 전제")
    ap.add_argument("--restart-cooldown", type=float, default=15.0,
                    help="재시작 재요청 최소 간격 s (재시작 자체가 ~7초)")
    ap.add_argument("--bit-reset", type=int, default=0)
    ap.add_argument("--bit-up", type=int, default=1)
    ap.add_argument("--bit-down", type=int, default=2)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        return selftest()

    st = {"estop": False}

    def on_connect(c, u, flags, rc):
        c.subscribe(a.prefix + "/state", 0)
        print(f"[mqtt] rc={rc}", flush=True)

    def on_message(c, u, msg):
        try:
            d = json.loads(msg.payload)
        except ValueError:
            return
        was, st["estop"] = st["estop"], bool(d.get("estop"))
        if was != st["estop"]:
            print(f"\n[estop] {'래치 — 리셋 버튼으로 해제' if st['estop'] else '해제됨'}",
                  flush=True)

    # 유선 우선 · 무선 폴백 (mqtt_link 참고)
    hosts = a.hosts.split(",")
    cli = mqtt_link.Link(hosts=hosts, port=a.port,
                         on_connect=on_connect, on_message=on_message)

    def stop_actuator(why):
        cli.publish(a.prefix + "/cmd/actuator", STOP, qos=1)
        print(f"\r리프트 정지 ({why}){' ' * 20}", end="", flush=True)

    io, reset_db, jogging, last_pub, last_restart = None, Debounce(2), None, 0.0, 0.0
    try:
        while True:
            # ── Modbus 읽기 (끊기면 재접속. 그 사이 액추에이터는 세워 둔다) ──
            if io is None:
                try:
                    io = Modbus(a.io_host, a.io_port, a.unit)
                    print(f"[io] {a.io_host}:{a.io_port} unit={a.unit} 접속", flush=True)
                except Exception as e:
                    print(f"\r[io] 접속 실패: {e} — 3초 후 재시도", end="", flush=True)
                    time.sleep(3.0)
                    continue
            try:
                bits = io.read_bits(2, 0x0000, 16)
            except Exception as e:
                io = None
                if jogging:
                    jogging = None
                    stop_actuator(f"IO 통신 끊김: {e}")
                continue

            want = decide(bits[a.bit_up], bits[a.bit_down])
            if st["estop"]:
                want = None       # e-stop 중에는 어차피 거부된다. 의도를 남기지 않는다

            # ── 조그: 누르고 있는 동안 반복 발행, 놓으면 명시적 정지 ──
            now = time.monotonic()
            if want and (want != jogging or now - last_pub >= a.period):
                cli.publish(a.prefix + "/cmd/actuator", JOG[want], qos=1)
                jogging, last_pub = want, now
                print(f"\r{LABEL[want]}{' ' * 24}", end="", flush=True)
            elif not want and jogging:
                jogging = None
                stop_actuator("버튼 뗌" if not st["estop"] else "e-stop")

            # ── 리셋: 디바운스된 상승엣지에서 navi 재시작 1회 ──
            #
            # cmd/reset 이 아니라 **서비스 재시작**이다(사용자 결정 2026-08-14).
            # cmd/reset 은 e-stop 래치만 풀고 `drive_down`(구동계 초기화 실패)은 못 고친다 —
            # 그건 재시작만 복구된다. 대신 ~7초 걸리고 전 장치가 재초기화된다.
            # 즉발 래치해제가 필요하면 콘솔의 RESET 버튼(cmd/reset)을 쓴다.
            if reset_db.update(bool(bits[a.bit_reset])):
                if now - last_restart < a.restart_cooldown:
                    print(f"\n[리셋] 쿨다운 중 — {a.restart_cooldown:.0f}초 내 재시작 무시",
                          flush=True)
                else:
                    # 🔴 호스트를 **박아두면 안 된다.** 유선 IP 를 상수로 두었더니 무선으로
                    #    돌던 2026-08-18 에 `Network is unreachable` 로 리셋 버튼이 조용히
                    #    전부 실패했다(열화상 정지 중이라 급했다). 영상 호스트가 갈라졌던 것과
                    #    같은 버그다 — 경로의 주인은 Link 하나다.
                    host = cli.host or mqtt_link.pick(hosts, a.port)
                    if not host:
                        print("\n[리셋] 닿는 경로가 없다 — 유선·무선 둘 다 끊김", flush=True)
                    else:
                        last_restart = now
                        cmd = a.restart_cmd.format(host=host)
                        # 블로킹하면 이 루프가 멈춰 리프트를 세울 주체가 사라진다 → 던지고 잊는다.
                        # 출력은 상속돼 journalctl --user -u crevis-io 에 남는다.
                        subprocess.Popen(cmd, shell=True)
                        print(f"\n[리셋] navi 재시작 요청: {cmd}", flush=True)

            cli.tick()
            time.sleep(POLL)
    except KeyboardInterrupt:
        pass
    finally:
        # 🔴 이 프로세스가 죽는 순간에도 액추에이터는 세워야 한다.
        #    주행 중이면 워치독이 대신 세워주지 않는다(위 STOP 주석 참고).
        cli.publish(a.prefix + "/cmd/actuator", STOP, qos=1)
        time.sleep(0.2)
        cli.stop()
        print("\n액추에이터 정지 발행 후 종료")


def selftest():
    assert decide(False, False) is None
    assert decide(True, False) == "up"
    assert decide(False, True) == "down"
    assert decide(True, True) is None, "동시 입력은 정지여야 한다"
    # 실측 방향 매핑 — 뒤집히면 여기서 잡힌다
    assert WIRE["up"] == "ret" and WIRE["down"] == "ext"
    assert json.loads(JOG["up"])["dir"] == "ret"
    assert json.loads(STOP)["duty"] == 0

    # 리셋 명령에 IP 를 박으면 다른 경로에서 조용히 전부 실패한다(2026-08-18). 재발 방지.
    src = pathlib.Path(__file__).read_text().split("def selftest")[0]
    assert "radxa@{host}" in src, "restart-cmd 기본값에 {host} 치환자가 없다"
    assert "10.10.10." not in src, "리셋 명령에 IP 가 박혀 있다"

    d = Debounce(2)
    assert d.update(True) is False          # 1샘플 — 아직 확정 아님
    assert d.update(True) is True           # 2샘플 연속 → 상승엣지
    assert d.update(True) is False          # 유지 중에는 반복 발행 안 함
    assert d.update(False) is False         # 하강 1샘플
    assert d.update(False) is False         # 하강 확정 (엣지 아님)
    assert d.update(True) is False          # 다시 1샘플
    assert d.update(True) is True           # 재상승
    # 스퍼리어스 1샘플 노이즈로는 절대 안 터진다
    d2 = Debounce(2)
    for _ in range(5):
        assert d2.update(True) or True       # 초기 안정화
    d3 = Debounce(2)
    assert d3.update(True) is False
    assert d3.update(False) is False
    assert d3.update(True) is False          # 튀는 입력 → 확정 안 됨
    print("selftest OK")


if __name__ == "__main__":
    sys.exit(main())
