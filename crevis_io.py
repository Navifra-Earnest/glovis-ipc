#!/usr/bin/env python3
"""Crevis IO(MODBUS TCP) 물리 버튼 → navi 리프트(액추에이터)·리셋 명령.

IPC 에서 실행한다. 조이스틱(joy_teleop)·콘솔(navi_console)과 **별도 프로세스**다 —
Modbus 가 멈춰도 주행이 죽지 않아야 한다.

  버튼 (Crevis discrete input, fn2 addr 0 / 실측 2026-08-14)
    bit0 = 리셋      → ① IPC UI(`navi-console`) 재시작 — 전체화면 복구. 항상 실행
                         ② 로봇 `systemctl restart navi` — 닿는 경로가 있을 때
                       (디바운스 상승엣지 1회, 쿨다운 15초)
                       cmd/reset 과 달리 drive_down 까지 복구된다. 콘솔 RESET 버튼은 즉발 래치해제.
                       ⚠️ UI 재시작 후에는 **구동허용이 잠긴다**(콘솔이 접속 시 잠금 발행) —
                          다시 주행하려면 콘솔에서 구동허용을 켠다. 안전 기본값이라 의도된 동작이다.
    bit1 = 리프트 UP  → navi/cmd/actuator {"dir":"ret"}   누르고 있는 동안 반복
    bit2 = 리프트 DOWN → navi/cmd/actuator {"dir":"ext"}   〃

라이브러리 없이 순수 소켓 Modbus (crevis_probe.Modbus). 납품 시 의존성 0.

  python3 crevis_io.py
  python3 crevis_io.py --prefix navitest   # 로봇 안 움직이는 발행 확인용
  python3 crevis_io.py --selftest          # 판정 로직 검증 (하드웨어 불필요)
"""
import argparse
import json
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

# 리셋 버튼이 실행할 명령. 🔴 `{host}` 를 **반드시** 남긴다 — 여기에 IP 를 박았더니
# 무선으로 돌던 2026-08-18 에 리셋이 조용히 전부 실패했다(핸드오프 4.14).
# 무선 IP 는 known_hosts 에 없을 수 있어 accept-new 로 붙는다.
RESTART_CMD = ("ssh -o BatchMode=yes -o ConnectTimeout=5 "
               "-o StrictHostKeyChecking=accept-new "
               "radxa@{host} sudo -n systemctl restart navi")

# IPC 콘솔(UI) 재시작. 🔴 **키보드·마우스가 없다** — 오류로 전체화면이 풀리면 이 버튼이
# 유일한 복구 수단이다. 유닛의 ExecStart 에 `--fullscreen` 이 있어서 재시작하면 되돌아온다.
# 로봇 재시작과 **분리**한다: 네트워크가 죽어도 UI 는 되살려야 한다.
UI_RESTART_CMD = "systemctl --user restart navi-console"

# duty 를 안 넣으면 보드의 navi.conf 설정값(actuator_duty)을 쓴다 — IPC 에 값을 중복하지 않는다.
JOG = {k: json.dumps({"dir": v}) for k, v in WIRE.items()}
LABEL = {"up": "리프트 상승", "down": "리프트 하강"}


# ── 리프트 막힘 판정 (2026-09-02) ─────────────────────────────────────────
# 차체 하부에 닿은 뒤에도 계속 밀면 모터가 상한다. 그런데 navi 의 자체 보호는
# **우리 때문에 무력화된다**: 매 `cmd/actuator` 가 jog()→start() 를 부르며
# `state_` 를 Running 으로, `t0_`·홀 엣지 타이머를 초기화하는데(actuator.hpp:136·144),
# `start_grace` 가 1000 ms 인 반면 우리는 350 ms 마다 재발행한다 → **홀 정지 판정
# (`elapsed > start_grace`)이 영영 참이 되지 않는다.** 남는 보호는 `i_avg_ >= 2.0 A`
# 하나뿐이고, 그마저 걸린 직후 우리 명령이 다시 밀어붙여 차체를 두드린다.
#
# 그래서 IPC 가 막는다. 판정은 **위치 변화**가 1차다:
#
#   상승 명령 중 홀 카운트가 STALL_S 동안 안 늘어나면 → 막힘
#
# 🔴 **절대 위치가 아니다.** 차체 높이는 차마다 달라서 "몇 카운트에서 멈춘다" 는
#    쓸 수 없다(기준점도 없다 — navi 재시작하면 0부터 센다). 높이가 얼마든
#    "멈췄다" 는 같으므로 변화량만 본다.
#
# 전류는 보조다. 실측(2026-09-02): 무부하 상승 0.87~1.00 A · 발 하중 0.71~1.09 A ·
# 하강 0.29 A. **무부하와 하중의 간격이 9% 뿐이라 전류만으로는 못 가른다.**
# 대신 명백한 과전류(navi 의 blocked_current 와 같은 1.5 A)는 즉시 차단한다.
STALL_S = 1.8          # 이 시간 동안 안 늘어나면 막힘 (기동 램프업 ~1초를 넘겨야 한다)
STALL_MIN_ADV = 3      # 홀 카운트 이 미만 변화는 노이즈
BLOCK_CUR_A = 1.5      # 보조 차단 전류


def lift_blocked(quiet_s, cur, stall_s=STALL_S, cur_a=BLOCK_CUR_A):
    """상승을 멈춰야 하나. quiet_s = 엔코더가 마지막으로 늘어난 뒤 경과 시간.

    반환: 멈출 사유 문자열, 아니면 None. 순수 함수라 selftest 로 검증한다.

    **사유를 두 가지로 나눈다** — 조종자에게 뜻이 완전히 다르다:
      · 엔코더 정지 → `도달`. 차체에 닿았거나 스트로크 끝이다. **정상 완료**다.
      · 과전류      → `과전류`. 이건 이상이다(무부하 1.0 A · 발하중 1.09 A 실측이라
                      1.5 A 는 정상 동작에서 나올 수 없는 값이다).
    같은 문구로 띄우면 "다 올라간 것" 과 "뭔가 잘못된 것" 을 구분할 수 없다.
    """
    if cur is not None and cur >= cur_a:
        return f"과전류 {cur:.2f}A (임계 {cur_a}A) — 상승 중단"
    if quiet_s >= stall_s:
        return f"도달 — 엔코더가 {quiet_s:.1f}초간 안 늘어남"
    return None


def status(msg, force=False, _s={"t": 0.0}):
    """tty 면 한 줄 갱신(\r), 서비스로 돌 때는 **개행해서 journald 에 남긴다**.

    🔴 \r 만 쓰면 journald 에 한 줄도 안 남는다(개행이 없어 커밋되지 않고
       `[116B blob data]` 로 뭉개진다). 2026-09-02 리프트 전류를 재려는데 **UP 이
       언제 나갔는지 확인할 방법이 없었다** — joy2_teleop 과 같은 버그였다.
       발행 주기로 다 남기면 저널이 넘치므로 1초에 한 줄로 줄인다.
       force=True 는 상태 전이(정지 등) — 빈도와 무관하게 항상 남긴다.
    """
    if sys.stdout.isatty():
        print("\r" + msg, end="", flush=True)
        return
    now = time.monotonic()
    if force or now - _s["t"] >= 1.0:
        _s["t"] = now
        print(msg, flush=True)


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
    ap.add_argument("--ui-restart-cmd", default=UI_RESTART_CMD,
                    help="리셋 버튼이 같이 재시작할 IPC UI. 빈 문자열이면 끈다")
    ap.add_argument("--restart-cmd", default=RESTART_CMD,
                    help="리셋 버튼이 실행할 명령. `{host}` 는 **현재 붙어 있는 경로**로 치환된다"
                         " (유선/무선). 키 인증 + sudoers NOPASSWD 가 전제")
    ap.add_argument("--restart-cooldown", type=float, default=15.0,
                    help="재시작 재요청 최소 간격 s (재시작 자체가 ~7초)")
    ap.add_argument("--block-topic", default="ipc/lift_blocked",
                    help="막힘 알람 토픽 (콘솔이 구독해 경고줄에 띄운다)")
    ap.add_argument("--stall-s", type=float, default=STALL_S,
                    help="상승 중 홀 카운트가 이 시간 안 늘면 막힘으로 본다")
    ap.add_argument("--block-cur", type=float, default=BLOCK_CUR_A,
                    help="보조 차단 전류 A")
    ap.add_argument("--bit-reset", type=int, default=0)
    ap.add_argument("--bit-up", type=int, default=1)
    ap.add_argument("--bit-down", type=int, default=2)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        return selftest()

    # `"radxa@10.10.10.64".format(host=x)` 는 **조용히 원문을 돌려준다** — 치환자를 빼먹으면
    # 경로가 박힌 채로 잘 도는 것처럼 보인다. selftest 는 기본값만 보므로 런타임에서도 막는다.
    assert "{host}" in a.restart_cmd, "--restart-cmd 에 {host} 치환자가 없다 — 경로가 박힌다"

    st = {"estop": False}

    def on_connect(c, u, flags, rc):
        c.subscribe(a.prefix + "/state", 0)
        print(f"[mqtt] rc={rc}", flush=True)

    def on_message(c, u, msg):
        try:
            d = json.loads(msg.payload)
        except ValueError:
            return
        act = d.get("actuator") or {}
        st["act_state"], st["act_cur"] = act.get("state"), act.get("current")
        pos = act.get("position")
        st["act_pos"] = pos
        # **늘어난 시각**만 기록한다(줄어드는 건 하강이라 상승 판정과 무관)
        if pos is not None:
            base = st.get("adv_pos")
            if base is None or pos - base >= STALL_MIN_ADV:
                st["adv_pos"], st["adv_t"] = pos, time.monotonic()
            elif pos < base:
                st["adv_pos"] = pos            # 하강으로 기준을 낮춘다
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
        status(f"리프트 정지 ({why})", force=True)

    io, reset_db, jogging, last_pub, last_restart = None, Debounce(2), None, 0.0, 0.0
    up_since, blocked = 0.0, None      # 상승 시작 시각 · 막힘 사유(래치)

    def set_blocked(reason):
        """막힘 래치. 액추에이터를 세우고 콘솔에 알린다.

        래치인 이유: 풀어주면 350 ms 뒤 다시 밀어붙인다 — navi 보호가 무력화된 것과
        같은 실수다. **버튼을 뗐다 다시 눌러야** 재시도된다.
        """
        cli.publish(a.prefix + "/cmd/actuator", STOP, qos=1)
        cli.publish(a.block_topic, json.dumps({"on": True, "reason": reason}),
                    qos=1, retain=True)
        print(f"\n⬆ [리프트] {reason}  (버튼을 뗐다 다시 눌러야 재시도)", flush=True)
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

            # ── 리프트 막힘: 상승만 본다(하강은 중력이 돕고 바닥/끝단에서 멈춘다) ──
            now0 = time.monotonic()
            if want != "up":
                if blocked:       # 버튼을 뗐다 → 래치 해제
                    blocked = None
                    cli.publish(a.block_topic, '{"on":false}', qos=1, retain=True)
                    print("\n[리프트] 상승 차단 해제 (버튼 뗌)", flush=True)
                up_since = 0.0
            else:
                if not up_since:
                    up_since = now0
                    st["adv_t"] = now0        # 기동 램프업을 유예한다
                if not blocked:
                    quiet = now0 - max(st.get("adv_t", now0), up_since)
                    blocked = lift_blocked(quiet, st.get("act_cur"),
                                           a.stall_s, a.block_cur)
                    if blocked:
                        set_blocked(blocked)
                if blocked:
                    want = None               # 상승 명령을 무시한다

            # ── 조그: 누르고 있는 동안 반복 발행, 놓으면 명시적 정지 ──
            now = time.monotonic()
            if want and (want != jogging or now - last_pub >= a.period):
                cli.publish(a.prefix + "/cmd/actuator", JOG[want], qos=1)
                first = want != jogging
                jogging, last_pub = want, now
                # 누른 순간은 반드시 남긴다(force) — 이후 반복은 1초에 한 줄
                status(f"{LABEL[want]}  (act {st.get('act_state', '?')} "
                       f"{st.get('act_cur', '?')}A pos {st.get('act_pos', '?')})",
                       force=first)
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
                    last_restart = now
                    # 블로킹하면 이 루프가 멈춰 리프트를 세울 주체가 사라진다 → 전부 던지고 잊는다.
                    # 출력은 상속돼 journalctl --user -u crevis-io 에 남는다.
                    #
                    # ① IPC UI — 로컬이라 **경로와 무관하게 항상** 재시작한다.
                    #    전체화면이 풀렸을 때 되돌릴 수단이 이 버튼뿐이다(입력장치 없음).
                    if a.ui_restart_cmd:
                        subprocess.Popen(a.ui_restart_cmd, shell=True)
                        print(f"\n[리셋] IPC UI 재시작: {a.ui_restart_cmd}", flush=True)
                    # ② 로봇 navi — 경로가 있어야 한다
                    host = cli.host or mqtt_link.pick(hosts, a.port)
                    if not host:
                        print("[리셋] navi 는 건너뜀 — 닿는 경로가 없다(유선·무선 둘 다 끊김)",
                              flush=True)
                    else:
                        cmd = a.restart_cmd.format(host=host)
                        subprocess.Popen(cmd, shell=True)
                        print(f"[리셋] navi 재시작 요청: {cmd}", flush=True)

            cli.tick()
            time.sleep(POLL)
    except KeyboardInterrupt:
        pass
    finally:
        # retain 알람을 남기면 콘솔에 영영 뜬다 — 나갈 때 반드시 지운다
        cli.publish(a.block_topic, '{"on":false}', qos=1, retain=True)
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
    assert "{host}" in RESTART_CMD, "리셋 명령에 {host} 치환자가 없다 — 경로가 박힌다"
    # UI 재시작은 로컬이라 호스트가 없어야 한다 — ssh 로 돌리면 경로가 죽을 때 같이 죽는다
    assert "{host}" not in UI_RESTART_CMD and "ssh" not in UI_RESTART_CMD
    assert "navi-console" in UI_RESTART_CMD
    assert RESTART_CMD.format(host="1.2.3.4").endswith("systemctl restart navi")

    # ── 리프트 막힘 판정 ──
    assert lift_blocked(0.0, 0.9) is None
    assert lift_blocked(1.7, 0.9) is None, "STALL_S 전에는 통과"
    # 엔코더 정지는 "도달"(정상 완료), 과전류는 "과전류"(이상) — 문구가 갈려야 한다
    assert "도달" in lift_blocked(1.9, 0.9)
    assert "과전류" in lift_blocked(0.0, 1.5), "명백한 과전류는 즉시"
    assert "과전류" in lift_blocked(0.0, 2.4)
    assert "도달" not in lift_blocked(0.0, 2.4), "과전류를 도달로 알리면 안 된다"
    # 둘이 동시면 과전류가 이긴다(더 급한 정보다)
    assert "과전류" in lift_blocked(9.9, 2.0)
    assert lift_blocked(0.0, None) is None, "전류를 아직 못 받았으면 전류로 안 막는다"
    assert lift_blocked(2.0, None) is not None, "전류 없어도 위치로는 막는다"
    # 🔴 실측된 무부하·하중 전류로는 절대 막히면 안 된다 (오작동 = 리프트가 안 올라간다)
    for cur in (0.29, 0.87, 0.92, 1.00, 1.09):
        assert lift_blocked(0.5, cur) is None, cur

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
