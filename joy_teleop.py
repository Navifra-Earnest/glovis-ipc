#!/usr/bin/env python3
"""조이스틱(디지털 4방향 + 버튼 1개) → 메카넘 → navi/cmd/wheel MQTT 발행.

  기본        위아래 = 전후진 · 좌우 = 게걸음
  버튼 홀드   위아래 = 제자리 회전(스핀턴) · 좌우 = 게걸음 (그대로)


IPC 에서 실행한다. cmd/body 는 쓰지 않는다 — kinematics:false 면 조용히 무시되고,
애초에 vy(측면 이동)를 표현할 수 없다. 메카넘 역기구학을 IPC 에서 풀어
축별 RPM 을 cmd/wheel 로 보낸다.

  python3 joy_teleop.py                       # 유선 우선, 끊기면 무선(AP) 자동 폴백
  python3 joy_teleop.py --hosts 192.168.50.1  # 무선만 강제(유선 배제)
  python3 joy_teleop.py --prefix navitest     # 로봇 안 움직이는 발행 확인용
  python3 joy_teleop.py --selftest           # 기구학 자체검증 (하드웨어 불필요)
"""
import argparse, glob, json, math, os, struct, sys, time
import mqtt_link

WHEEL_RADIUS_M = 0.076   # 실측 후 수정. 속도 스케일에만 영향 — 틀려도 방향은 안 바뀐다
RPM_PER_MS     = 60.0 / (2.0 * 3.141592653589793 * WHEEL_RADIUS_M)   # m/s → RPM
MAX_RPM        = 20.0    # 축별 안전 상한 (문서 예제 최대치)
RAMP           = 0.5
POLL           = 0.05    # 입력 폴링 주기. 발행 주기(--period)와 분리한다 — 아래 due() 참조


def gate(spin, x, y, locked):
    """스핀턴 → 이동 전환 인터락. 반환 (이동 허용?, 다음 locked).

    버튼을 스틱보다 먼저 떼면 남아있던 축 값이 곧바로 게걸음/전후진으로 새어
    의도치 않게 잠깐 주행한다. 그래서 스핀턴이 끝나면 잠그고,
    **스틱이 중립(0,0)을 한 번 찍어야** 다시 이동을 허용한다.
    """
    if spin:
        return True, True                    # 스핀턴 중에는 통과. 끝나는 순간 잠긴다
    if locked:
        if x == 0.0 and y == 0.0:
            return True, False               # 중립 확인 → 해제
        return False, True                   # 아직 꺾여 있다 → 이동 금지
    return True, False


def due(rpm, last_rpm, now, last_pub, period):
    """지금 발행해야 하나. 입력이 바뀌면 즉시, 아니면 period 마다.

    발행 주기(350ms)를 폴링 주기로도 쓰면 스틱을 꺾거나 놓은 순간이
    최대 350ms 늦게 나간다 — 특히 손을 뗐을 때 정지가 그만큼 밀린다.
    반복 발행은 워치독 갱신이 목적이므로 주기를 유지하되, 변화는 즉시 내보낸다.
    """
    return rpm != last_rpm or now - last_pub >= period

# 메카넘 역기구학 혼합표 — 행 = FL FR RL RR (cmd/wheel 배열 순서), 열 = (vx, vy, wz)
# X-config 기준.
#
# wz(제자리 회전)는 원래 (lx+ly)·wz 로 차체 치수가 필요하지만, 디지털 입력이라
# 한 번에 한 방향만 들어온다 → 치수는 "선속도와 각속도를 섞을 때"만 필요하므로
# 여기서는 부호 패턴 × 고정 속도로 충분하다. wz 인자도 vx·vy 와 같은 단위로 받는다.
#
# ⚠️ 캘리브레이션: 최저속으로 순수 좌우(strafe)만 주고 각 바퀴 회전방향을 본다.
#    측면 이동이 반대로 가면 vy 열 4개의 부호를 전부 뒤집는다 (O-config 장착).
#    회전이 반대로 돌면 wz 열 4개만 뒤집는다 — 게걸음과 독립이다.
#    전후진 부호는 main() 의 hat.y 매핑에서 잡는다 (2026-08-14 반전함).
MIX = ((1.0, -1.0, -1.0),    # FL
       (1.0, +1.0, +1.0),    # FR
       (1.0, +1.0, -1.0),    # RL
       (1.0, -1.0, +1.0))    # RR


def mecanum_rpm(vx, vy, wz=0.0, radius=WHEEL_RADIUS_M, max_rpm=MAX_RPM, cap=None):
    """(vx, vy, wz) → [FL, FR, RL, RR] RPM. 상한 초과 시 4축을 **함께** 축소한다.

    cap: 어느 축도 넘지 않을 RPM. 대각선(x·y 동시)은 메카넘 혼합에서 두 바퀴가
    합산을 받아 **직진의 2배**가 된다 — 직진 [3,3,3,3] vs 대각 [0,6,6,0].
    체감상 갑자기 튀어나가므로(2026-08-18 사용자 지적) 직진 속도로 상한을 건다.
    회전+게걸음 같은 혼합 지령에도 같은 보호가 걸린다.
    """
    k = 60.0 / (2.0 * math.pi * radius)
    rpm = [(a * vx + b * vy + c * wz) * k for a, b, c in MIX]
    limit = max_rpm if cap is None else min(max_rpm, cap)
    peak = max(abs(r) for r in rpm)
    if peak > limit:                       # 축별 클램프는 지령 벡터를 회전시킨다 → 전체 스케일
        rpm = [r * limit / peak for r in rpm]
    return [round(r, 2) for r in rpm]


def find_joystick():
    """터치스크린이 js0 을 잡고 있으므로 경로를 박지 말고 by-id 로 찾는다.
    같은 장치가 `-event-joystick`(evdev, 24바이트 이벤트)로도 나오므로 반드시 제외한다."""
    for p in sorted(glob.glob("/dev/input/by-id/*-joystick")):
        if "TouchController" not in p and "-event-" not in p:
            return p
    raise SystemExit("조이스틱을 못 찾았다: /dev/input/by-id/*-joystick 없음")


class Hat:
    """joydev 논블로킹 리더. 디지털 4방향이라 HAT 축 두 개 + 버튼 하나만 본다."""
    AXIS_X, AXIS_Y = 6, 7

    def __init__(self, path, spin_btn=8):
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.x = self.y = 0.0
        self.spin_btn = spin_btn
        self.spin = False          # 버튼 홀드 = 좌우를 게걸음 대신 제자리 회전으로

    def poll(self):
        while True:
            try:
                data = os.read(self.fd, 8 * 32)   # 8바이트 단건 읽기는 레거시 경로로 빠진다
            except BlockingIOError:
                return
            if not data:
                return
            for off in range(0, len(data) - 7, 8):
                _, value, typ, num = struct.unpack_from("IhBB", data, off)
                if typ & 0x80:                     # 초기 상태 통보는 무시
                    continue
                if typ & 0x01:                     # 버튼 (실측: 8번, 눌림 1 / 뗌 0)
                    if num == self.spin_btn:
                        self.spin = bool(value)
                    continue
                if not typ & 0x02:
                    continue
                v = 0.0 if value == 0 else (1.0 if value > 0 else -1.0)
                if num == self.AXIS_X:
                    self.x = v
                elif num == self.AXIS_Y:
                    self.y = v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hosts", default=",".join(mqtt_link.HOSTS),
                    help="브로커 후보. 쉼표 구분이고 **앞이 우선순위**다 (유선 → 무선)")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--prefix", default="navi")
    ap.add_argument("--js", default=None)
    ap.add_argument("--speed", type=float, default=0.0239,
                    help="조이스틱 최대 입력 시 선속도 m/s (기본 0.0239 ≈ 3 RPM — 4축 동시 구동으로 검증된 값)")
    ap.add_argument("--period", type=float, default=0.35,
                    help="반복 발행 주기 s. 워치독 500ms 미만 · 모터 정착창 250ms 초과 구간 "
                         "(규격서 권장 200ms 로 하면 wheels[].rpm 이 조용히 0 으로 찍힌다)")
    ap.add_argument("--spin-button", type=int, default=8,
                    help="누르고 있는 동안 위아래가 제자리 회전(스핀턴)이 되는 버튼 번호 (실측 8번)")
    ap.add_argument("--spin-scale", type=float, default=0.3,
                    help="스핀턴 속도 배율. 제자리 회전은 같은 휠속도라도 체감이 빨라 낮춰 둔다 "
                         "(0.4 도 빨라서 2026-08-14 실기에서 0.3 으로 재조정)")
    ap.add_argument("--enable-topic", default="ipc/drive_enable",
                    help="콘솔의 구동 허용 토글 토픽. navi 접두사 밖이라 로봇은 구독하지 않는다")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        return selftest()

    hat = Hat(a.js or find_joystick(), a.spin_button)
    st = {}

    def on_connect(c, u, flags, rc):
        c.subscribe([(a.prefix + "/state", 0), (a.prefix + "/event", 1),
                     (a.enable_topic, 1)])
        print(f"[mqtt] rc={rc}  구동잠김(콘솔에서 허용해야 움직인다)")

    def on_message(c, u, msg):
        try:
            d = json.loads(msg.payload)
        except ValueError:
            return
        if msg.topic == a.enable_topic:
            # 콘솔이 소유하는 상태. 콘솔이 죽으면 브로커가 LWT 로 false 를 대신 발행한다.
            was, st["enabled"] = st.get("enabled", False), bool(d.get("on"))
            if was != st["enabled"]:
                print(f"\n[구동] {'허용' if st['enabled'] else '잠김'}")
            return
        if msg.topic.endswith("/event"):
            print(f"\n[event] {d}")
            return
        # 거부는 조용하다 → 반영 여부는 state 의 wheels[].rpm 으로만 확인된다
        cur = (d.get("drive_ok"), d.get("wheels_alive"), d.get("estop"),
               tuple(w.get("rpm") for w in d.get("wheels", [])))
        if cur != st.get("last"):
            st["last"] = cur
            print(f"\n[state] drive_ok={cur[0]} alive={cur[1]} estop={cur[2]} rpm={cur[3]}")
            if cur[2]:
                print("        ⚠ e-stop 래치 — cmd/reset 만 해제된다")

    # 유선 우선 · 무선 폴백. tick() 이 재접속과 승격을 맡는다 (mqtt_link 참고)
    cli = mqtt_link.Link(hosts=a.hosts.split(","), port=a.port,
                         on_connect=on_connect, on_message=on_message)

    moving, last_rpm, last_pub, locked = False, None, 0.0, False
    try:
        while True:
            hat.poll()
            cli.tick()
            # hat: 위=-1, 좌=-1 / ROS: +y=좌, +wz=CCW(좌회전).
            # vx 부호는 2026-08-14 실기에서 뒤집었다 — 전진 지령에 로봇이 뒤로 갔다.
            # 좌우(vy)는 건드리지 않는다. SBC 의 wheel sign 을 뒤집으면 좌우까지 같이 뒤집힌다.
            fwd = hat.y * a.speed
            vy = -hat.x * a.speed          # 좌우는 항상 게걸음
            # 버튼 홀드 중에는 위아래가 전후진 대신 제자리 회전(스핀턴)이 된다.
            vx, wz = (0.0, fwd * a.spin_scale) if hat.spin else (fwd, 0.0)
            allow, locked = gate(hat.spin, hat.x, hat.y, locked)
            # 구동 잠김이면 스틱을 놓은 것과 똑같이 취급한다 — 아래 정지 경로를 그대로 탄다
            allow = allow and st.get("enabled", False)
            # 어느 방향이든 --speed 가 정한 직진 휠속도를 넘지 않게 한다
            rpm = (mecanum_rpm(vx, vy, wz, cap=a.speed * RPM_PER_MS)
                   if allow else [0.0, 0.0, 0.0, 0.0])
            now = time.monotonic()
            if any(rpm) and due(rpm, last_rpm, now, last_pub, a.period):
                cli.publish(a.prefix + "/cmd/wheel",
                            json.dumps({"rpm": rpm, "ramp": RAMP}), qos=1)
                moving, last_pub = True, now
                mode = "스핀턴" if hat.spin else "이동  "
                print(f"\r[{mode}] vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f} rpm={rpm}   ",
                      end="", flush=True)
            elif not any(rpm) and moving:
                # rpm=[0,0,0,0] 은 절대 보내지 않는다 — 여자 유지 ~6.5A 로 과전류 e-stop 위험.
                # cmd/stop 후 0.8s ease-out 동안 watchdog 알람이 뜨는 건 정상이다.
                cli.publish(a.prefix + "/cmd/stop", "{}", qos=1)
                moving = False
                msg = ("구동 잠김" if not st.get("enabled", False)
                       else "인터락 — 스틱 중립 후 재개" if locked else "정지")
                print(f"\r{msg}{' ' * 24}", end="", flush=True)
            last_rpm = rpm
            time.sleep(POLL)
    except KeyboardInterrupt:
        pass
    finally:
        # wait_for_publish() 를 쓰면 링크가 끊긴 순간 PUBACK 을 영영 못 받아 Ctrl+C 가 안 먹는다.
        # 어차피 명령이 끊기면 워치독이 500ms 에 세운다 — 짧게 흘려보내고 나간다.
        cli.publish(a.prefix + "/cmd/stop", "{}", qos=1)
        time.sleep(0.2)
        cli.stop()
        print("\n정지 명령 발행 후 종료")


def selftest():
    r, k = WHEEL_RADIUS_M, 60.0 / (2 * math.pi * WHEEL_RADIUS_M)
    assert mecanum_rpm(0, 0) == [0, 0, 0, 0]

    fwd = mecanum_rpm(0.04, 0)
    assert all(x > 0 for x in fwd) and len(set(fwd)) == 1, fwd      # 전진 = 4축 동일 부호·크기
    assert abs(fwd[0] - 0.04 * k) < 0.01, fwd

    left = mecanum_rpm(0, 0.04)                                     # +y=좌 → FL·RR 후진, FR·RL 전진
    assert left[0] < 0 and left[1] > 0 and left[2] > 0 and left[3] < 0, left

    diag = mecanum_rpm(0.04, 0.04)                                  # 대각 = 두 축만 구동
    assert diag[0] == 0 and diag[3] == 0 and diag[1] > 0 and diag[2] > 0, diag
    # 상한 없으면 대각이 직진의 2배다 (메카넘 혼합의 성질)
    assert abs(diag[1] - 2 * fwd[0]) < 0.02, (diag, fwd)
    # cap 을 직진 속도로 주면 대각도 그 값을 안 넘는다 (2026-08-18 사용자 요청)
    capped = mecanum_rpm(0.04, 0.04, cap=0.04 * RPM_PER_MS)
    assert max(abs(x) for x in capped) <= fwd[0] + 0.02, (capped, fwd)
    assert capped[1] > 0 and capped[2] > 0 and capped[0] == 0, capped   # 방향은 유지
    assert mecanum_rpm(0.04, 0, cap=0.04 * RPM_PER_MS) == fwd, "직진은 영향 없어야 한다"

    big = mecanum_rpm(10.0, 5.0)                                    # 상한 초과 → 비율 보존 축소
    ref = mecanum_rpm(0.10, 0.05)
    assert max(abs(x) for x in big) - MAX_RPM < 1e-6, big
    for b, f in zip(big, ref):                                      # 벡터 방향 불변 (반올림 여유)
        assert abs(b / big[1] - f / ref[1]) < 0.01, (big, ref)

    back = mecanum_rpm(-0.04, 0)
    assert back == [-x for x in fwd], back

    # 제자리 회전(wz): 좌회전 CCW 는 좌측 후진 · 우측 전진. 게걸음과 패턴이 달라야 한다
    ccw = mecanum_rpm(0, 0, 0.04)
    assert ccw[0] < 0 and ccw[1] > 0 and ccw[2] < 0 and ccw[3] > 0, ccw
    assert mecanum_rpm(0, 0, -0.04) == [-x for x in ccw], ccw
    assert ccw != mecanum_rpm(0, 0.04), "회전이 게걸음과 같은 패턴이면 부호표가 틀린 것"
    assert all(abs(x) == abs(ccw[0]) for x in ccw), ccw   # 4축 같은 크기

    # 스핀턴 → 이동 인터락: 중립을 찍기 전에는 이동이 안 나가야 한다
    assert gate(True, 0.0, -1.0, False) == (True, True)      # 스핀턴 중 — 통과하고 잠금 예약
    assert gate(False, -1.0, 0.0, True) == (False, True)     # 버튼만 뗌, 스틱 꺾인 채 → 차단
    assert gate(False, 0.0, -1.0, True) == (False, True)     # y 만 남아도 차단
    assert gate(False, 0.0, 0.0, True) == (True, False)      # 중립 찍음 → 해제
    assert gate(False, -1.0, 0.0, False) == (True, False)    # 평상시 게걸음은 그대로

    # 발행 시점 판정: 변화는 즉시, 무변화는 주기마다
    assert due([3, 3, 3, 3], None, 1.0, 0.0, 0.35)              # 첫 발행
    assert due([3, 3, 3, 3], [0, 3, 3, 3], 1.0, 0.99, 0.35)     # 방향 바뀜 → 주기 무시하고 즉시
    assert not due([3, 3, 3, 3], [3, 3, 3, 3], 1.0, 0.99, 0.35)  # 무변화 + 주기 전 → 억제
    assert due([3, 3, 3, 3], [3, 3, 3, 3], 1.4, 0.99, 0.35)     # 무변화 + 주기 경과 → 워치독 갱신
    print("selftest OK")


if __name__ == "__main__":
    main()
