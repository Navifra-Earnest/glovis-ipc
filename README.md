# Glovis 화재진압로봇 — IPC 측 구현

화재진압로봇의 **상위 시스템(IPC)** 코드다. 로봇(SBC)에서 도는 `navi` 는 이 저장소 범위 밖이며, **MQTT** 로만 연동한다(영상만 TCP).

```
IPC  ──명령(MQTT QoS1)──────────>  로봇 (Radxa ROCK 5A, 단일 바이너리 navi)
     <─상태·알람·이벤트(MQTT)──
     <─IR 영상(H.264/TCP:5000)──
```

## 구성

IPC 에서 **3개 프로세스**가 돈다. 일부러 분리했다 — 하나가 죽어도 나머지가 살고,
GUI 렌더링이 구동 명령의 350 ms 케이던스를 막으면 로봇이 워치독으로 멈춘다.

| 서비스 | 파일 | 담당 | 입력 |
|---|---|---|---|
| `navi-console` | `navi_console.py` | 영상·상태 표시, e-stop/리셋 버튼, 구동 허용 토글 | 터치스크린 |
| `joy-teleop` | `joy_teleop.py` | 주행 (메카넘 역기구학 → `cmd/wheel`) | USB 조이스틱 |
| `crevis-io` | `crevis_io.py` | 리프트 UP/DOWN, navi 재시작 | Crevis IO 물리버튼 (MODBUS TCP) |

진단 도구(서비스 아님): `crevis_probe.py`(Modbus 주소 스캔) · `crevis_map.py`(버튼↔비트 매핑)

## 설치

저장소가 진짜(truth)이고 IPC 는 배포본이다. **유닛이 `%h/navifra/*.py` 를 직접 가리키므로
경로는 `~/navifra` 고정**이다 — 사본을 만들지 않는다.

```bash
ssh navifra@<IPC>
git clone <이 저장소> ~/navifra
~/navifra/install.sh
```

갱신:

```bash
cd ~/navifra && git pull && ./install.sh        # 유닛까지 바뀐 경우
systemctl --user restart joy-teleop crevis-io  # 코드만 바뀐 경우
```

의존성은 apt 로만 받는다(`python3-paho-mqtt` · `mosquitto-clients` · `ffmpeg`). `install.sh` 가 빠진 걸 알려준다.

## 네트워크

⚠️ **주소가 자주 바뀌었다. 붙기 전에 `ip -br a` 로 실측할 것.** 아래는 2026-08-14 기준.

| 경로 | 주소 | 용도 |
|---|---|---|
| IPC `enp3s0` ↔ 로봇 `end1` | `10.10.10.74` ↔ **`10.10.10.64`** | **로봇 전용 유선 — 모든 명령이 이 길로** |
| IPC `enp2s0` | `192.168.100.10` | 관리망 (개발 PC 접속) |
| IPC `enp2s0` 대역 | `192.168.100.100` | **Crevis IO** (MODBUS TCP 502) |
| IPC `wlp4s0` | DHCP | 인터넷 |
| 로봇 AP | `192.168.50.1` (`EV-DL_AP`) | **무선 폴백 경로** + 개발 PC 직접 접속 |

MQTT 브로커는 **로봇에** 있다(`listener 1883 0.0.0.0`, 익명). IPC 가 꺼져 있어도 로봇은 상태를 계속 발행한다.

### 이중 경로 — 유선 우선, 무선 폴백

브로커가 `0.0.0.0` 에 바인딩돼 있어 **유선·무선 주소 양쪽으로 같은 브로커**가 열린다.
그래서 경로 전환은 전송 경로만 바뀌는 것이고 상태가 갈라지지 않는다(로봇 쪽 설정 변경 불필요).

세 서비스가 `mqtt_link.py` 를 공유해 같은 규칙으로 동작한다:

| 상황 | 동작 | 실측 지연 |
|---|---|---|
| 둘 다 살아있음 | **유선** 사용 | — |
| 유선 끊김 | 무선으로 폴백 | **3.1 초** |
| 유선 복구 | 유선으로 승격 | **0.9 초** |
| 둘 다 끊김 | 발행 실패 → 로봇이 워치독으로 정지 | 0.5 초(로봇 측) |

- 우선순위는 `--hosts` 의 **순서**다: `--hosts 192.168.50.1` 로 무선만 강제할 수도 있다.
- 콘솔 하단 첫 줄에 현재 경로가 표시된다 — `🔗 유선 …` / `📶 무선 … -41 dBm (99%)` / `⚠ 링크 없음`.
  무선 세기는 `/proc/net/wireless` 를 직접 읽는다(의존성·권한 0).
- 영상(TCP 5000)도 같은 경로를 따라간다 — 전환 시 `tcpclientsrc` 의 host 를 갈아끼우고 파이프라인을 재시작한다.
- ⚠️ **IPC 의 wifi 는 라디오가 하나다.** 로봇 AP(`EV-DL_AP`)에 붙어 있는 동안은 인터넷 AP 에 못 붙는다. 무선 폴백을 상시로 쓰려면 IPC wifi 를 로봇 AP 에 고정하거나 USB 동글을 하나 더 달아야 한다.
- ⚠️ 로봇 AP 를 켜면 **폐쇄망 전제가 깨진다** — 브로커가 익명 허용이라 AP 에 붙은 누구나 구동 명령을 쏠 수 있다.

## 조작

| 입력 | 동작 |
|---|---|
| 조이스틱 위/아래 | 전진 / 후진 |
| 조이스틱 좌/우 | 좌 / 우 게걸음 (메카넘) |
| **버튼 8 홀드** + 위/아래 | 제자리 회전 (스핀턴, 속도 0.3배) |
| IO 버튼 bit1 / bit2 | 리프트 UP / DOWN |
| IO 버튼 bit0 | **로봇 `navi` 서비스 재시작** (쿨다운 15초) |
| 콘솔 RESET 버튼 | `cmd/reset` — e-stop 래치 즉시 해제 |

## 알아야 하는 함정

실기에서 하나씩 밟은 것들이다. 자세한 근거는 각 파일 주석에 있다.

1. 🔴 **구동 명령 반복 주기는 350 ms 다.** 규격서는 200 ms 를 권하지만, 모터가 명령 후 250 ms 간 피드백을 버려서(`kCmdSettle`) 200 ms 로 반복하면 **`state.wheels[].rpm` 이 갱신되지 않는다** — 4축이 실제로 도는데 화면엔 0 으로 찍힌다. 워치독 500 ms 미만 · 정착창 250 ms 초과 구간이 350 ms 다.
2. 🔴 **액추에이터를 세우는 건 `{"dir":"ext","duty":0}` 뿐이다.** 운용 매뉴얼의 `{"dir":"stop"}` 예제는 **틀렸다**(`dir=="ret"` 이 아닌 모든 값이 ext 로 처리됨 → 멈추라고 보내면 전진한다). `cmd/stop` 은 구동계까지 감속 정지시킨다. 그리고 **"발행을 멈추는 것"도 정지가 아니다** — 워치독은 명령 종류 무관 타임스탬프 하나만 보고, joy-teleop 의 주행 명령이 그걸 계속 갱신한다.
3. **거부는 조용하다.** 명령이 거부돼도 오류 응답이 없다. e-stop 래치 중에는 모든 구동 명령이 무시되므로 "안 움직인다" 의 1순위 원인이다 → `state.estop` 확인.
4. **정상 정지에도 `event: watchdog` 이 뜬다.** `cmd/stop` 후 감속 구간에 명령이 없어서다. 이상상황이 아니다.
5. **IP 가 바뀌면 서비스를 재시작해야 한다.** 기존 TCP 연결은 소켓만 살아있고 명령이 안 나가는데 **아무 에러도 안 뜬다**. 확인: `ss -tnp | grep 1883` 의 local 주소가 현재 NIC 주소와 같은지.
6. 🔴 **MQTT 끊김은 paho keepalive 로 못 잡는다.** 링크가 죽어도 클라이언트는 최대 `1.5 × keepalive` 동안 "연결됨"으로 믿는다 — 실측으로 유선을 막고 15초를 기다려도 죽은 소켓을 붙들고 있었다. 그래서 `mqtt_link` 는 **현재 경로를 2초마다 직접 TCP 프로브**한다. 연속 2회 실패해야 옮기고(프로브 한 번 튀는 걸로 흔들리지 않게), 상위 경로 복귀는 즉시다.
7. 메카넘 부호는 **X-config 가정**이다. 게걸음이 반대로 가면 `joy_teleop.py` 의 `MIX` vy 열 4개만 뒤집는다(전후진·회전과 독립).

## 리셋 버튼 전제

IO 버튼 bit0 은 로봇에서 `systemctl restart navi` 를 실행한다. 두 가지가 미리 설정돼 있어야 한다:

```bash
# 1) IPC → 로봇 키 인증
ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519      # 없으면
ssh-copy-id radxa@10.10.10.64

# 2) 로봇에서 그 명령만 비밀번호 없이 허용
ssh radxa@10.10.10.64
echo 'radxa ALL=(root) NOPASSWD: /usr/bin/systemctl restart navi' | sudo tee /etc/sudoers.d/navi-restart
sudo chmod 0440 /etc/sudoers.d/navi-restart
sudo visudo -cf /etc/sudoers.d/navi-restart           # parsed OK 확인
```

`cmd/reset` 대신 재시작인 이유: `cmd/reset` 은 e-stop 래치만 풀고 **`drive_down`(구동계 초기화 실패)은 못 고친다**. 즉발 래치해제는 콘솔 RESET 버튼이 담당한다.

## 부팅 자동 시작

세 서비스는 systemd **user** 유닛이라 로그인 세션이 필요하다. IPC 는 **gdm 자동 로그인**(`AutomaticLogin = navifra`)이라 전원만 넣으면 세션이 생기고 따라 뜬다.
자동 로그인을 끄면 `sudo loginctl enable-linger navifra` 로 대체해야 한다.

## 로봇(SBC) 쪽

`navi` 소스는 보드의 `~/navi_src` 에 있고 **git 저장소가 아니다.** 설정 설치본은 `/opt/navi/navi.conf`.
현장 튜닝값(4축 매핑 · `gear_ratio 20` · `current_window 15` · `overcurrent_a 10` · wheel sign)이 **보드에만** 있다 → 원본 저장소 확보 + 스냅샷이 남은 과제다.

durable 지식(구성·결정·트러블슈팅·접속정보·TODO)은 Obsidian 핸드오프 노트에 있다 — `CLAUDE.md` 참조.
