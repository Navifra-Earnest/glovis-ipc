# 로봇(SBC) `navi` 스냅샷 — **백업이지 원본이 아니다**

> [!important] 2026-09-03 부터 여기가 **로봇 소스의 작업 공간**이다
> 원래는 "떠온 사본, 여기서 고쳐도 로봇에 반영 안 됨" 이었다. 그런데 작업등(`cmd/led`)을
> 구현하면서 실제로 **여기서 고치고 로봇으로 올렸다.** 원본 저장소가 없는 상태에서
> 이게 최선이다 — 로봇의 유일본을 직접 편집하면 이력이 안 남는다.
>
> **작업 순서** (반드시 이 순서로):
> ```bash
> # 1) 여기서 고친다 (git 이력이 남는다)
> # 2) 바뀐 파일만 올린다
> scp sbc/navi_src/src/<바뀐것> radxa@<robot>:~/navi_src/src/
> # 3) 보드에서 하드웨어 없이 도는 검증부터
> ssh radxa@<robot> 'cd ~/navi_src && make check && make navi'
> # 4) 설치·재시작
> ssh radxa@<robot> 'cd ~/navi_src && sudo make install && sudo systemctl restart navi'
> # 5) md5 로 동기 확인 — 이걸 빼면 "로봇엔 있는데 저장소엔 없는" 코드가 생긴다
> ```
> ⚠️ `make install` 은 `/opt/navi/navi.conf` 를 **덮지 않는다.** 설정 키를 추가했으면
> 직접 복사해야 하고, 그때 **튜닝값이 살아있는지 확인**할 것(gear_ratio·overcurrent_a·
> current_window·wheel sign). 2026-09-03 에 실제로 확인하고 넘어갔다.

> [!warning] 원본 저장소는 여전히 없다
> 전임자 코드이고 보드의 `~/navi_src` 는 git 이 아니다.
> 전임자가 만든 코드이고 보드의 `~/navi_src` 는 **git 저장소가 아니다** — 즉 **보드가 유일본이었다.**
> 보드 장애 시 오늘까지의 작업을 잃지 않기 위한 백업이다.

## 무엇을 언제 떴는가

- 채취 시각: **2026-08-14**
- 경로: 로봇 `~/navi_src/{src,tools,etc,Makefile}` (빌드 산출물 바이너리는 제외 — 보드에서 `make` 로 재생성된다)
- 채취 방법: 개발 PC 는 로봇 전용망(10.10.10.x)에 직접 못 가므로 **IPC 를 경유**했다

```bash
ssh navifra@192.168.100.10 \
  'ssh radxa@10.10.10.64 "cd ~/navi_src && tar cz src tools etc Makefile"' > navi_src.tgz
```

- `/opt/navi/navi.conf`(설치본)과 `etc/navi.conf`(저장소본)는 채취 시점에 **내용이 동일**했다 → 하나만 보관한다. (2026-09-02 재확인: 여전히 동일)

## 여기 담긴 값이 중요한 이유

`etc/navi.conf` 에 **실기에서만 알 수 있는 값**이 들어있다. 문서에 없거나 문서와 다르다:

| 항목 | 값 | 근거 |
|---|---|---|
| 휠 매핑 | `1=FL 2=FR 3=RL 4=RR` | 교체 모터에 ID 재부여(2026-08-14) |
| 휠 sign | 좌 `-1` / 우 `+1` | 실주행에서 네 바퀴가 통째로 반대라 반전 |
| `gear_ratio` | **20.0** | 문서·초기값은 16 이었다. 감속기 실제값 |
| `overcurrent_a` | **10.0** | 기본 5.0 은 무부하 주행에서도 오트립 |
| `current_window` | **15** | 기본 5(=400 ms 창)는 스파이크 3개만 몰려도 트립 |

## `system/` — 보드 밖(OS)에 사는 설정 (2026-09-02 추가)

`navi_src/` 는 소스고, 이건 **OS 에 흩어져 있어서 보드를 다시 깔면 통째로 날아가는 것들**이다.
소스만 백업해두면 로봇은 빌드되지만 **네트워크도 안 붙고 리셋 버튼도 안 먹는다.**

| 파일 | 원래 위치 | 없으면 |
|---|---|---|
| `sudoers.d-navi-restart` | `/etc/sudoers.d/navi-restart` (0440 root:root) | **리셋 버튼이 죽는다** (무인 `systemctl restart navi` 불가) |
| `NetworkManager/ipc-direct.nmconnection` | `/etc/NetworkManager/system-connections/` | 유선(10.10.10.64)이 안 붙는다 |
| `NetworkManager/navi-ap.nmconnection` | 〃 | **AP(`EV-DL_AP`)가 안 뜬다** = 무선 경로 전멸 |
| `NetworkManager/navifra.nmconnection` | 〃 | 로봇이 사내 wifi 로 못 붙는다(`autoconnect=false`, 필요할 때만) |
| `navi.conf.dist` | `/opt/navi/navi.conf.dist` | 기본값 원본 — **우리가 뭘 바꿨는지** 대조용 |

> [!danger] PSK 는 지워서 넣었다
> `.nmconnection` 의 `psk=` 는 `<핸드오프 노트 6장 참조>` 로 치환돼 있다.
> **복원할 때 그 줄을 실제 값으로 바꿔야 한다.** 저장소가 public 이라 넣을 수 없다.

복원:

```bash
sudo install -m 0600 -o root -g root NetworkManager/*.nmconnection \
     /etc/NetworkManager/system-connections/
sudo sed -i 's|^psk=.*|psk=<실제값>|' /etc/NetworkManager/system-connections/navi-ap.nmconnection
sudo nmcli connection reload

sudo install -m 0440 -o root -g root sudoers.d-navi-restart /etc/sudoers.d/navi-restart
sudo visudo -cf /etc/sudoers.d/navi-restart        # parsed OK 확인
```

## ⚠️ 이 보드의 시계는 틀려 있다 — **mtime 으로 변경을 판단하면 안 된다**

RTC 가 없고 AP 가 `method=shared`(상류 인터넷 없음)라 NTP 가 동기화되지 않는다:

```
System clock synchronized: no      NTP service: active
```

그래서 **오늘 쓴 파일도 `Aug 12` 로 찍히고**, journalctl 도 과거 날짜로 나온다.
2026-09-02 에 `find -newermt` 로 변경을 찾으려다 헛돌았다 — **내용을 직접 비교해야 한다**:

```bash
ssh radxa@<robot> 'cd ~/navi_src && tar cz src tools etc Makefile' | tar xz -C /tmp/fresh
diff -r sbc/navi_src /tmp/fresh
```

## 대조 결과 (2026-09-02)

| 대상 | 결과 |
|---|---|
| `~/navi_src` 전체 (src·tools·etc·Makefile) | **파일 목록·내용 100% 동일** — 8/14 이후 소스 변경 없음 |
| `/opt/navi/navi.conf` ↔ `navi_src/etc/navi.conf` | 동일 |
| `/etc/systemd/system/navi.service` ↔ `etc/navi.service` | 동일 |
| `/etc/mosquitto/conf.d/navi.conf` ↔ `etc/navi-mqtt.conf` | 동일 |
| udev · modprobe · rc.local | 커스텀 없음 |
| navi 외 systemd 유닛 | 전부 배포판/벤더 기본 |

## 파일로 백업할 수 없는 것 — **모터 EEPROM**

ID·보레이트·감속비는 파일이 아니라 **각 모터 드라이버의 EEPROM** 에 있다. 모터를 교체하면
소스·설정을 다 복원해도 안 돈다. 재부여 절차(`tools/nuri_ping.py`, navi 를 **멈추고** 실행):

```bash
sudo systemctl stop navi                     # 포트(/dev/ttyS2) 점유 해제
cd ~/navi_src/tools
python3 nuri_ping.py --scan                  # 현재 보레이트·ID 확인 (공장초기값 9600 / ID 0)
python3 nuri_ping.py --baud 9600 --id 0 --set-ratio 20     # 감속비 20:1
python3 nuri_ping.py --baud 9600 --id 0 --set-id 3         # ID (FL1 FR2 RL3 RR4)
python3 nuri_ping.py --baud 9600 --id 3 --set-baud 115200  # ⚠️ 보레이트는 **맨 마지막**
sudo systemctl start navi
```

- EEPROM 쓰기는 **300 ms 이상** 간격을 둔다.
- 한 번에 **한 축만** 버스에 물린다(ID 가 겹치면 응답이 충돌한다).
- 보레이트를 먼저 바꾸면 그 뒤 명령이 안 닿는다 → 순서 엄수.

## 복구 절차 (보드를 새로 깔았을 때)

```bash
# 1) 소스 올리기
scp -r navi_src/* radxa@<robot>:~/navi_src/

# 2) 보드에서 빌드·설치 (크로스컴파일 안 쓴다)
ssh radxa@<robot>
sudo apt install -y build-essential libgpiod-dev libmosquitto-dev nlohmann-json3-dev
cd ~/navi_src && make && sudo make install

# 3) ⚠️ 설치본 설정은 make install 이 덮지 않는다 — 직접 넣어야 한다
sudo cp ~/navi_src/etc/navi.conf /opt/navi/navi.conf
sudo systemctl enable --now navi
```

## 남은 과제

- **원본 저장소 확보** — 전임자/개발 PC 에 있을 것. 찾으면 이 스냅샷은 버리고 거기로 이관한다.
- 찾기 전까지 `deploy.sh` 를 쓰면 안 된다: 문서(§4)상 그 스크립트는 **저장소본 `etc/navi.conf` 를 덮어쓴다** → 위 튜닝값이 날아간다. (실제로 보드에 `deploy.sh` 는 없다 — 문서에만 있다)
