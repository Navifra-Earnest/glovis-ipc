# 로봇(SBC) `navi` 스냅샷 — **백업이지 원본이 아니다**

> [!warning]
> 이건 로봇 보드에서 그대로 떠온 **사본**이다. 원본 저장소가 아니고, 여기서 고쳐도 로봇에 반영되지 않는다.
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

- `/opt/navi/navi.conf`(설치본)과 `etc/navi.conf`(저장소본)는 채취 시점에 **내용이 동일**했다 → 하나만 보관한다.

## 여기 담긴 값이 중요한 이유

`etc/navi.conf` 에 **실기에서만 알 수 있는 값**이 들어있다. 문서에 없거나 문서와 다르다:

| 항목 | 값 | 근거 |
|---|---|---|
| 휠 매핑 | `1=FL 2=FR 3=RL 4=RR` | 교체 모터에 ID 재부여(2026-08-14) |
| 휠 sign | 좌 `-1` / 우 `+1` | 실주행에서 네 바퀴가 통째로 반대라 반전 |
| `gear_ratio` | **20.0** | 문서·초기값은 16 이었다. 감속기 실제값 |
| `overcurrent_a` | **10.0** | 기본 5.0 은 무부하 주행에서도 오트립 |
| `current_window` | **15** | 기본 5(=400 ms 창)는 스파이크 3개만 몰려도 트립 |

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
