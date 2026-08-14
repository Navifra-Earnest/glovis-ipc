# Makefile — 보드(ROCK 5A, arm64)에서 빌드한다.
#
#   make            운용 바이너리 + 도구
#   make navi       통합 프로그램만
#   make check      하드웨어 없이 도는 검증 — 커밋 전에 돌린다
#   make install    /opt/navi 에 설치 + systemd 등록 (sudo 필요)
#   make uninstall
#   make clean
#
# 구조
#   src/    코드 (헤더 온리 — 한 폴더에 모아 두면 include 가 단순하다)
#   tools/  진단 스크립트
#   etc/    설정·systemd
#   docs/   문서
#   hw/     KiCad·BOM
#
# VPATH 로 src/ 를 찾게 해서, 규칙마다 경로를 붙이지 않는다.
# 빌드 산출물은 루트에 떨어진다 — 실행할 때 ./navi 로 바로 쓸 수 있게.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
PREFIX   ?= /opt/navi

# 헤더가 하나라도 바뀌면 전부 다시 빌드한다. 헤더 온리 구성이라 의존 관계를
# 일일이 적는 것보다 이게 정확하고, 파일 수가 적어 비용도 크지 않다.
VPATH := src
HDRS  := $(wildcard src/*.hpp)
CXXFLAGS += -Isrc

# ── 무엇을 만드나 ───────────────────────────────────────────────────
# 운용:  현장에 올라가는 것
# 도구:  점검·진단용. 운용에는 필요 없지만 있으면 문제를 빨리 잡는다
# 검증:  하드웨어 없이 도는 자체 시험
RUNTIME  := navi
TOOLS    := nuri_demo cam_demo cam_view actuator_demo hall_demo pos_demo
TESTS    := nuri_selftest overcurrent_test

# 열화상 자체는 v4l2 로 읽으므로 TmSDK 없이도 빌드·동작한다.
# SDK 가 있으면 FFC(셔터 보정)와 SDK 컬러맵, 그리고 tm_* 진단 도구가 추가로 열린다.
# 배포판에 libTmCore.so 심볼릭 링크가 없어 .so 를 직접 지정해야 한다.
TMSDK_LIB := /usr/lib/TmSDK/libTmCore.2.1.0.so
ifneq ($(wildcard $(TMSDK_LIB)),)
  TOOLS      += tm_probe tm_watch tm_view tm_ffc_trace
  TMSDK_LINK := $(TMSDK_LIB) -Wl,-rpath,/usr/lib/TmSDK
  NAVI_TMSDK := -DNAVI_HAS_TMSDK $(TMSDK_LINK)
else
  $(info [i] TmSDK 없음 — 열화상은 v4l2 로 돈다. FFC·SDK 컬러맵·tm_* 도구만 빠진다)
endif

BINS := $(RUNTIME) $(TOOLS) $(TESTS)

.PHONY: all runtime tools check install uninstall clean help

all: $(BINS)
	@echo "빌드 완료 — 운용: $(RUNTIME) / 도구: $(words $(TOOLS))개 / 검증: $(words $(TESTS))개"

runtime: $(RUNTIME)
tools:   $(TOOLS)

# ── 운용 ────────────────────────────────────────────────────────────
navi: main.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -o $@ $< -lgpiod -lpthread -lmosquitto -lrockchip_mpp $(NAVI_TMSDK)

# ── 검증 ────────────────────────────────────────────────────────────
# 둘 다 하드웨어(모터·카메라) 없이 돈다. 다만 nuri_selftest 는 nurirobot.hpp 를
# 포함하느라 **libgpiod 헤더**가 필요해서, 개발 PC 에는 안 깔려 있을 수 있다.
# 그래서 libgpiod 유무를 보고 자동으로 갈라준다 — 개발 PC 에서도 `make check` 가 돌게.
#
#   nuri_selftest    : RS485 프레임·체크섬이 프로토콜 문서 예제와 일치하는지
#   overcurrent_test : 과전류 판정 — 스파이크에 안 흔들리고 지속 과전류만 잡는지
HAVE_GPIOD := $(shell test -e /usr/include/gpiod.h && echo 1)

check: overcurrent_test
	@echo "── 과전류 판정"
	@./overcurrent_test
ifeq ($(HAVE_GPIOD),1)
	@$(MAKE) --no-print-directory nuri_selftest
	@echo ""
	@echo "── 프로토콜 프레임"
	@./nuri_selftest
else
	@echo ""
	@echo "[i] libgpiod 없음 — 프로토콜 검증은 건너뛴다 (보드에서 make check 로 확인)"
endif

nuri_selftest: nuri_selftest.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -o $@ $< -lgpiod

overcurrent_test: overcurrent_test.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

# ── 점검 도구 ───────────────────────────────────────────────────────
nuri_demo: nuri_demo.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -o $@ $< -lgpiod

cam_demo: cam_demo.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -o $@ $<

cam_view: cam_view.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -o $@ $< -lpthread

actuator_demo: actuator_demo.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -o $@ $<

hall_demo: hall_demo.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -o $@ $< -lgpiod -lpthread

pos_demo: pos_demo.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -o $@ $< -lgpiod -lpthread

tm_probe: tm_probe.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(TMSDK_LINK)

tm_watch: tm_watch.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(TMSDK_LINK)

tm_view: tm_view.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(TMSDK_LINK) -lpthread

# ── 설치 ────────────────────────────────────────────────────────────
# 바이너리와 설정을 $(PREFIX) 에 두고 systemd 에 등록한다.
#
# ⚠ navi.conf 는 **덮어쓰지 않는다.** 현장에서 맞춘 값(휠 sign, DE 핀, 차체 치수)이
#   들어 있어서, 업데이트할 때마다 날아가면 안 된다. 새 항목은 navi.conf.dist 를 보고
#   손으로 옮긴다.
install: $(RUNTIME) $(TESTS)
	@echo "── 검증 먼저"
	@./nuri_selftest >/dev/null && ./overcurrent_test >/dev/null && echo "   통과" \
		|| (echo "   🔴 검증 실패 — 설치 중단"; exit 1)
	install -d $(PREFIX) $(PREFIX)/bin
	install -m755 $(RUNTIME) $(PREFIX)/bin/
	install -m755 $(wildcard tools/nuri_ping.py tools/nuri_spin.py tools/nuri_ctlparam.py) $(PREFIX)/bin/
	install -m644 etc/navi.conf $(PREFIX)/navi.conf.dist
	@test -f $(PREFIX)/navi.conf \
		&& echo "[i] navi.conf 는 그대로 둔다 (새 항목은 navi.conf.dist 참고)" \
		|| install -m644 etc/navi.conf $(PREFIX)/navi.conf
	@# 보존한 설정이 낡으면 조용히 옛 동작을 한다 — 실제로 de_guard_us 가 5A 로 옮긴 뒤에도
	@# 3A 값(300)으로 남아 있었다. 빠진 키를 여기서 알려준다.
	@# 키에 숫자가 들어간다(rs485_baud) — [a-z_]+ 만 쓰면 'rs' 로 잘려 늘 빠진 것처럼 보인다
	@missing=$$(grep -oE '^[a-z0-9_]+[[:space:]]*=' etc/navi.conf | tr -d ' =' | sort -u | while read -r k; do \
	    grep -qE "^$$k[[:space:]]*=" $(PREFIX)/navi.conf || echo "$$k"; done); \
	  [ -z "$$missing" ] || { echo "[!] 설치된 navi.conf 에 없는 항목 — navi.conf.dist 에서 옮길 것:"; \
	                          echo "$$missing" | sed 's/^/      /'; }
	install -m644 etc/navi.service /etc/systemd/system/
	systemctl daemon-reload
	@echo ""
	@echo "설치 완료: $(PREFIX)"
	@echo "  자동시작 켜기 : sudo systemctl enable --now navi"
	@echo "  상태 보기     : systemctl status navi / journalctl -u navi -f"
	@echo "  설정          : $(PREFIX)/navi.conf"

uninstall:
	-systemctl disable --now navi 2>/dev/null
	rm -f /etc/systemd/system/navi.service
	systemctl daemon-reload
	@echo "제거 완료. 설정은 남겨 둔다: $(PREFIX)/navi.conf"

clean:
	rm -f $(BINS)

help:
	@echo "빌드"
	@echo "  make          운용 + 도구 + 검증"
	@echo "  make runtime  navi 만"
	@echo "  make check    하드웨어 없이 검증 (커밋 전)"
	@echo ""
	@echo "설치 (sudo)"
	@echo "  sudo make install     $(PREFIX) 에 설치 + systemd 등록"
	@echo "  sudo systemctl enable --now navi"
	@echo ""
	@echo "실행 — sysfs(PWM·GPIO)를 쓰므로 sudo 필요"
	@echo "  sudo ./navi           상태 모니터 (모터를 돌리지 않는다 — 기본이 안전)"
	@echo "  sudo ./navi mqtt      운용 모드"
	@echo "  sudo ./navi spin 5 3  | turn | body | act | wheelcheck | wdtest"
	@echo ""
	@echo "🔴 모터 구동 전 전자 브레이크를 풀 것 — 인출선 2가닥에 DC 24V"
	@echo "   안 풀고 돌리면 무부하인데 정격의 6~9배가 흘러 모터가 탄다"

# CDC(FFC) 프로토콜을 뜨는 진단 도구. 명령 바이트는 이미 알아냈지만(thermal.hpp),
# 펌웨어가 바뀌어 형식이 달라지면 이걸로 다시 확인한다.
#   sudo strace -f -y -e trace=write,read -s 300 -o /tmp/t.log ./tm_ffc_trace
tm_ffc_trace: tm_ffc_trace.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(TMSDK_LINK)
