#!/usr/bin/env python3
"""U6 (STSPIN948) PWM 제어 확인 — ROCK 3A + Radxa_Rock5A_Hat_V1.

정해진 시퀀스만 돌고 반드시 정지한다. 무한 루프 없음.

  헤더 24 → PWM1A  : pwmchip1 (fe700010 = PWM13)
  헤더 26 → PHA    : GPIO4_D1 = 153  (방향)
  헤더 22 → EN/nFAULT : GPIO0_C1 = 17  ← 읽기 전용. 절대 출력으로 쓰지 말 것
  헤더 37 → VA     : SARADC_VIN5 (전류 모니터, 0.5 V/A)

사용: sudo python3 u6_pwm_test.py [듀티%] [초]
      sudo python3 u6_pwm_test.py --check      상태만 확인
"""
import sys
import time
from pathlib import Path

PWMCHIP = Path('/sys/class/pwm/pwmchip1')
PWM = PWMCHIP / 'pwm0'
GPIO = Path('/sys/class/gpio')
PHA = 153          # 헤더 26, GPIO4_D1
EN = 17            # 헤더 22, GPIO0_C1 (입력 전용)
ADC = Path('/sys/bus/iio/devices/iio:device0/in_voltage5_raw')

PERIOD_NS = 50_000     # 20 kHz — 가청 위, STSPIN948 최소펄스 280ns 대비 충분
MAX_DUTY = 100         # 액추에이터가 48V 정격이라 100%까지 정상 범위
ADC_VREF, ADC_MAX = 1.8, 1023
VA_PER_A = 0.5         # R35 50mΩ × A_CL 10
I_LIMIT_SOFT = 2.0     # 액추에이터(TiMOTION MA5) 정격. HW 제한 3.63A보다 낮게 잡는다


def read(p, default=None):
    try:
        return p.read_text().strip()
    except OSError:
        return default


def write(p, v):
    p.write_text(str(v))


def gpio_export(n, direction):
    if not (GPIO / f'gpio{n}').exists():
        write(GPIO / 'export', n)
        time.sleep(0.05)
    write(GPIO / f'gpio{n}' / 'direction', direction)


def gpio_get(n):
    return read(GPIO / f'gpio{n}' / 'value')


def current_a():
    raw = read(ADC)
    if raw is None:
        return None
    return int(raw) * ADC_VREF / ADC_MAX / VA_PER_A


def fault():
    """EN/nFAULT가 L이면 fault 또는 UVLO(=VBAT 미인가)."""
    return gpio_get(EN) == '0'


def status():
    print(f'  EN/nFAULT : {gpio_get(EN)}   (1=정상, 0=fault 또는 UVLO)')
    print(f'  PHA       : {gpio_get(PHA)}')
    raw = read(ADC)
    print(f'  VA(ADC)   : raw={raw}  ≈ {current_a():.2f} A')


def pwm_start(duty_pct):
    if not PWM.exists():
        write(PWMCHIP / 'export', 0)
        for _ in range(50):                    # export 직후 노드 생성 대기
            if (PWM / 'period').exists():
                break
            time.sleep(0.02)
    # ponytail: period=0이면 enable/duty 쓰기가 EINVAL. 순서를 지킨다
    for p, v in ((PWM / 'enable', 0), (PWM / 'duty_cycle', 0)):
        try:
            write(p, v)
        except OSError:
            pass
    write(PWM / 'period', PERIOD_NS)
    # 기본값이 inversed면 duty 10%가 실제 90%로 나간다.
    # period 설정 후·enable 전에만 변경 가능
    if read(PWM / 'polarity') != 'normal':
        write(PWM / 'polarity', 'normal')
    write(PWM / 'duty_cycle', int(PERIOD_NS * duty_pct / 100))
    write(PWM / 'enable', 1)


def check_polarity():
    if PWM.exists() and read(PWM / 'polarity') != 'normal':
        sys.exit('🔴 polarity가 normal이 아니다. duty가 반전되어 나간다. 중단.')


def pwm_stop():
    if PWM.exists():
        try:
            write(PWM / 'duty_cycle', 0)
            write(PWM / 'enable', 0)
        except OSError:
            pass


def run(direction, duty_pct, seconds):
    """한 방향으로 duty_pct 로 seconds 동안. fault 뜨면 즉시 정지."""
    print(f'\n▶ PHA={direction}, duty={duty_pct}%, {seconds}s')
    write(GPIO / f'gpio{PHA}' / 'value', direction)
    pwm_start(duty_pct)
    t0 = time.monotonic()
    peak = 0.0
    while time.monotonic() - t0 < seconds:
        if fault():
            pwm_stop()
            print('  🔴 FAULT 감지 — 즉시 정지')
            return False
        i = current_a()
        peak = max(peak, i)
        # 하드웨어 전류제한(3.63A)이 액추에이터 정격(2A)보다 높다. 소프트로 막는다
        if i > I_LIMIT_SOFT:
            pwm_stop()
            print(f'  🔴 전류 {i:.2f}A > 소프트제한 {I_LIMIT_SOFT}A — 즉시 정지')
            return False
        print(f'    t={time.monotonic()-t0:4.1f}s  I={i:5.2f} A')
        time.sleep(0.05)
    pwm_stop()
    print(f'  정지. 최대 전류 {peak:.2f} A')
    return True


def main():
    args = sys.argv[1:]
    check_only = '--check' in args
    nums = [a for a in args if not a.startswith('-')]
    duty = int(nums[0]) if nums else 10
    secs = float(nums[1]) if len(nums) > 1 else 1.0

    if duty > MAX_DUTY:
        sys.exit(f'듀티 {duty}% > 상한 {MAX_DUTY}%. 스크립트의 MAX_DUTY를 고쳐라.')

    gpio_export(EN, 'in')
    gpio_export(PHA, 'out')

    print('=== 초기 상태 ===')
    status()

    if fault():
        print('\n🔴 EN/nFAULT = LOW')
        print('   U6가 fault 또는 UVLO 상태다. 다음을 확인할 것:')
        print('     - VBAT(48V)가 J1에 인가되어 있는가')
        print('     - HAT의 +3.3V(U4 출력)가 살아있는가  → C25 양단 측정')
        print('     - +5V_REF(U2 벅 출력)가 나오는가')
        sys.exit(1)

    if check_only:
        return

    try:
        if run(0, duty, secs):
            time.sleep(0.5)
            run(1, duty, secs)
    finally:
        pwm_stop()
        print('\n=== 종료 상태 ===')
        status()


if __name__ == '__main__':
    main()
