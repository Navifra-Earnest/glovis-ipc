#!/usr/bin/env python3
"""U6 구동 + 홀센서 카운트 — ROCK 3A.

  헤더 24 → PWM1A     : pwmchip1 (PWM13)
  헤더 26 → PHA       : GPIO4_D1 = 153
  헤더 22 → EN/nFAULT : GPIO0_C1 = 17  (읽기 전용)
  헤더 35 → Hall_S1   : gpiochip3 line 4
  헤더 31 → Hall_S2   : gpiochip3 line 0
  헤더 37 → VA        : SARADC_VIN5

사용: sudo python3 u6_hall_test.py [듀티%] [초]
      sudo python3 u6_hall_test.py --hall 5      모터 없이 홀만 5초 감시
"""
import subprocess
import sys
import time
from pathlib import Path

PWMCHIP = Path('/sys/class/pwm/pwmchip1')
PWM = PWMCHIP / 'pwm0'
GPIO = Path('/sys/class/gpio')
PHA, EN = 153, 17
HALL_CHIP, HALL_S1, HALL_S2 = 'gpiochip3', 4, 0
ADC = Path('/sys/bus/iio/devices/iio:device0/in_voltage5_raw')

PERIOD_NS = 50_000
MAX_DUTY = 100
ADC_VREF, ADC_MAX = 1.8, 1023
VA_PER_A = 0.5
I_LIMIT_SOFT = 2.0


def read(p):
    try:
        return p.read_text().strip()
    except OSError:
        return None


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
    return None if raw is None else int(raw) * ADC_VREF / ADC_MAX / VA_PER_A


def fault():
    return gpio_get(EN) == '0'


def hall_watch(seconds):
    """gpiomon을 백그라운드로 띄워 엣지를 수집한다."""
    return subprocess.Popen(
        ['timeout', str(seconds), 'gpiomon', '--format=%e %o %s.%n',
         HALL_CHIP, str(HALL_S1), str(HALL_S2)],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)


def hall_report(proc, label):
    out, _ = proc.communicate()
    lines = [l.split() for l in out.strip().splitlines() if l.strip()]
    s1 = sum(1 for l in lines if l[1] == str(HALL_S1))
    s2 = sum(1 for l in lines if l[1] == str(HALL_S2))
    print(f'  [{label}] 엣지 총 {len(lines)}개  (S1 {s1} / S2 {s2})')
    if lines:
        # 채널 교대 패턴이면 쿼드러처, 한쪽만이면 단일 채널
        seq = [l[1] for l in lines[:40]]
        alt = sum(1 for a, b in zip(seq, seq[1:]) if a != b)
        print(f'      앞 {len(seq)}개 채널 교대율 {alt}/{len(seq)-1}'
              f' → {"쿼드러처(방향 판별 가능)" if alt > len(seq)*0.5 else "단일채널 우세"}')
    return len(lines)


def pwm_start(duty_pct):
    if not PWM.exists():
        write(PWMCHIP / 'export', 0)
        for _ in range(50):
            if (PWM / 'period').exists():
                break
            time.sleep(0.02)
    for p, v in ((PWM / 'enable', 0), (PWM / 'duty_cycle', 0)):
        try:
            write(p, v)
        except OSError:
            pass
    write(PWM / 'period', PERIOD_NS)
    if read(PWM / 'polarity') != 'normal':
        write(PWM / 'polarity', 'normal')
    write(PWM / 'duty_cycle', int(PERIOD_NS * duty_pct / 100))
    write(PWM / 'enable', 1)


def pwm_stop():
    if PWM.exists():
        try:
            write(PWM / 'duty_cycle', 0)
            write(PWM / 'enable', 0)
        except OSError:
            pass


def run(direction, duty_pct, seconds):
    name = 'EXT(전진)' if direction == 0 else 'RET(후진)'
    print(f'\n▶ PHA={direction} {name}, duty={duty_pct}%, {seconds}s')
    write(GPIO / f'gpio{PHA}' / 'value', direction)
    mon = hall_watch(seconds + 0.5)
    time.sleep(0.2)
    pwm_start(duty_pct)
    t0 = time.monotonic()
    peak = 0.0
    while time.monotonic() - t0 < seconds:
        if fault():
            pwm_stop()
            hall_report(mon, name)
            print('  🔴 FAULT — 정지')
            return False
        i = current_a()
        peak = max(peak, i)
        if i > I_LIMIT_SOFT:
            pwm_stop()
            hall_report(mon, name)
            print(f'  🔴 전류 {i:.2f}A > {I_LIMIT_SOFT}A — 정지')
            return False
        time.sleep(0.05)
    pwm_stop()
    print(f'  정지. 최대 전류 {peak:.2f} A')
    hall_report(mon, name)
    return True


def status():
    print(f'  EN/nFAULT : {gpio_get(EN)}   PHA : {gpio_get(PHA)}   '
          f'VA : {current_a():.2f} A')


def main():
    args = sys.argv[1:]
    gpio_export(EN, 'in')
    gpio_export(PHA, 'out')

    if '--hall' in args:
        secs = float(next((a for a in args if not a.startswith('-')), 5))
        print(f'모터 정지 상태로 홀만 {secs}s 감시 — 축을 손으로 움직여 보세요')
        hall_report(hall_watch(secs), '수동')
        return

    nums = [a for a in args if not a.startswith('-')]
    duty = int(nums[0]) if nums else 30
    secs = float(nums[1]) if len(nums) > 1 else 3.0
    if duty > MAX_DUTY:
        sys.exit(f'듀티 {duty}% > 상한 {MAX_DUTY}%')

    print('=== 초기 상태 ===')
    status()
    if fault():
        sys.exit('🔴 EN/nFAULT = LOW. VBAT / 3.3V 확인 필요')

    print('\n=== 기준: 모터 정지 상태 홀 노이즈 ===')
    base = hall_report(hall_watch(1.5), '정지')
    if base > 0:
        print('  ⚠ 정지 상태에서 엣지 발생 — 노이즈 또는 채터링')

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
