#pragma once
// 작업등(12V LED ×3) — HAT 의 Q3 스위칭을 GPIO 한 줄로 켠다.
//
// 배선(2026-09-03 실측): 12V IO 3선을 HAT 의 **J4·J5·J7** 에 하나씩 꽂았고,
// `gpiochip1:5`(40핀 16번)를 HIGH 로 올리면 **세 개가 같이 켜진다.**
//
// 🔴 3채널을 **개별 제어할 수 없다** — 회로상 한 트랜지스터(Q3)에 물려 있다.
//    "1번 등만 켜기" 같은 API 를 만들면 안 된다. 하려면 기판을 바꿔야 한다.
//
// 🔴 구동계·워치독과 완전히 무관하다:
//      · 유지 발행이 필요 없다 — 한 번 켜면 끌 때까지 켜져 있다
//      · **e-stop 중에도 켜진다** — 멈춘 뒤에도 상황을 봐야 하기 때문이다
//    그래서 robot.hpp 의 setLed() 는 `cmd_at_` 을 건드리지 않고 accept() 게이트도
//    통과하지 않는다. 건드리면 작업등이 **구동 워치독의 keepalive** 가 되어
//    "명령이 끊기면 정지" 가 깨진다. (이 프로젝트에서 반복 발행이 로봇 쪽 보호를
//     무력화한 사고가 세 번 있었다 — 같은 실수를 여기서 또 만들지 않는다)
//
// 핀은 `led_chip`·`led_line` 으로 바꾼다. **보드가 바뀌면 실측할 것** — 문서 핀을
// 믿고 적었다가 틀린 전례가 있다(actuator.hpp 의 pwmchip 주석 참고).
#include <gpiod.h>

#include <stdexcept>
#include <string>

namespace navi {

class Led {
public:
    Led(const char* chip, unsigned line) {
        chip_ = gpiod_chip_open_by_name(chip);
        if (!chip_) fail(std::string("gpiod_chip_open 실패: ") + chip);
        ln_ = gpiod_chip_get_line(chip_, line);
        if (!ln_) fail("gpiod_chip_get_line 실패: " + std::to_string(line));
        // 초기값 0 — 기본값이 안전이다(설계원칙 ③). 기동만으로 켜지지 않는다.
        if (gpiod_line_request_output(ln_, "navi-led", 0))
            fail("출력 요청 실패 (다른 프로세스가 잡고 있나)");
    }

    ~Led() {
        if (ln_) {
            gpiod_line_set_value(ln_, 0);   // 나갈 때 끈다 — 켜둔 채 죽으면 배터리를 먹는다
            gpiod_line_release(ln_);
        }
        if (chip_) gpiod_chip_close(chip_);
    }

    Led(const Led&) = delete;
    Led& operator=(const Led&) = delete;

    void set(bool on) {
        if (gpiod_line_set_value(ln_, on ? 1 : 0))
            throw std::runtime_error("작업등 GPIO 쓰기 실패");
        on_ = on;
    }

    bool on() const { return on_; }

private:
    [[noreturn]] void fail(const std::string& why) {
        if (ln_) gpiod_line_release(ln_);
        if (chip_) gpiod_chip_close(chip_);
        ln_ = nullptr;
        chip_ = nullptr;
        throw std::runtime_error("작업등: " + why);
    }

    gpiod_chip* chip_ = nullptr;
    gpiod_line* ln_ = nullptr;
    bool on_ = false;
};

}  // namespace navi
