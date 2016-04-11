import time

from multiprocessing import Process

from Adafruit_PWM_Servo_Driver import PWM


class Servo:
    def __init__(self, channel, min, max, freq):
        self.pwm = PWM(0x40)
        self.pwm.setPWMFreq(freq)

        self.channel = channel
        self.min = min
        self.max = max
        self.range = max - min
        # get middle of the range
        self.current = 0
        self.move_to(0.5)

    def move_to(self, end_pos):
        p = Process(target=move_to_process,
                    args=(self.pwm,
                          self.current,
                          self.channel,
                          self.range,
                          self.min,
                          end_pos))
        p.start()
        self.current = end_pos

    def get_dc_by_range(self, position):
        return self.range * position + self.min


def move_to_process(pwm, current_, channel, range_, min_, end_pos):
    current = current_
    ch = channel
    while current >= end_pos:
        current -= 0.1
        dc = range_ * current + min_
        pwm.setPWM(ch, 0, int(dc))
        time.sleep(0.05)

    while current < end_pos:
        current += 0.1
        dc = range_ * current + min_
        pwm.setPWM(ch, 0, int(dc))
        time.sleep(0.05)
