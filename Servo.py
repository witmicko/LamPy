import time

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
        self.current = 1
        self.move_to(0.5)

    def move_to(self, end_pos):
        current = self.current
        ch = self.channel
        while current >= end_pos:
            current -= 0.1
            dc = self.range * current + self.min
            self.pwm.setPWM(ch, 0, int(dc))
            time.sleep(0.05)

        while current < end_pos:
            current += 0.1
            dc = self.range * current + self.min
            self.pwm.setPWM(ch, 0, int(dc))
            time.sleep(0.05)

        self.current = current



    def get_dc_by_range(self, position):
        return self.range * position + self.min
