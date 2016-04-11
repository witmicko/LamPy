import time

from Adafruit_PWM_Servo_Driver import PWM
from Servo import Servo


if __name__ == '__main__':
    # mg995
    servo = Servo(channel=0, min=115, max=460, freq=50) # mg995

    # sg90
    servo1 = Servo(channel=1, min=190, max=550, freq=50)

    # sg90 2
    servo2 = Servo(channel=2, min=190, max=550, freq=50)

    time.sleep(0.5)

    for i in range(11):
        # print i
        dc = i / 10.0
        print dc
        servo.move_to(dc)
        servo1.move_to(dc)
        servo2.move_to(dc)
        time.sleep(0.3)

    # time.sleep(0.5)
    # servo.move_to(0)
    # servo1.move_to(0)
    # servo2.move_to(0)
    #
    time.sleep(1)
    pwm = PWM(0x40)
    pwm.setPWMFreq(50)
    pwm.softwareReset()
