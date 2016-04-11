from threading import Thread

import time

from multiprocessing import Pipe, Process
from Adafruit_PWM_Servo_Driver import PWM

pwm = PWM(0x40)
pwm.setPWMFreq(50)


def move_servos(pipe):
    servo_1 = 0
    servo_2 = 1

    servo_1_dc_old = 0
    servo_2_dc_old = 0
    while True:
        x = pipe.recv()
        try:
            arg_1, arg_2 = x.split(" ")
            on = int(arg_1)
            off = int(arg_2)
            pwm.setPWM(servo_1, on, off)
        except ValueError as e:
            print 'breaking servo process', e
            break



end = False
# end_position = need to checkout each servo i think
# def move_servo(on, end_position):
#     end = 0
#     while True

servos, keyboard = Pipe()
t1 = Process(target=move_servos, args=(servos,))
t1.daemon = True
t1.start()
end = False
while True:
    x = raw_input("?")
    if x == 'q':
        end = True
        pwm.softwareReset()
        break
    elif x == 's':
        s = Process(target=move_servos, args=(servos,))
        s.daemon = True
        s.start()
    else:
        keyboard.send(x)
        # time.sleep(2)
    if end:
        break
