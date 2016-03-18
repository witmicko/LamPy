from threading import Thread

import RPi.GPIO as GPIO
import time

from multiprocessing import Pipe, Process


def move_servos(pipe):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(12, GPIO.OUT)
    GPIO.setup(11, GPIO.OUT)
    servo_1 = GPIO.PWM(12, 50)
    servo_2 = GPIO.PWM(11, 50)

    servo_1.start(0)
    servo_2.start(0)
    servo_1_dc_old = 0
    servo_2_dc_old = 0
    while True:
        x = pipe.recv()
        print 'pipe', x
        try:
            arg_1, arg_2 = x.split(" ")
            servo = int(arg_1)
            dc = float(arg_2)
            if servo == 1:
                print 'set dc_1', dc
                servo_1_dc = dc
                servo_1.ChangeDutyCycle(dc)
                while servo_1_dc > servo_1_dc_old:
                    servo_1_dc_old += 0.1
                    if servo_1_dc_old > 100 or servo_1_dc_old < 0:
                        break
                    servo_1.ChangeDutyCycle(servo_1_dc_old)
                    time.sleep(0.02)

                while servo_1_dc < servo_1_dc_old:
                    servo_1_dc_old -= 0.1
                    if servo_1_dc_old > 100 or servo_1_dc_old < 0:
                        break
                    servo_1.ChangeDutyCycle(servo_1_dc_old)
                    time.sleep(0.02)

            if servo == 2:
                print 'set dc_2', dc
                servo_2_dc = dc
                if servo_2_dc > servo_2_dc_old:
                    while servo_2_dc > servo_2_dc_old:
                        servo_2_dc_old += 0.1
                        if servo_2_dc_old > 100 or servo_2_dc_old < 0:
                            break
                        servo_2.ChangeDutyCycle(servo_2_dc_old)
                        time.sleep(0.01)
                else:
                    while servo_2_dc < servo_2_dc_old:
                        servo_2_dc_old -= 0.1
                        if servo_2_dc_old > 100 or servo_2_dc_old < 0:
                            break
                        servo_2.ChangeDutyCycle(servo_2_dc_old)
                        time.sleep(0.01)
                servo_2_dc_old = servo_2_dc
        except ValueError as e:
            print 'breaking servo process', e
            break

    servo_1.stop()
    servo_2.stop()
    GPIO.cleanup()


end = False


def get_input(pipe, servos):
    global end
    while True:
        x = raw_input("?")
        if x == 'q':
            end = True
            break
        elif x == 's':
            s = Process(target=move_servos, args=(servos,))
            s.daemon = True
            s.start()
        else:
            pipe.send(x)


servos, keyboard = Pipe()
t1 = Process(target=move_servos, args=(servos,))

t2 = Thread(target=get_input, args=(keyboard, servos))
t1.daemon = True
t2.daemon = True
t2.start()
t1.start()

while True:
    if end:
        break
