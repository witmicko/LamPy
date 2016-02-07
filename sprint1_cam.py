# imports
from threading import Thread, Lock, Event
from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
import time
import sys
import signal
import cv2
import servo_stepper
import pygame.mixer
from pygame.mixer import Sound

pygame.mixer.init()

# constants
# turns off/on camera display, very expensive
DEBUG = False
# LEDs to indicate face found
RED_pin = 40
GREEN_pin = 38

# Servos duty cycle ranges
servos_x_limits = {'min': 1.7, 'max': 10.6, 'range': 8.9}
servos_y_limits = {'min': 2.4, 'max': 11.4, 'range': 9}

# vision
RES_X = 100
RES_Y = 100
FPS = 20
FACE_CASCADE = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml')

# stats
servo_thread = 0
audio_thread = 0
vision_thread = 0
start = time.time()


class Runner:
    def __init__(s):
        # s.sound_x = Sound("bass_x.wav")
        # s.sound_y = Sound("bass_y.wav")
        s.sound_x = Sound("x_1.wav")
        s.sound_y = Sound("y_1.wav")

        # servos
        s.servos_curr_pos_x = 0
        s.servos_curr_pos_y = 0
        s.servo_x_curr_pos_ranged = 0.5
        s.servo_y_curr_pos_ranged = 0.25
        s.servo_x = None
        s.servo_y = None

        # camera
        s.camera = PiCamera()
        s.camera.resolution = (RES_X, RES_Y)
        s.camera.framerate = FPS
        s.rawCapture = PiRGBArray(s.camera, size=(RES_X, RES_Y))
        s.face_x = 0.5
        s.face_y = 0.5

        # IPC
        s.mutex = Lock()
        # an event to synch servo and vision thread
        s.event = Event()

    def servos(self):
        while True:
            # wait for event to be set in vision thread
            self.event.wait()
            global servo_thread
            # acquire mutex loc for global vars, releases it after its done
            with self.mutex:
                # reset event so it can run again
                self.event.clear()
                # increment stats counter
                servo_thread += 1
                # using detected face position get value by how much to move servos
                delta_x = servo_stepper.step_x(self.face_x)
                delta_y = servo_stepper.step_y(self.face_y)

                # update current servos positions
                self.servo_x_curr_pos_ranged += delta_x
                self.servo_y_curr_pos_ranged += delta_y

                # safety checks to make sure that servos will stay within their dc ranges
                if self.servo_x_curr_pos_ranged > 1:
                    self.servo_x_curr_pos_ranged = 0.9
                elif self.servo_x_curr_pos_ranged < 0:
                    self.servo_x_curr_pos_ranged = 0.1
                if self.servo_y_curr_pos_ranged > 1:
                    self.servo_y_curr_pos_ranged = 0.9
                elif self.servo_y_curr_pos_ranged < 0:
                    self.servo_y_curr_pos_ranged = 0.1

                # translate our 0-1 servo range to its actual duty cycle value corresponding to that position
                dc_x, dc_y = calculate_duty_cycle(self.servo_x_curr_pos_ranged, self.servo_y_curr_pos_ranged)

                # if we need to move - change dc, update current dc, play sound and set volume based on distance
                if dc_x != self.servos_curr_pos_x:
                    self.servo_x.ChangeDutyCycle(dc_x)
                    self.servos_curr_pos_x = dc_x
                    self.sound_x.play(loops=-1)
                    vol = abs(delta_x * 15)
                    self.sound_x.set_volume(vol)
                # stop sound if we re not moving anything
                else:
                    self.sound_x.stop()

                if dc_y != self.servos_curr_pos_y:
                    self.servo_y.ChangeDutyCycle(dc_y)
                    self.servos_curr_pos_y = dc_y
                    self.sound_y.play(loops=-1)
                    vol = abs(delta_y * 15)
                    self.sound_y.set_volume(vol)
                else:
                    self.sound_y.stop()

    def vision(self):
        while True:
            global vision_thread
            # for frames in camera feed
            for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
                # acquire mutex loc for global vars, releases it after its done
                with self.mutex:
                    # update stats
                    vision_thread += 1

                    # get munpy array out of the frame
                    image = frame.array
                    # binarize so its easier/cheaper to process
                    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

                    # camera is mounted upside down so we need to flip the image
                    flipped = gray
                    cv2.flip(src=gray, dst=flipped, flipCode=0)

                    # detect faces
                    faces = FACE_CASCADE.detectMultiScale(image=gray,
                                                          scaleFactor=1.1,
                                                          minNeighbors=4)
                    cface = [RES_X / 2, RES_Y / 2]
                    if len(faces) > 0:
                        # get details for 1st face
                        x, y, w, h = faces[0]

                        # calculate its centre
                        cface = [(w / 2 + x), (h / 2 + y)]

                        # translate into 0-1 range
                        # percentage = (value - min) / (max - min)
                        self.face_x = (cface[0] * 1.0) / RES_X
                        self.face_y = (cface[1] * 1.0) / RES_Y

                        # set LEDs
                        GPIO.output(GREEN_pin, GPIO.HIGH)
                        GPIO.output(RED_pin, GPIO.LOW)
                    else:
                        # not detected so set it to centre
                        self.face_x = 0.5
                        self.face_y = 0.5
                        # set LEDs
                        GPIO.output(RED_pin, GPIO.HIGH)
                        GPIO.output(GREEN_pin, GPIO.LOW)

                    if DEBUG:
                        cv2.circle(flipped, (cface[0], cface[1]), 5, (255, 0, 0))
                        cv2.imshow("Frame", flipped)
                        cv2.waitKey(33)
                    self.rawCapture.truncate(0)
                    self.event.set()

    def run(self):
        self.init_servos()

        # start servo and vision thread
        t = Thread(target=self.servos)
        t.daemon = True
        t.start()

        t = Thread(target=self.vision)
        t.daemon = True
        t.start()

        while True:
            pass

    def init_servos(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(12, GPIO.OUT)
        GPIO.setup(11, GPIO.OUT)
        # LEDs
        GPIO.setup(GREEN_pin, GPIO.OUT)
        GPIO.setup(RED_pin, GPIO.OUT)
        self.servo_x = GPIO.PWM(12, 50)  # channel=12 frequency=50Hz
        self.servo_y = GPIO.PWM(11, 50)  # channel=11 frequency=50Hz
        self.servo_x.start(0)
        self.servo_y.start(0)


def calculate_duty_cycle(face_x, face_y):
    # translates 0-1 range into duty cycle based on the dc range of servos
    x = (servos_x_limits['range'] * face_x) + servos_x_limits['min']
    y = (servos_y_limits['range'] * face_y) + servos_y_limits['min']
    return x, y


def exit_gracefully():
    # clean up GPIO and camera on ctrl+c exit
    # ref: stackoverflow

    signal.signal(signal.SIGINT, original_sigint)
    GPIO.output(RED_pin, GPIO.LOW)
    GPIO.output(GREEN_pin, GPIO.LOW)
    end = time.time()
    print
    print 'audio ', audio_thread
    print 'servo', servo_thread
    print 'vision', vision_thread
    print 'fps', vision_thread / round(end - start)

    sys.exit(1)
    # restore the exit gracefully handler here
    signal.signal(signal.SIGINT, exit_gracefully)


if __name__ == '__main__':
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)
    Runner().run()
