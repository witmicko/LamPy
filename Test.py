import time

from Adafruit_PWM_Servo_Driver import PWM
from Servo import Servo

# mg995
servo = Servo(channel=0, min= 115, max=500, freq=50) # mg995

# sg90
servo = Servo(channel=1, min=180, max=800, freq=50)

# time.sleep(1)
# servo.move_to(1)
time.sleep(1)
#
# servo.move_to(0)


pwm = PWM(0x40)
pwm.setPWMFreq(50)
pwm.softwareReset()
