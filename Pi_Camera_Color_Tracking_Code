import cv2
import numpy as np
# import RPi.GPIO as GPIO
import time
from gpiozero import Servo, pins
from gpiozero.pins.pigpio import PiGPIOFactory

# def SetAngle(angle,pwm,servoPIN):
#     print(angle)
#     duty = angle / 18 + 2
#     GPIO.output(servoPIN, True)
#     pwm.ChangeDutyCycle(duty)
#     time.sleep(1/10)
#     GPIO.output(servoPIN, False)
#     pwm.ChangeDutyCycle(0)


servoPIN = 18
servo = Servo(servoPIN,min_pulse_width=1/1000,max_pulse_width=2/1000,pin_factory=PiGPIOFactory())

def deg_to_val(angle):
    # angle = angle%180
    if(angle<0 or angle>180):
        angle = 90
    return (-1 +(angle/90))
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(servoPIN, GPIO.OUT)

# p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
# p.start(0)
# SetAngle(90,p,servoPIN)

# from PCA9685 import PCA9685

# pwm = PCA9685(0x40, debug=False)
# pwm.setPWMFreq(50)
# pwm.setServoPosition(0, 90)
servo.value = deg_to_val(90)
time.sleep(0.1)
cap = cv2.VideoCapture(0)

cap.set(3, 480)
cap.set(4, 320)

_, frame = cap.read()
rows, cols, _ = frame.shape

x_medium = int(cols / 2)
center = int(cols / 2)
position = 90 # degrees
while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # red color
    low_red = np.array([161, 155, 84])
    high_red = np.array([179, 255, 255])
    # low_red = np.array([0, 0, 0])
    # high_red = np.array([50, 50, 50])
    red_mask = cv2.inRange(hsv_frame, low_red, high_red)
    contours,_ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
    
    for cnt in contours:
        (x, y, w, h) = cv2.boundingRect(cnt)
        
        x_medium = int((x + x + w) / 2)
        # print(x_medium)
        break
    
    cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)
    
    cv2.imshow("Frame", frame)
    
    
    key = cv2.waitKey(1)
    
    if key == 27:
        break
    
    # Move servo motor
    if x_medium < center -30:
        position += 5
    elif x_medium > center + 30:
        position -= 5
        
    # pwm.setServoPosition(0, position)
    # SetAngle(position,p,servoPIN
    if(position<15):
        position+= abs(position-15)
    elif(position>170):
        position-=abs(position-170)
    servo.value = deg_to_val(position)
    time.sleep(0.16)


    
    
cap.release()
cv2.destroyAllWindows()
