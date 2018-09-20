import RPi.GPIO as GPIO

bottomSignal = 17
middleSignal = 27
topSignal = 22

def servoGPIOSetup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(bottomSignal, GPIO.OUT)

def servoTest():
    # GPIO 17 for PWM with 50Hz
    pwm = GPIO.PWM(bottomSignal, 50)
    # Initialization
    pwm.start(2.5)
    try:
        while True:
            pwm.ChangeDutyCycle(5)
            time.sleep(0.5)
            pwm.ChangeDutyCycle(7.5)
            time.sleep(0.5)
            pwm.ChangeDutyCycle(10)
            time.sleep(0.5)
            pwm.ChangeDutyCycle(12.5)
            time.sleep(0.5)
            pwm.ChangeDutyCycle(10)
            time.sleep(0.5)
            pwm.ChangeDutyCycle(7.5)
            time.sleep(0.5)
            pwm.ChangeDutyCycle(5)
            time.sleep(0.5)
            pwm.ChangeDutyCycle(2.5)
            time.sleep(0.5)
