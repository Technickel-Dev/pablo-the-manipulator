import RPi.GPIO as GPIO
import time

class ServoGPIO():
    # GPIO pins for the different servos
    signalBottom = 17
    signalMiddle = 27
    signalTop = 22
    # Frequency of 50Hz gives period of 20 ms
    frequency = 50

    def __init__(self):
        # Use Broadcom SOC channel numbers for pins
        GPIO.setmode(GPIO.BCM)
        # Set GPIO pins to optput
        GPIO.setup(self.signalBottom, GPIO.OUT)
        GPIO.setup(self.signalMiddle, GPIO.OUT)
        GPIO.setup(self.signalTop, GPIO.OUT)
        # Initialize all the PWMs with the frequency
        self.pwmBottom = GPIO.PWM(self.signalBottom, self.frequency)
        self.pwmMiddle = GPIO.PWM(self.signalMiddle, self.frequency)
        self.pwmTop = GPIO.PWM(self.signalTop, self.frequency)
        # Neutral for HS-322HD is 1.5ms
        # 2.5% (0 degrees) 12.5% (180 degrees)
        # Set all servos to starting position
        self.pwmBottom.start(2.5)
        self.pwmMiddle.start(2.5)
        self.pwmTop.start(2.5)
        time.sleep(2)

    def servoGPIOCleanup(self):
        GPIO.cleanup()

    def servosSet(self, pwm1, pwm2, pwm3, dutyCycle):
        pwm1.ChangeDutyCycle(dutyCycle)
        pwm2.ChangeDutyCycle(dutyCycle)
        pwm3.ChangeDutyCycle(dutyCycle)
        time.sleep(1)

def main():
    pablo = ServoGPIO()
    pablo.servosSet(pablo.pwmBottom, pablo.pwmMiddle, pablo.pwmTop, 12.5)
    pablo.servoGPIOCleanup()

if __name__ == "__main__":
    main()
