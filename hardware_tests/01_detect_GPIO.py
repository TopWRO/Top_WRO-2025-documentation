import RPi.GPIO as GPIO
import time

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)  # Use BCM GPIO numbering


gpio_Num = 25 
GPIO.setup(gpio_Num, GPIO.IN,GPIO.PUD_UP)


while True:
    print(GPIO.input(gpio_Num))
    time.sleep(0.1)


