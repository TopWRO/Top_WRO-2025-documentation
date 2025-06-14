
import os
import time
import sys
import serial
import Adafruit_PCA9685
import pygame
import cv2


pygame.init()
screen = pygame.display.set_mode((1,1),pygame.NOFRAME)

"""
init pwm9685
"""
# pwm = Adafruit_PCA9685.PCA9685()
# Alternatively specify a different address and/or bus:
pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
# Configure min and max servo pulse lengths
# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)
# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)


default_servo_signal = 340 # 345 is 
default_servo_steering_signal = 310 #350
servo_signal = default_servo_signal
servo_steering_signal = default_servo_steering_signal
run_main= True

# keys = pygame.key.get_pressed()
while run_main:

    pwm.set_pwm(0, 0, servo_signal)
    pwm.set_pwm(1, 0, servo_steering_signal)
    time.sleep(0.05)
    
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                run_main = False
            if event.key == pygame.K_i:
                servo_signal = servo_signal + 2
            if event.key == pygame.K_k:
                servo_signal = servo_signal - 2
            if event.key == pygame.K_j:
                servo_steering_signal = servo_steering_signal + 5
            if event.key == pygame.K_l:
                servo_steering_signal = servo_steering_signal - 5
    print("servo_steering_signal",servo_steering_signal,"servo_signal",servo_signal)


print("break!")
pygame.quit()
servo_signal = default_servo_signal
servo_steering_signal = default_servo_steering_signal
for i in range(0,20):

    
    pwm.set_pwm(0, 0, servo_signal)
    pwm.set_pwm(1, 0, servo_steering_signal)
    print("servo_steering_signal",servo_steering_signal,"servo_signal",servo_signal)
    time.sleep(0.05)