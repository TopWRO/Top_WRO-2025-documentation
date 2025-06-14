'''
free
'''

#importing necessary files for the code
import os
import time
import sys
current_path = os.getcwd()
sys.path.append(current_path)
from donkeycar.parts.camera import PiCamera
import cv2
from donkeycar.parts.interpreter import KerasInterpreter
from donkeycar.parts.keras import KerasLinear
import Adafruit_PCA9685
import RPi.GPIO as GPIO
import smbus2

'''
init GYRO Sensor
'''
# I2C address of the WT901 sensor (default is 0x50)
DEVICE_ADDRESS = 0x50
# Gyro Z register
GYRO_Z_REGISTER = 0x39  # Example register for Z-axis gyroscope

# Initialize the I2C bus
bus = smbus2.SMBus(1)

# Variable to store the yaw angle
yaw_angle = 0

# Time tracking for integrationframe
previous_time = time.time()


# Preset variables
home_path = '/home/pi/airc_drive/models20occw/models22occw/'
sys.path.append(home_path)
#model = "turkeyfreerumodelncwmodel11/mypilot.h5"
model = "mypilot.h5"
stop_deg = 1130
default_servo_signal = 370
default_servo_signal_offset = 23
servo_signal_multiplier = 140
default_servo_steering_signal = 305
default_servo_steering_signal_offset = 100 #150 originally
servo_steering_signal_multiplier = -300


'''
init GPIO
'''
# Set the GPIO mode
GPIO.setmode(GPIO.BCM)  # Use BCM GPIO numbering
gpio_Num = 25 
GPIO.setup(gpio_Num, GPIO.IN,GPIO.PUD_UP)

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


def read_gyro_z():
    # Read two bytes from the gyro Z register
    data = bus.read_i2c_block_data(DEVICE_ADDRESS, GYRO_Z_REGISTER, 2)
    # Combine the two bytes into a 16-bit value
    gyro_z_raw = data[1] << 8 | data[0]
    
    # Convert the raw gyroscope value to angular velocity in degrees per second
    # (You might need to adjust the scale depending on your gyro's sensitivity)
    if gyro_z_raw > 32767:
        gyro_z_raw -= 65536
    gyro_z = gyro_z_raw / 32768.0 * 2000  # Assuming +/- 2000 dps range, adjust if needed
    return gyro_z
# ======================================================


servo_steering_signal= 0
servo_signal= 0

"""
init camera
from dondeycar/parts/camera
"""
IMAGE_W = 160
IMAGE_H = 120
IMAGE_DEPTH = 3
cam = PiCamera(image_w=IMAGE_W, image_h=IMAGE_H, image_d=IMAGE_DEPTH,
                 vflip=False, hflip=False)


# """
# init tf and models
# """
model_path = home_path + model
input_shape = (120, 160, 3)
model_type = 'linear'
interpreter = KerasInterpreter()
kl = KerasLinear(interpreter=interpreter, input_shape=input_shape)
kl.load(model_path)




'''
run Main CODE
'''

#Moves the car steering back and forth to indicate that the model is about to run
for n in range(0,1):
    for i in range(0,5):
        pwm.set_pwm(1, 0, default_servo_steering_signal)
        pwm.set_pwm(0, 0, default_servo_signal)
        time.sleep(0.1)   

    for i in range(0,5):
        pwm.set_pwm(1, 0, default_servo_steering_signal+100)
        pwm.set_pwm(0, 0, default_servo_signal)
        time.sleep(0.1)
        
    for i in range(0,5):
        pwm.set_pwm(1, 0, default_servo_steering_signal-100)
        pwm.set_pwm(0, 0, default_servo_signal)
        time.sleep(0.1)
    
    for i in range(0,5):
        pwm.set_pwm(1, 0, default_servo_steering_signal)
        pwm.set_pwm(0, 0, default_servo_signal)
        time.sleep(0.1) 
          
     
# # forces the user to press the off switch when starting
while (not GPIO.input(gpio_Num)):
    pwm.set_pwm(1, 0, default_servo_steering_signal)
    pwm.set_pwm(0, 0, default_servo_signal)
    print("turn off switch")
    time.sleep(0.1)

# waits for the user to press the on switch before starting the vehicle
while (GPIO.input(gpio_Num)):
    pwm.set_pwm(1, 0, default_servo_steering_signal)
    pwm.set_pwm(0, 0, default_servo_signal)
    print("waiting for switch to be turned on")
    time.sleep(0.1)


# variable boolean that indicates whether to continue running the program or not
main_run_flag = True
# Time tracking for integration
previous_time = time.time()
# setup car status
time_last = time.time()

# Stop conditions: either the gyro sensor reads that 3 laps have been completed or the model is turned off manually by the user
while 1:
    
    gyro_z = read_gyro_z()
    if gyro_z > 0:
        gyro_z = gyro_z * 1.008

    # Get the current time
    current_time = time.time()

    # Calculate time difference (delta t)
    delta_t = current_time - previous_time

    # Update the yaw angle using gyro data (angular velocity * time)
    yaw_angle += gyro_z * delta_t

    # Update previous time for the next iteration
    previous_time = current_time
    print('yaw_angle=',round(yaw_angle,2))
    
    if (GPIO.input(gpio_Num) or abs(yaw_angle) > stop_deg):
        main_run_flag = False
    
    
    frame = PiCamera.run(cam)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
#     frame = cv2.resize(frame,(160,120))
    
#     height, width, square = frame.shape
#     print(height, width, square)
#     cv2.imshow("cam", frame )
#     cv2.waitKey(1)

    outputs = KerasLinear.run(kl,img_arr = frame)
    print("steering=",round(outputs[0],2),"throttle=",round(outputs[1],2))
    print(int((round(outputs[0],2))*(-200)+ default_servo_steering_signal-70))


#Prepare the PMW data
            

    servo_signal = int((round(outputs[1],2))*(0+servo_signal_multiplier)+ default_servo_signal)
    if servo_signal > default_servo_signal + default_servo_signal_offset:
        servo_signal = int(default_servo_signal + default_servo_signal_offset)
    if servo_signal < default_servo_signal - default_servo_signal_offset:
        servo_signal = int(default_servo_signal - default_servo_signal_offset)
        

    servo_steering_signal = int(((round(outputs[0],2))*(servo_steering_signal_multiplier)+ default_servo_steering_signal)*1)
    if servo_steering_signal > default_servo_steering_signal + default_servo_steering_signal_offset:
        servo_steering_signal = int(default_servo_steering_signal + default_servo_steering_signal_offset)
    if servo_steering_signal < default_servo_steering_signal - default_servo_steering_signal_offset:
        servo_steering_signal = int(default_servo_steering_signal - default_servo_steering_signal_offset)


      

#output to 9685    

    pwm.set_pwm(1, 0, servo_steering_signal)
    pwm.set_pwm(0, 0, servo_signal)

    print("streeing PWM = ",servo_steering_signal, "throttle PWM = ",servo_signal)



    time.sleep(0.01)


    #cv2.imshow('Camera Feed', frame)
     # Break the loop on 'q' key press

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# after th emodel has finished running we input stop signals for the car in order to get it to stop
for i in range(0,10):
    pwm.set_pwm(1, 0, default_servo_steering_signal)
    pwm.set_pwm(0, 0, default_servo_signal)
    time.sleep(0.1)
    print('pwm out stop')


# finally we close the camera and shut donw everything for the process to end
PiCamera.shutdown(cam)
cv2.destroyAllWindows()