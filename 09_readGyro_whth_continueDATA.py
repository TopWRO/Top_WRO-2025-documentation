import smbus2
import time

# I2C address of the WT901 sensor (default is 0x50)
DEVICE_ADDRESS = 0x50
# Register for yaw data
YAW_REGISTER = 0x3F

GYRO_Z_REGISTER = 0x39  # Example register for Z-axis gyroscope
# Initialize the I2C bus
bus = smbus2.SMBus(1)

# Variable to store the previous yaw value
previous_yaw = None
# Variable to store the cumulative yaw value
cumulative_yaw = 0
last_delta_yaw = 0

def read_yaw():
    # Read two bytes from the yaw register
    data = bus.read_i2c_block_data(DEVICE_ADDRESS, YAW_REGISTER, 2)
    # Combine the two bytes into a 16-bit value
    yaw_raw = data[1] << 8 | data[0]
    
    # Convert the raw yaw value to degrees
    if yaw_raw > 32767:
        yaw_raw -= 65536
    yaw = yaw_raw / 32768.0 * 180  # Convert to degrees
    return yaw

def read_cumulative_yaw():
    global previous_yaw
    global cumulative_yaw
    global last_delta_yaw
    
    current_yaw = read_yaw()
    if previous_yaw is not None:
        # Calculate the change in yaw
        delta_yaw = current_yaw - previous_yaw
        # Handle the wrapping from +180 to -180 and vice versa
        if delta_yaw > 180:
            delta_yaw -= 360
        elif delta_yaw < -180:
            delta_yaw += 360

        # Add the change to the cumulative yaw
        cumulative_yaw = cumulative_yaw + delta_yaw*1# + last_delta_yaw*0.9
        last_delta_yaw = delta_yaw
    # Update the previous yaw value for the next iteration
    previous_yaw = current_yaw

    return round(0-cumulative_yaw,0)




try:
    while True:
        print(read_cumulative_yaw())
        # Wait for a short period before the next reading
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by User")