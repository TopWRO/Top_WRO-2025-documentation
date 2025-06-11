# Top_WRO-2025-documentation
# 🚗 WRO 2025 Future Engineers - AI Autonomous Driving Project

This project is developed for the WRO 2025 Future Engineers competition. Based on Tensor Flow(machining learning). We use camera to gather data first,then give all the data to CNN neural network to generate/train a model,which will learn from the data,find the rule,and making its own prediction while running the car. We encoded the car’s steering angle and throttle as continuous numerical outputs, enabling the neural network to learn the correlation between visual inputs and control signals. Then the car will generate the steering/throttle value based on the current image and the rule it found. The model outputs continuous control values for steering and throttle. These values are then interpreted by the control loop and converted into PWM signals via the PCA9685 driver, which in turn control the servo and motor to perform the actual driving actions.  
这里放个总体图片  

##  Hardware Components
1.Raspberry Pi 4  
Acts as the central computing unit, running the software stack, processing camera input, and executing the trained AI model in real time.  
2. PCA9685  
Converts numerical steering and throttle outputs into PWM signals, allowing precise control of the servo (for steering) and ESC (for throttle).  
3. PiCamera  
Captures real-time image frames from the car’s perspective, which serve as input to the neural network for decision-making.  
4. PS4 Controller  
Used for manual control during data collection; allows the driver to steer and throttle the car while the system records input-output pairs.  
5.Gyro  
Gather the steering angles to determine whether to stop the car or not.For example,if you want your car runs 3 laps and then stop,you can wirte a pregram to sto[ it when the sum of all steering angles equals 1080（360*3）.

## Software & Code Structure  

