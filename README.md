# Top_WRO-2025-documentation
# 🚗 WRO 2025 Future Engineers - AI Autonomous Driving Project

This project is developed for the **WRO 2025 Future Engineers competition**. It is based on the open-source [DonkeyCar](https://docs.donkeycar.com/) platform and uses **TensorFlow** for machine learning. Inspired by DonkeyCar's modular structure, we designed a system where a camera captures image data, which is then passed to a **CNN neural network** for training. The model learns from this data to detect driving patterns and make predictions in real time during autonomous operation.

We encode the car’s **steering angle** and **throttle** as continuous numerical outputs, allowing the neural network to learn the correlation between visual input and control signals. During operation, the car generates real-time steering and throttle values based on the current image and the learned patterns. These outputs are then converted into **PWM signals** by the **PCA9685 driver**, which controls the servo and motor accordingly to execute driving behavior.

📷 *(Insert an overview image here)*

---

## 🧱 Hardware Components

1. **Raspberry Pi 4**  
Acts as the central computing unit, running the software stack, processing camera input, and executing the trained AI model in real time.  
📷 *(Insert photo here)*

2. **PCA9685**  
Converts numerical steering and throttle outputs into PWM signals, allowing precise control of the servo (for steering) and ESC (for throttle).  
📷 *(Insert photo here)*

3. **PiCamera**  
Captures real-time image frames from the car’s perspective, which serve as input to the neural network for decision-making.  
📷 *(Insert photo here)*

4. **PS4 Controller**  
Used for manual control during data collection; allows the driver to steer and throttle the car while the system records input-output pairs.  
📷 *(Insert photo here)*

5. **Gyroscope**  
Tracks cumulative steering angles to determine when the car should stop.  
For example, if you want the car to stop after completing 3 laps, the program halts once the total steering angle reaches 1080° (360° × 3).  
📷 *(Insert photo here)*

---

## 💻 Software & Code Structure

### 🧠 Software:

- **Raspberry Pi OS**  
  The Linux-based operating system running on the Raspberry Pi, providing the environment for executing scripts and AI models.

- **TensorFlow**  
  A deep learning framework used to train and load the neural network model for autonomous driving.

- **Neural network model (.h5 file)**  
  A trained model file that stores driving behavior patterns, used for inference during car operation.

---

### 🐍 Python Code:

- [`manage.py`](manage.py)  
  The main entry point of the project; used to initiate training or start autonomous driving.

- [`train.py`](train.py)  
  Trains the CNN model using labeled image-throttle-steering datasets.

- [`main_freerun.py`](main_freerun.py)  
  Runs the trained model in real-time on the vehicle for free-run mode.

- [`_Control_RCcar_with_KB.py`](_Control_RCcar_with_KB.py)  
  Manual keyboard control script to test if the motors and servo respond correctly to PWM signals.

- [`01_detect_GPIO.py`](01_detect_GPIO.py)  
  Verify GPIO functionality before running the car.

- [`09_readGyro_whth_continueDATA.py`](09_readGyro_whth_continueDATA.py)  
  Test whether the gyroscope is functioning as expected.

- [`Adafruit_LCD1602.py`](Adafruit_LCD1602.py)  
  Run a test to ensure the LCD is functioning properly.

- [`US2025_FreeRun_V01.py`](US2025_FreeRun_V01.py)  
  This is a program for free run to run the model and make it stop after 3 laps.

- [`US2025_Obstaclerun_V01.py`](US2025_Obstaclerun_V01.py)  
  This is the version 1 program for the obstacle run. The car can start at the parking area while there hasn't some code for it to park correctly.

---

## The Overall Training Pipeline  
## 🛠️ Data Collection and Model Training

### 1. Hardware Preparation  
Before training begins, we insert the battery into the car, connect the router, power on the RC car, and pair the controller.

### 2. System Check  
Before collecting data, it is crucial to check that each component of the car is functioning correctly.  
We use the test scripts provided in the previous section to verify the following components in sequence:  
- GPIO  
- Gyroscope  
- LCD screen  
- Camera  
- Finally, we manually control the car using the keyboard to check if it starts correctly.

### 3. Official Data Collection  
In the terminal, we start the driving process by entering the following commands:

```bash
cd mycar
python manage.py drive --js
```
This will allow the car to be controlled by the joystick and start recording data. (See the image below)  
![Terminal1](./Screenshot%202025-07-06%20143555.png)
![Collecting Data](./Screenshot%202025-07-06%20143620.png)



### 4. After all data has been collected  
I use FileZilla to transfer the data folder to our own computer for model training.  
![FileZilla Transfer](./Screenshot%202025-07-06%20143846.png)

### 5.Then, on Ubuntu, we run the following commands:

```bash
# a. Activate the donkey environment
conda activate donkey

# b. Enter the project folder
cd mycar

# c. Train the model using your dataset
donkey train --tub <your_data_path> --model ./models/mypilot.h5
```
![Start Training](./Screenshot%202025-07-06%20151029.png)

You will get a folder named models, and the .h5 file inside it is the result of the training.  
If you want to continue training based on an existing model, use the following command:

```bash
donkey train --tub ./<your_data_path> --model ./models/mypilot.h5 --transfer ./<original_model_path>/mypilot.h5
```
During training, the terminal output will look like this:  
![During Training](./Screenshot%202025-07-06%20151038.png)

---



## 🟢 Free Run: Implementation and Challenges We Overcame  
*这部分介绍我们freerun的训练 主要三点：1.多次变换场地 达到让车子不依赖单一环境的效果 2.故意输入极端情况 给车有正值的输入让他学会应对 3.用陀螺仪实现停止程序。*


---

## 🚧 Obstacle Run: Implementation and Training Strategy
In the WRO 2025 Future Engineers competition, the Obstacle Run part of our project was about making the AI car drive safely through a track with randomly placed red and green pillars. These pillars would change position every time, so the car had to learn how to react to new situations by itself, without following a fixed path. This made the training harder but also more meaningful.

### Our Goal and Difficulties
Our main goal was to teach the car to avoid the pillars, slow down when needed, and then go back to the center of the road after passing the obstacle. Since the positions of the pillars changed each time, the AI had to be trained to handle different cases, instead of just memorizing one.

### How We Trained the AI
We used a tool called Donkey Car, which helps train self-driving cars using AI. The training method we used is called behavior cloning, which means the AI learns by watching how humans drive. We collected many driving examples by manually driving the car using a PS4 controller. While we drove, the system recorded the images from the camera and the matching steering and throttle values.

To help the AI learn better, we divided the training into two parts:

#### Driving clockwise around the field.
#### Driving counter-clockwise.
For each part, we trained with about 200,000 pieces of data, and we did around 40 to 50 training sessions. In each session, we collected about 4,000 driving examples.

Before each session, we changed the positions of the red and green pillars. This way, the AI saw many different layouts and didn’t get used to just one. We also created some hard situations, like putting the pillars near the center or making quick turns, so the AI could learn to react better.

### What the AI Learned to Do
During the training, we wanted the car to learn a few important actions:

1)Slow down when getting close to a pillar.

2)Turn the steering wheel just enough to go around the obstacle.

3)Go back to the middle of the road after passing it.
We used a PiCamera to capture the car’s view, a Raspberry Pi 4 to run the AI model, and a PCA9685 driver to control the motor and steering. After training, the model was saved as a .h5 file and used for real-time driving.

### Results
After many training rounds, the car became much better at handling obstacles, even when we placed the pillars in totally new positions. It could predict the best path, slow down, and drive safely through the track. Training with both clockwise and counter-clockwise data also made the car smarter and more balanced.
This project helped us understand how AI can learn from human actions and improve over time. The Obstacle Run part was a big challenge, but also a great experience in using real data to train a smart and flexible self-driving car.





*

---

## 🅿️ Solving stop and parking problem: Difficulties We Faced and How We Solved Them  
### Part 1: How We Make the Car Stop   
In the Free Run section, our goal was to automatically stop the car after it completed three laps. To achieve this, we used the cumulative yaw angle detected by the gyroscope (final_gyro_degree), and set a threshold of approximately 1080 degrees as the stopping condition.   
```python
stop_degree_1 = 1080
stop_degree_end = 1150
stop_delate_time = 3
```   
   
In the main loop, the cumulative yaw angle is updated in real-time:

```python
final_gyro_degree = read_cumulative_yaw()
```

Once the yaw exceeds the threshold, we command the vehicle to stop:
```python
if abs(final_gyro_degree) > stop_degree_end or GPIO.input(gpio_Num) == 1:
    Run_main = False
    print('stop at time_end')
```
#### 🧩 Problem 1: The Car Didn't Stop Near the Starting Point   
In practice, we found that stopping the car immediately at 1080° caused it to stop midway through the final curve, rather than near the original starting point. This happens because the vehicle may still be in motion due to the last model prediction.

To fix this, we introduced a 3-second delay after reaching the angle threshold, allowing the car to continue moving before fully stopping. This is implemented as follows:
```python
if abs(final_gyro_degree) < stop_degree_1:
    stop_timer = time.time()
if time.time() - stop_timer > stop_delate_time:
    Run_main = False
    print('stop at delate time')
```

#### 📉 Problem 2: Inaccuracy in the Gyroscope   
We also observed that the gyroscope was not always accurate. Even after completing three laps, the cumulative yaw might read too high or too low. This could be due to sensor drift, calibration issues, or environmental noise.

To handle this issue, we implemented two solutions:    

##### 1. Apply a correction factor (gyro_offset)   
This scale factor slightly adjusts the yaw value during integration:
```python
return round(0 - cumulative_yaw * gyro_offset, 0)
```

##### 2.Tune the stopping threshold
We tested multiple threshold values (from 1080 to 1150), and finally set:   
```python
stop_degree_end = 1150
```
This ensured the car stopped only after reliably returning near the starting point.   


### Part 2:How the Car Exits the Parking Lot   
To start running, we first needed the car to exit the parking space autonomously. Because the parking space is narrow and turning radius is limited, we adopted a “twist-out” strategy: drive forward with the wheels turned in one direction, then reverse with the wheels turned the other way, and repeat this pattern until the car is aligned and ready to go.   

#### 💡 Twist-Out Logic
This logic is implemented in the start_from_parkinglot() function. Here's a simplified explanation of the steps:
##### 1.Forward with left/right turn:
```python
servo_signal = default_servo_signal + 23
servo_steering_signal = default_servo_steering_signal + 150
pwm.set_pwm(0, 0, servo_signal)
pwm.set_pwm(1, 0, servo_steering_signal)
```
##### 2.Backward with opposite turn:
```python
servo_signal = default_servo_signal - 27
servo_steering_signal = default_servo_steering_signal - 150
pwm.set_pwm(0, 0, servo_signal)
pwm.set_pwm(1, 0, servo_steering_signal)
```

##### 3.Repeat this sequence, while checking the angle change using the gyroscope:   
```python
while abs(final_gyro_degree) < 28:
    final_gyro_degree = read_cumulative_yaw()
    time.sleep(0.1)
```

#### ⚠️ RC Car Safety Mechanism
Because of the RC car's built-in safety, it doesn't allow immediate switching from forward to reverse. We had to issue the reverse command twice with a short delay to actually engage the reverse motion.   

#### 🤖 Transition to Obstacle Avoidance
Once the start_from_parkinglot() routine completes, the car is properly oriented and placed on track. At that point, we immediately switch to the trained AI model (obstaclerun) for autonomous driving.
In the main function, we call:
```python
start_from_parkinglot()
print('start !!!!')
```
Then the loop begins where we:

Capture the camera frame

Use the trained obstaclerun model to predict

Convert model outputs to PWM signals

Drive the car accordingly   
```python
frame = PiCamera.run(cam)
outputs = KerasLinear.run(kl, img_arr=frame)
servo_steering_signal = round(outputs[0], 2)
servo_signal = round(outputs[1], 2)
```

#### Summary
We use a twist-out maneuver to exit the parking lot.

RC car requires double reverse commands due to safety logic.

After parking exit, the car immediately enters AI-based obstacle avoidance mode using our trained model.


---

## 🔚 Conclusion & Future Work  
*总结和展望 总结段*


---

## 📚 Citation (DonkeyCar parts)

> DonkeyCar Contributors. *Donkey Car: A Self Driving Platform for DIY Robotic Cars*. DonkeyCar, 2024, https://docs.donkeycar.com/. Accessed 6 July 2025.
