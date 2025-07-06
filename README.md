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


### 4. After all data has been collected  
I use FileZilla to transfer the data folder to our own computer for model training.

### 5.Then, on Ubuntu, we run the following commands:

```bash
# a. Activate the donkey environment
conda activate donkey

# b. Enter the project folder
cd mycar

# c. Train the model using your dataset
donkey train --tub <your_data_path> --model ./models/mypilot.h5
```
You will get a folder named models, and the .h5 file inside it is the result of the training.  
If you want to continue training based on an existing model, use the following command:

```bash
donkey train --tub ./<your_data_path> --model ./models/mypilot.h5 --transfer ./<original_model_path>/mypilot.h5
```
During training, the terminal output will look like this:  

---



## 🟢 Free Run: Implementation and Challenges We Overcame  
*这部分介绍我们freerun的训练 主要三点：1.多次变换场地 达到让车子不依赖单一环境的效果 2.故意输入极端情况 给车有正值的输入让他学会应对 3.用陀螺仪实现停止程序。*


---

## Obstacle Run:一个副标题 还没想好  
*主要内容就是说我们如何训练障碍跑 比如也是多次随机换柱子 摆好停车区 故意在柱子前放慢转弯等*

---

## 🅿️ Training the Stop Model: Difficulties We Faced and How We Solved Them  
*这部分写停车 可以先空着 因为还没做*

---

## 🔚 Conclusion & Future Work  
*总结和展望 总结段*


---

## 📚 Citation (DonkeyCar parts)

> DonkeyCar Contributors. *Donkey Car: A Self Driving Platform for DIY Robotic Cars*. DonkeyCar, 2024, https://docs.donkeycar.com/. Accessed 6 July 2025.
