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
我已经上传了一部分的代码 这一部分是介绍我们用到的软体和代码的作用

## The Overall Training Pipeline
这一部分是介绍我们训练的大体流程：路由器/车子开机、链接手柄、测试车子性能、终端命令开始收集数据、filezella传数据到自己电脑、训练、返回模型。

## 🟢 Free Run: Implementation and Challenges We Overcame
这部分介绍我们freerun的训练 主要三点：1.多次变换场地 达到让车子不依赖单一环境的效果 2.故意输入极端情况 给车有正值的输入让他学会应对 3.用陀螺仪实现停止程序。

## Obstacle Run:一个副标题 还没想好
主要内容就是说我们如何训练障碍跑 比如也是多次随机换柱子 摆好停车区 故意在柱子前放慢转弯等

## 🅿️ Training the Stop Model: Difficulties We Faced and How We Solved Them
这部分写停车 可以先空着 因为还没做

## 🔚 Conclusion & Future Work
总结和展望 总结段落


