import RPi.GPIO as GPIO		#引入RPi.GPIO库函数命名为GPIO
import time					#引入计时time函数

#GPIO.setwarnings(False)

GPIO.setmode(GPIO.BOARD)     	#将GPIO编程方式设置为BCM模式，基于插座引脚编号

#接口定义
# TRIG = 5					#将超声波模块TRIG口连接到树莓派Pin29
# ECHO = 6                    #将超声波模块ECHO口连接到树莓派Pin31
SONAR = 13
INT1a = 22                   
INT2a = 24                   
INT3a = 26                   
INT4a = 32                   
INT1b = 29                   
INT2b = 23                   
INT3b = 21                   
INT4b = 19                   
EA1 = 3 # 前轮ENA
EB1 = 5 # 前轮EAB
EA2 = 37 # 后轮ENA
EB2 = 38 # 后轮ENB

freq = 50
init_duty = 50


# 将前侧左右轮、后侧左右轮分别命名为0123轮，四个使能按顺序为EB1\EA1\EA2\EB2
# 注意麦克纳姆轮的安装方向，右轮的正方向为左轮的反方向

#输出模式
# GPIO.setup(TRIG,GPIO.OUT)
# GPIO.setup(ECHO,GPIO.IN)
GPIO.setup(INT1a,GPIO.OUT)
GPIO.setup(INT2a,GPIO.OUT)
GPIO.setup(INT3a,GPIO.OUT)
GPIO.setup(INT4a,GPIO.OUT)
GPIO.setup(INT1b,GPIO.OUT)
GPIO.setup(INT2b,GPIO.OUT)
GPIO.setup(INT3b,GPIO.OUT)
GPIO.setup(INT4b,GPIO.OUT)
GPIO.setup(EA1,GPIO.OUT)
GPIO.setup(EB1,GPIO.OUT)
GPIO.setup(EA2,GPIO.OUT)
GPIO.setup(EB2,GPIO.OUT)

pwm0 = GPIO.PWM(EB1, freq)
pwm1 = GPIO.PWM(EA1, freq)
pwm2 = GPIO.PWM(EA2, freq)
pwm3 = GPIO.PWM(EB2, freq)
pwm0.start(init_duty)
pwm1.start(init_duty)
pwm2.start(init_duty)
pwm3.start(init_duty)

#一直前进函数
def SetMotorDir(motor, dir): # 输入电机编号和转动方向 (1 正 0 负 2 停机)
    if dir == 2:
        if motor == 0 :
            GPIO.output(INT3a,GPIO.LOW)
            GPIO.output(INT4a,GPIO.LOW)
        if motor == 1 :
            GPIO.output(INT2a,GPIO.LOW)
            GPIO.output(INT1a,GPIO.LOW)
        if motor == 2 :
            GPIO.output(INT3b,GPIO.LOW)
            GPIO.output(INT4b,GPIO.LOW)
        if motor == 3 :
            GPIO.output(INT2b,GPIO.LOW)
            GPIO.output(INT1b,GPIO.LOW)
    if motor == 0 :
        GPIO.output(INT3a,GPIO.LOW if dir else GPIO.HIGH) 
        GPIO.output(INT4a,GPIO.HIGH if dir else GPIO.LOW)
    if motor == 1 :
        GPIO.output(INT2a,GPIO.LOW if dir else GPIO.HIGH) 
        GPIO.output(INT1a,GPIO.HIGH if dir else GPIO.LOW)
    if motor == 2 :
        GPIO.output(INT4b,GPIO.LOW if dir else GPIO.HIGH) 
        GPIO.output(INT3b,GPIO.HIGH if dir else GPIO.LOW)
    if motor == 3 :
        GPIO.output(INT1b,GPIO.LOW if dir else GPIO.HIGH) 
        GPIO.output(INT2b,GPIO.HIGH if dir else GPIO.LOW)
   
def SetMotorSpeed(motor, speed): # speed 从 0 到 50
    if motor == 0 : 
        pwm0.ChangeDutyCycle(speed)
    if motor == 1 : 
        pwm1.ChangeDutyCycle(speed)
    if motor == 2 : 
        pwm2.ChangeDutyCycle(speed)
    if motor == 3 : 
        pwm3.ChangeDutyCycle(speed)
    pass

def Forward():
    SetMotorDir(0, 1)
    SetMotorDir(1, 1)
    SetMotorDir(2, 1)
    SetMotorDir(3, 1)
    SetMotorSpeed(0,  5)
    SetMotorSpeed(1, 20)

#后退指定时间函数
def Back_time(time_sleep):
    GPIO.output(INT1a,GPIO.HIGH)
    GPIO.output(INT2a,GPIO.LOW)
    GPIO.output(INT3a,GPIO.LOW)
    GPIO.output(INT4a,GPIO.HIGH)
    GPIO.output(INT1b,GPIO.HIGH)
    GPIO.output(INT2b,GPIO.LOW)
    GPIO.output(INT3b,GPIO.LOW)
    GPIO.output(INT4b,GPIO.HIGH)
    time.sleep(time_sleep)

#左转指定时间函数
def Left_time(time_sleep):
    GPIO.output(INT1a,GPIO.LOW)
    GPIO.output(INT2a,GPIO.LOW)
    GPIO.output(INT3a,GPIO.LOW)
    GPIO.output(INT4a,GPIO.HIGH)
    GPIO.output(INT1b,GPIO.LOW)
    GPIO.output(INT2b,GPIO.LOW)
    GPIO.output(INT3b,GPIO.LOW)
    GPIO.output(INT4b,GPIO.HIGH)
    time.sleep(time_sleep)

#停止函数
def Stop():
    GPIO.output(INT1a,GPIO.LOW)
    GPIO.output(INT2a,GPIO.LOW)
    GPIO.output(INT3a,GPIO.LOW)
    GPIO.output(INT4a,GPIO.LOW)
    GPIO.output(INT1b,GPIO.LOW)
    GPIO.output(INT2b,GPIO.LOW)
    GPIO.output(INT3b,GPIO.LOW)
    GPIO.output(INT4b,GPIO.LOW)

#超声波测距函数
def Distance_Ultrasound():

    # GPIO.output(TRIG,GPIO.LOW)		#输出口初始化置LOW（不发射）
    GPIO.setup(SONAR,GPIO.OUT)
    time.sleep(0.000002)
    # GPIO.output(TRIG,GPIO.HIGH)		#发射超声波
    GPIO.output(SONAR, GPIO.HIGH)
    time.sleep(0.00001)
    # GPIO.output(TRIG,GPIO.LOW)		#停止发射超声波
    GPIO.output(SONAR, GPIO.LOW)
    GPIO.setup(SONAR,GPIO.IN)
    while GPIO.input(SONAR) == 0:
        emitTime = time.time()		#记录发射时间
    while GPIO.input(SONAR) == 1:
        acceptTime = time.time()	#记录接收时间
    totalTime = acceptTime - emitTime		#计算总时间
    distanceReturn = totalTime * 340 / 2 * 100  	#计算距离（单位：cm）
    return  distanceReturn			#返回距离

#避障函数
def Obstacle_Avoidance():
    dis = Distance_Ultrasound()
    print("距离", dis, "cm")
    while dis<30:
        Back_time(0.5)
        Left_time(1.5)
        Forward()
        dis = Distance_Ultrasound() # 重新获取距离
        print("距离", dis, "cm")


	# dis = Distance_Ultrasound()
	# print("距离 ",dis,"cm")
    #     while dis<30:
    #         Back_time(0.5)		#距离小于30cm时后退0.5s
    #         dis=Distance_Ultrasound()
    #         print("距离 ",dis,"cm")
    #     Left_time(1.5)			#左转1.5s
    #     forward()				#继续前进
    # time.sleep(0.5)


print("超声波避障系统运行中，按Ctrl+C退出...")
# try:
    # Forward()				#初始状态为前进
    # Obstacle_Avoidance()
# except KeyboardInterrupt:
    # Stop()
GPIO.output(EA1,GPIO.HIGH)
GPIO.output(EB1,GPIO.HIGH)
GPIO.output(EA2,GPIO.HIGH)
GPIO.output(EB2,GPIO.HIGH)
Forward()
while True:
    pass
    # Left_time(0)
    Obstacle_Avoidance()
