import RPi.GPIO as GPIO		#引入RPi.GPIO库函数命名为GPIO
import time					#引入计时time函数

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)     	#将GPIO编程方式设置为BCM模式，基于插座引脚编号

#接口定义
# TRIG = 5					#将超声波模块TRIG口连接到树莓派Pin29
# ECHO = 6                    #将超声波模块ECHO口连接到树莓派Pin31
SONAR = 13
INT1b = 22                   
INT2b = 24                   
INT3b = 26                   
INT4b = 32                   
INT1a = 29                   
INT2a = 23                   
INT3a = 21                   
INT4a = 15                   
EA1 = 3 # 前轮ENA
EB1 = 5 # 前轮EAB
EA2 = 37 # 后轮ENA
EB2 = 38 # 后轮ENB

freq = 100
init_duty = 15 
speed_default = 15 


# 将前侧左右轮、后侧左右轮分别命名为0123轮，四个使能按顺序为EB1\EA1\EA2\EB2
# 注意麦克纳姆轮的安装方向，右轮的正方向为左轮的反方向

#输出模式
# GPIO.setup(TRIG,GPIO.OUT)
# GPIO.setup(ECHO,GPIO.IN)
GPIO.setup(SONAR,GPIO.OUT) # 初始化为输出口
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

pwm0 = GPIO.PWM(EA1, freq)
pwm1 = GPIO.PWM(EB1, freq)
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
            GPIO.output(INT1a,GPIO.LOW)
            GPIO.output(INT2a,GPIO.LOW)
        if motor == 2 :
            GPIO.output(INT1b,GPIO.LOW)
            GPIO.output(INT2b,GPIO.LOW)
        if motor == 3 :
            GPIO.output(INT3b,GPIO.LOW)
            GPIO.output(INT4b,GPIO.LOW)
    if motor == 0 :
        GPIO.output(INT3a,GPIO.LOW if dir else GPIO.HIGH) 
        GPIO.output(INT4a,GPIO.HIGH if dir else GPIO.LOW)
    if motor == 1 :
        GPIO.output(INT2a,GPIO.LOW if dir else GPIO.HIGH) 
        GPIO.output(INT1a,GPIO.HIGH if dir else GPIO.LOW)
    if motor == 2 :
        GPIO.output(INT1b,GPIO.LOW if dir else GPIO.HIGH) 
        GPIO.output(INT2b,GPIO.HIGH if dir else GPIO.LOW)
    if motor == 3 :
        GPIO.output(INT4b,GPIO.LOW if dir else GPIO.HIGH) 
        GPIO.output(INT3b,GPIO.HIGH if dir else GPIO.LOW)
   
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

def Forward(speed=None):
    if speed is None:
        speed = speed_default
    SetMotorSpeed(0, speed)
    SetMotorSpeed(1, speed)
    SetMotorSpeed(2, speed)
    SetMotorSpeed(3, speed)
    SetMotorDir(0, 1)
    SetMotorDir(1, 1)
    SetMotorDir(2, 1)
    SetMotorDir(3, 1)

def Back(time_sleep, speed=None):
    if speed is None:
        speed = speed_default
    SetMotorSpeed(0, speed)
    SetMotorSpeed(1, speed)
    SetMotorSpeed(2, speed)
    SetMotorSpeed(3, speed)
    SetMotorDir(0, 0)
    SetMotorDir(1, 0)
    SetMotorDir(2, 0)
    SetMotorDir(3, 0)
    time.sleep(time_sleep)

def Left(time_sleep, speed=None):
    if speed is None:
        speed = speed_default
    SetMotorSpeed(0, speed)
    SetMotorSpeed(1, speed)
    SetMotorSpeed(2, speed)
    SetMotorSpeed(3, speed)
    SetMotorDir(0, 0)
    SetMotorDir(1, 1)
    SetMotorDir(2, 0)
    SetMotorDir(3, 1)
    time.sleep(time_sleep)

def Stop(time_sleep):
    SetMotorDir(0, 2)
    SetMotorDir(1, 2)
    SetMotorDir(2, 2)
    SetMotorDir(3, 2)
    time.sleep(time_sleep)

#超声波测距函数
def Distance_Ultrasound():
    GPIO.output(SONAR, GPIO.LOW) # 初始化为LOW
    time.sleep(0.000002)
    GPIO.output(SONAR, GPIO.HIGH) # 发射超声波
    time.sleep(0.00001)
    GPIO.output(SONAR, GPIO.LOW) # 停止发射超声波
    GPIO.setup(SONAR, GPIO.IN) # 设置为输入模式以读取信息
    while GPIO.input(SONAR) == 0:
        emitTime = time.time()		#记录发射时间
    while GPIO.input(SONAR) == 1:
        acceptTime = time.time()	#记录接收时间
    GPIO.setup(SONAR, GPIO.OUT) # 设置回输出模式
    totalTime = acceptTime - emitTime		#计算总时间
    distanceReturn = totalTime * 340 / 2 * 100  	#计算距离（单位：cm）
    return distanceReturn			#返回距离

#避障函数
def Obstacle_Avoidance():
    dis = Distance_Ultrasound()
    print("距离", dis, "cm")
    while dis<50:
        Back(1.0)
        Stop(0.2)
        Left(1.0)
        Stop(0.2)
        dis = Distance_Ultrasound() # 重新获取距离
        print("距离", dis, "cm")
        time.sleep(0.01)
    Forward()

def clean_up():
    GPIO.output(SONAR, GPIO.LOW)
    GPIO.output(INT1a, GPIO.LOW)
    GPIO.output(INT2a, GPIO.LOW)
    GPIO.output(INT3a, GPIO.LOW)
    GPIO.output(INT4a, GPIO.LOW)
    GPIO.output(INT1b, GPIO.LOW)
    GPIO.output(INT2b, GPIO.LOW)
    GPIO.output(INT3b, GPIO.LOW)
    GPIO.output(INT4b, GPIO.LOW)
    GPIO.output(EA1, GPIO.LOW)
    GPIO.output(EB1, GPIO.LOW)
    GPIO.output(EA2, GPIO.LOW)
    GPIO.output(EB2, GPIO.LOW)

clean_up()
print("超声波避障系统运行中，按Ctrl+C退出...")
GPIO.output(EA1,GPIO.HIGH)
GPIO.output(EB1,GPIO.HIGH)
GPIO.output(EA2,GPIO.HIGH)
GPIO.output(EB2,GPIO.HIGH)
time.sleep(10)
while True:
    Obstacle_Avoidance()
    time.sleep(0.01)
    # print("Begin Forward")
    # Forward()
    # time.sleep(5)
    # print("Begin Back")
    # Back(5)
    # print("Begin Left")
    # Left(5)
    # GPIO.output(INT3a, GPIO.LOW)
    # GPIO.output(INT4a, GPIO.HIGH)
