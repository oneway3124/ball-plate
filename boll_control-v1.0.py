# Untitled - By: Administrator - 周一 十一月 29 2021
import time,image,math
import sensor#引入感光元件的模块
from pyb import Pin
import json
from pyb import UART

from pyb import Servo

#舵机IO设置
Servo_x = Servo(1) # P7
Servo_y = Servo(2) # P8

Servo_x.angle(0)
Servo_y.angle(0)

# x y 轴舵机控制函数封装
def Set_Servo_x_angle(temp):
   if (temp > -50 and temp <46) :
      Servo_x.angle(temp)
   elif temp < -50:
      Servo_x.angle(-50)
   else:
      Servo_x.angle(50)

def Set_Servo_y_angle(temp):
   if (temp > -46 and temp <46) :
      Servo_y.angle(temp)
   elif temp < -46:
      Servo_y.angle(-46)
   else:
      Servo_y.angle(46)



threshold_index = 0 # 0 for red, 1 for green, 2 for blue

red = (10,10,10,10,10,20)
blue =  ( 0, 80 -70, -10, -0,30)
yellow = (10,10,10,10,10,20)
green_threshold   = (  15,   40,  -40,   -10,   10,   40)

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds = [(30, 40, -40, -20, 30, 50)] # generic_red_thresholds
# 设置摄像头
sensor.reset()#初始化感光元件
sensor.set_pixformat(sensor.RGB565)#设置为彩色 RGB565: 彩色
sensor.set_framesize(sensor.SVGA)#设置图像的大小 sensor.QVGA: 320x240，sensor.VGA: 640x480 HVGA:480×320
sensor.set_windowing(70,0,600,600)
#sensor.set_auto_gain(True) # 自动增益开启（True）
#sensor.set_auto_whitebal(True) #自动白平衡开启
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
# 开灯
from pyb import LED
led = LED(1) # 红led
led.on()#亮
led.off()


sensor.set_pixformat(sensor.RGB565)#设置为彩色 RGB565: 彩色



time.sleep_ms(100)
#pin9.value(0)
#time.sleep_ms(500)
sensor.skip_frames()#跳过n张照片，在更改设置后，跳过一些帧，等待感光元件变稳定。nr

clock = time.clock()
flag_dw = True


class PID_Inc:
    """增量式PID"""
    def __init__(self, KP, KI, KD):
        self.Kp = KP
        self.Ki = KI
        self.Kd = KD

        self.PreError = 0
        self.PreDerror = 0
        self.error = 0
        self.d_drror = 0
        self.dd_error = 0
        self.dcalc = 0

        self.sumerror =0
    def setKp(self,KP):
        self.Kp=KP

    def setKi(self,KI):
        self.Ki=KI

    def setKd(self,KD):
        self.Kd=KD

    def update(self, Target, Current):
        self.error = Target - Current
        self.d_error = self.error - self.PreError
        self.dd_error = self.d_error - self.PreDerror
        self.PreError = self.error
        self.PreDerror = self.d_error

        self.sumerror +=self.error
        '''
        self.dcalc = (
            self.Kp * self.d_error + self.Ki * self.error + self.Kd * self.dd_error
        )
        '''
        self.dcalc = (
            self.Kp * self.error + self.Ki * self.sumerror + self.Kd * self.d_error
        )

        return self.dcalc

    #pid 设置
Pid_x = PID_Inc(0.02, 0.0005, 0.18)
Pid_y = PID_Inc(0.02, 0.0005, 0.18)
cnt =0
while(flag_dw):
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)#lens_corr(1.8)畸变矫正
    #img = sensor.snapshot()
    # Circle objects have four values: x, y, r (radius), and magnitude. The
    # magnitude is the strength of the detection of the circle. Higher is
    # better...

    # `threshold` controls how many circles are found. Increase its value
    # to decrease the number of circles detected...

    # `x_margin`, `y_margin`, and `r_margin` control the merging of similar
    # circles in the x, y, and r (radius) directions.

    # r_min, r_max, and r_step control what radiuses of circles are tested.
    # Shrinking the number of tested circle radiuses yields a big performance boost.

    #img.draw_circle((300+correct_x, 300+correct_y, 90), color=(255,0,0))
    #img.draw_cross(300,284,size=30, color=(255,255,255))

    img.draw_cross(300,300,size=20, color=(255,255,255))
    img.draw_circle(300,300,40, color=(255,0,0))

    #find circle
    '''
    for c in img.find_circles(threshold = 3500, x_margin = 10, y_margin = 10, r_margin = 10,
        r_min = 10, r_max = 200, r_step = 5):
        img.draw_circle(c.x(), c.y(), c.r(), color = (255, 255, 0))
      #  print(c)
    '''
    # img.find_blobs([green_threshold, yellow])
    # for blob in blobs:
    #   print(blob.cx())

    blobs = img.find_blobs([green_threshold],x_stride=20,y_stride=20,area_threshold =300)

    if blobs: #如果找到了目标颜色
            cnt =0
            for blob in blobs:
            #迭代找到的目标颜色区域
                # Draw a rect around the blob.
              #  img.draw_rectangle(blob[0:4]) # rect
                #用矩形标记出目标颜色区域
                img.draw_cross(blob.cx(), blob.cy() ) # cx, cy

                #在目标颜色区域的中心画十字形标记


                if abs(300 - blob.cx()) > 20  or abs(300 - blob.cy()) > 20:
                    x_ange = Pid_x.update(300, blob.cx())  #pid计算
                    y_ange = Pid_y.update(300, blob.cy())  #pid计算
                    #print("x_ange:\r\n",x_ange)
                    #print("cx:\r\n", blob.cx())
                   # print("y_ange:\r\n",y_ange)

                    Set_Servo_x_angle(x_ange)
                    Set_Servo_y_angle(y_ange)
                else:
                    Set_Servo_x_angle(0)
                    Set_Servo_y_angle(0 )
                time.sleep_ms(20)


   #没发现目标回正
    else:
       cnt = cnt+1
       if cnt > 50:
           Set_Servo_x_angle(0)
           Set_Servo_y_angle(0)
           cnt = 0

    '''
    Set_Servo_x_angle(-10)
    Set_Servo_y_angle(-10)
    #Servo_y.pulse_width(1500);
    #Servo_x.pulse_width(1500);

    time.sleep_ms(100)

    Set_Servo_x_angle(10)
    Set_Servo_y_angle(10)

    #Servo_x.pulse_width(1700);
    #Servo_y.pulse_width(1700);
    time.sleep_ms(100)
    '''
