# 自动灰度颜色追踪例程
#
# 这个例子展示了使用OpenMV的单色自动灰度色彩跟踪。

import sensor, image, time
from pyb import UART
import struct
import math
sensor.reset()

sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.VGA)
sensor.set_vflip(True)
sensor.skip_frames(time = 2000)

sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()
uart = UART(3, 115200)
threshold = [250, 255] # Middle grayscale values.

filterNum = 3
index = 0
inputsX = [0]*filterNum
inputsY = [0]*filterNum
total = 0

yawDetal = 0
pitchDetal = 0

def Pythagorean_theorem(a,b):
    return (a**2 + b**2)**(1/2)

while(True):
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs([threshold],y_stride=20,pixels_threshold=30, area_threshold=30, merge=True, margin=30)
    if blobs:
        for blob in blobs:
            FirstLength = Pythagorean_theorem(blob.w(),blob.h())

            if FirstLength < 50 and FirstLength >5 :
                length = int( Pythagorean_theorem(blob.w(),blob.h())*2.5)
                left_roi = [blob.cx()-length,blob.cy()-length,length*2,length*2]
                #print(FirstLength)
                #img.draw_rectangle(left_roi,thickness = 2)
                for amor in img.find_blobs([threshold],roi=left_roi,y_stride=20,pixels_threshold=30, area_threshold=30, merge=True, margin=30):
                    if abs(amor.x() -blob.x()) > FirstLength*2.0:
                        print(abs(amor.x() -blob.x())/FirstLength)
                        ThirdLength = Pythagorean_theorem(amor.w(),amor.h())
                        rate = FirstLength/ThirdLength

                        if rate > 0.9 and rate < 1.1:

                            inputsX[index] = (amor.cx()+blob.cx())/2
                            inputsY[index] = (amor.cy()+blob.cy())/2
                            index = index+1
                            if index >= filterNum:
                                index = 0
                            for i in inputsX:
                                total = total + i
                            NewCx = int(total/filterNum)
                            total = 0
                            for i in inputsY:
                                total = total + i
                            NewCy = int(total/filterNum)
                            total = 0
                            img.draw_cross(NewCx, NewCy,size = 10)
                            img.draw_rectangle(amor.rect())
                            yawDetal = (NewCx - 320)/203.82
                            pitchDetal = (NewCy - 240)/152.87

    else:
        yawDetal = 0;
        pitchDetal = 0;

   #print('yaw %f ;pit %f' %(math.sin(yawDetal),math.sin(pitchDetal)))
    buffer = struct.pack("ff",math.sin(yawDetal)*0.0005,math.sin(pitchDetal)*0.0003)
    #buffer = struct.pack("ff", -float(yawDetal)*0.000005,float(pitchDetal)*0.000004)
    #buffer = struct.pack("ff", float(yawDetal)*0.00001,float(pitchDetal)*0.00001)
    uart.write(buffer)
