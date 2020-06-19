import time
import cv2
import numpy as np
import math
import serial
import struct

def  hypotenuse(a,b):#三角函数
    return(math.sqrt(a**2+b**2))


def findAromr(contours,maxH,minH,maxW,maxDLRatio,minDLRatio,maxLenRatio,minLenRatio):#40 5 20 3.0 2.0,1.5 0.5
    if contours:
        for cnt in contours:
            x1,y1,w1,h1 = cv2.boundingRect(cnt)#从一个闭合的轮廓中找到外接矩形
            if h1 <maxH and h1>minH and w1<maxW:#筛选适合的单个灯条
                lengthOne = hypotenuse(w1,h1)#计算灯条的长度（计算矩形的对角线）
                #cv2.rectangle(image,(x1,y1),(x1+w1,y1+h1),(255,0,0),3)
                for cnt in contours:
                    x2,y2,w2,h2 = cv2.boundingRect(cnt)#查找其他的灯条的外接矩形
                    if h2 <maxH and h2>minH and w2<maxW:#筛选适合的单个灯条
                        lengthTwo = hypotenuse(w2,h2)#计算灯条的长度（计算矩形的对角线）
                        distance = abs(x2 - x1)#计算与上一个灯条的距离
                       # print('distance/lengthOne:',distance/lengthOne)
                        #cv2.rectangle(image,(x2,y2),(x2+w2,y2+h2),(0,255,255),3)#画框
                        if distance/lengthOne >minDLRatio and distance/lengthOne <maxDLRatio:#判断长宽比
                            if lengthOne/lengthTwo >minLenRatio and lengthOne/lengthTwo < maxLenRatio:#判断两灯条长度比例
                                #print('distance/lengthOne:',distance/lengthOne)
                                CX = (x1 + x2 + (w1 + w2)/2)/2#相对的中点坐标
                                CY = (y1 + y2 + (h1 + h2)/2)/2
                                cv2.circle(image,(int(CX),int(CY)),8,(255,255,0),2)
                                return 1,CX,CY,distance#是否识别，中点坐标
    return 0,0,0,0

def forecastMove(x, y):#卡尔曼预测
    global current_measurement,current_prediction,last_prediction
    last_prediction = current_prediction # 把当前预测存储为上一次预测
    current_measurement = np.array([[np.float32(x)], [np.float32(y)]]) # 当前测量
    kalman.correct(current_measurement) # 用当前测量来校正卡尔曼滤波器
    current_prediction = kalman.predict() # 计算卡尔曼预测值，作为当前预测
    lpx, lpy = last_prediction[0], last_prediction[1] # 上一次预测坐标
    cpx, cpy = current_prediction[0], current_prediction[1] # 当前预测坐标
    return cpx ,cpy,lpx,lpy

# 初始化测量坐标和运动预测的数组
current_measurement = np.array((2, 1), np.float32)
current_prediction = np.zeros((2, 1), np.float32)
kalman = cv2.KalmanFilter(4, 2) # 4：状态数，包括（x，y，dx，dy）坐标及速度（每次移动的距离）；2：观测量，能看到的是坐标值
kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32) # 系统测量矩阵
kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32) # 状态转移矩阵
kalman.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)*0.1 # 系统过程噪声协方差

width = 640
heigh = 480

# 打开串口
ser = serial.Serial("/dev/ttyAMA0", 115200)

#初始化相机的参数和二值化的阀值

cap1 = cv2.VideoCapture(0)
ret=cap1.set(3,width)
ret=cap1.set(4,heigh)

ret=cap1.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
#ret=cap1.set(5,5)
print(cap1.get(5))
lower_blue = np.array([0,0, 255])#0 0 255
upper_blue = np.array([255, 255, 255])#100 60 255
kernel1 = np.ones((2,1),np.uint8)#去除噪声的积卷核
kernel2 = np.ones((4,2),np.uint8)#放大的积卷核

#变量
CX = 0#预测前的xy相对坐标
CY = 0
lengthOne = 0#两个装甲板的高度
lengthTwo = 0
distance = 0#两个装甲板直接的距离
lastDistance = 0
flag = 0#失败成功标识符
x=0#预测后的xy绝对坐标
y=0
xlast=0#for Kalman predict next track
ylast=0
count = 0#记录识别失败的帧数
cutFlag = 0#the ROI flag
sendX = 0
sendY = 0
startTime = time.time()#caculate the fps
frame = 0
fps = 0
#主程序
while 1:
    ret,imageRAW = cap1.read()#读取一帧
    frame = frame + ret
    #flag是只有找到才会裁剪，小于20帧是加入预判的20  count<20
    try:
        
        if flag and int(x-lastDistance*2) > 0 and int(y+lastDistance) < heigh and int(y-lastDistance*2) > 0 and int(x+lastDistance) < width:
            cutFlag = 1
            image = imageRAW[int(y-lastDistance):int(y+lastDistance),int(x-lastDistance*2):int(x+lastDistance*2)]
            #cv2.imshow("armor", image)
            #print(int(y-lastDistance),int(y+lastDistance),int(x-lastDistance),int(x+lastDistance),lastDistance)
        else:
            image = imageRAW
            cutFlag = 0
    #cv2.imshow("armor", image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)#二值化
        
    except:
        
        cap1 = cv2.VideoCapture(0)
        ret,imageRAW = cap1.read()#读取一帧
        if ret:
            ret=cap1.set(3,width)
            ret=cap1.set(4,heigh)
            ret=cap1.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
            time.sleep(1)
            continue
        else:
            cap1 = cv2.VideoCapture(1)
            ret=cap1.set(3,width)
            ret=cap1.set(4,heigh)
            ret=cap1.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
            time.sleep(1)
            continue
        
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)#二值化
    img = cv2.inRange(hsv, lower_blue, upper_blue)
    
    erosion = cv2.erode(img,kernel1,iterations = 1)#去除噪声
    #opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    dilation = cv2.dilate(erosion,kernel2,iterations = 2)#放大信号
    #cv2.imshow("h", dilation)
    f,contours,hierarchy = cv2.findContours(dilation,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)#找到所有轮廓
    flag,CX,CY,distance=findAromr(contours,50,5,20,2.5,1.6,1.4,0.6)#返回是否识别到和相对坐标   

    if flag == 0:#如果没有找到
        count = count+1#累加识别失败的帧数
        if count < 20:#如果没有达到阀值，那么用上一次预测的值再带入，预测出下一帧的数据
            x,y,xlast,ylast = forecastMove(xlast,ylast)
            cv2.circle(imageRAW,(int(x),int(y)),3,(0,0,255),5)
            
        else:#如果超过了阀值，认定为识别失败
            cv2.putText(imageRAW,'TARGET LOST', (100,200), 0, 2, (0,255,255), 3)
            x,y = (320,240)
    else:#识别到了，将坐标带入预测，得到预测后的值
        count = 0
        lastDistance = distance
        if cutFlag:
            x,y,xlast,ylast = forecastMove(x-lastDistance*2+CX,y-lastDistance+CY)
        else:
            x,y,xlast,ylast = forecastMove(CX,CY)
        
        cv2.circle(imageRAW,(int(x),int(y)),3,(0,0,255),5)
    if time.time() - startTime > 1:
        fps = frame
        frame = 0
        startTime = time.time()
    
    if  abs(width/2-x)>100:
        sendX = (width/2-x)/3
    else:
        sendX = width/2-x
    if  abs(heigh/2-y)>80:
        sendY = (heigh/2-y)/2
    else:
        sendY = (heigh/2-y)
    buffer = struct.pack("ff",float(sendX)*0.035,-float(sendY)*0.02)
    ser.write(buffer)
    print(width/2-x,heigh/2-y,fps)
    #cv2.putText(imageRAW,'FPS:%d'%fps, (width-200,heigh-100), 0, 1, (255,255,255), 2)
    cv2.imshow("hsv", imageRAW)
    cv2.waitKey(1)

    

