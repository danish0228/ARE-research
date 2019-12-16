import wiringpi as pi

import time
from PIL import Image
import numpy as np
import FaBo9Axis_MPU9250 as FB9
import qua
import picamera
import serial
from micropyGPS import MicropyGPS
import random


""""""""""""""""""""""""""""""

""""""""""""""""""""""""""""""

ser = serial.Serial('/dev/ttyUSB0', 9600)
r = 6378137#地球の半径
r_thr = 180 #赤色成分の閾値
ok_dif = 40 #許容可能なcansat中心と赤いコーンの写真上のpxの差
goal = np.array([0,0])#goal地点の緯度軽度
dropped_shock #投下後に生じる衝撃によってcansatの加速度センサに働く加速度
K #analyzeで用いる比例定数

class StopWatch:
    def __init__(self):
        self.sec_elapsed = 0
        self.start_time = None

    def start(self):
        if self.start_time == None:
            self.start_time = time.time()
        else:
            print("Error")
            return None

    def stop(self):
        if self.start_time != None:
            self.sec_elapsed += time.time() - self.start_time
            self.start_time = None

        else:
            print("Error")
            return None

    def _stop(self):
        return self.sec_elapsed + time.time() - self.start_time

    def time(self):
        if self.start_time == None:
            return self.sec_elapsed
        else:
            return self._stop()

    def reset(self):
        self.sec_elapsed = 0
        self.start_time = None
        
    def restart(self):
        self.reset()
        self.start()
        
sw = StopWatch()

class STEP:
    def __init__(self):#ファイル開け
        self.q1 = qua.quatanion() #姿勢quation
        self.gps=micropyGPS.MicropyGPS(9, 'dd')
        self.gpser=serial.Serial('/dev/serial0', 115200, timeout=10)##デバイス名は実験で確かめること
        self.file = open("log.txt","w") #cansatデータ保存
        self.MPU9250 = FB9.MPU9250()
        
    def Set_Offset(self):
        """この関数は必ず実行すること。でないとオフセット補正されない"""
        self.MPU9250.get_Offset()
        
    def send_String(self,data):
        """dataには任意の文字列を入力"""
        ser.write(data.encode())
        
    def read_MPU(self):
        """加速度,角速度,地磁気の値がlog.txtに保存され、データ送信される。self.Accel,self.Gyro,self.Magにも保存"""
        self.Accel = self.MPU9250.readAccel()
        textA = "Accel: {}, {}, {}".format(str(self.Accel[0]),str(self.Accel[1]),str(self.Accel[2]))
        self.file.write(textA)
        self.send_String(textA)
        
        self.Gyro = self.MPU9250.readGyro()
        textG = "Gyro: {}, {}, {}".format(str(self.Gyro[0]),str(self.Gyro[1]),str(self.Gyro[2]))
        self.file.write(textG)
        self.send_String(textG)
        
        self.Mag = self.MPU9250.readMagnet()
        textM = "Mag: {}, {}, {}".format(str(self.Mag[0]),str(self.Mag[1]),str(self.Mag[2]))
        self.file.write(textM)
        self.send_String(textM)
        
    def read_GPS(self):#numpy1次元配列
        self.gpser.readline()#最初の一行はデータが崩れることが多いので捨てるといいらしい
        t=self.gps.timestamp
        sentence = self.gpser.readline().decode('utf-8')#読み取った文字列を読める形式に変換
        while t==self.gps.timestamp:#NMEAフォーマットでは緯度経度と時刻は一行で出力されるので、タイムスタンプが更新されれば緯度経度も更新されてるはず？
            for x in sentence: 
                self.gps.update(x)#文字列を解析してオブジェクトの情報を更新
        return np.array([self.gps.latitude[0],self.gps.longtitude[0]])
    
    def drop_Detection(self):
        self.read_MPU()
        if np.sum(np.power(self.Accel,2)) > dropped_shock**2:
            return ture
        else:
            return false 
    
    def navi_to_Goal(self):
        """機体の真正面の方向とgoalの方向の角度がangle MPU9250を、加速度の3軸のうちx軸の正方向が機体の正面を向くように置いて欲しい"""
        position = self.read_GPS()
        distance = r*np.arccos(np.sin(position[1])*np.sin(goal[1])+np.cos(position[1])*np.cos(goal[1])*np.cos(goal[0]-position[0]))
        angle = 90 - np.rad2deg(np.arctan2(np.cos(position[1])*np.tan(goal[1])-np.sin(position[1])*np.cos(goal[0]-position[0]),np.sin(goal[0]-position[0])))
        self.update_quatanion(sw.time())
        yaw = self.q1.return_yaw() #姿勢quatanionが[1,0,0,0]の時は機体の正面が北の時だから、yaw角がそっくりそのまま北から機体正面までの角度になる
        angle = -angle if angle <= 180 else 360-angle
        
        angle = angle - theta
        return angle, distance
    
    def update_quatanion(self,dt):
        a = time.time()
        self.read_MPU()
        self.q1.update(self.Gyro,dt+a+time.time())
        
    def set_quatanion(self):
        self.read_MPU()
        self.q1._set_qua(self.Accel,self.Mag)
        
    def calibrate_quatanion(self):
        self.read_MPU()
        self.q1.calibrate(self.Accel,self.Mag)
        
class Move:
    def __init__(self):
        self.operation = 0
        self.rduty = 50
        self.lduty = 50
        self.e = np.zeros(3)
        self.operation = 0
        self.angle = 0
        self.mode = 0 #走行モード　1で前進、0で停止、-1で後退

        rmotor= 18 #右モータPWM
        lmotor= 13 #左モータPWM
        rsw=4 #正逆切り替えピン(右)
        lsw=5 #正逆切り替えピン(左)
        range = 1024
        dconv =0.01*range
        clock = 2
        pi.wiringPiSetupGpio()
        pi.pwmSetMode(pi.GPIO.PWM_MODE_BAL)
        pi.pinMode(rmotor,pi.PWM_OUTPUT)
        pi.pinMode(lmotor,pi.PWM_OUTPUT)
        pi.pinMode(rsw,pi.OUTPUT)
        pi.pinMode(lsw,pi.OUTPUT)
        pi.pwmSetClock(clock)
        Kp = 0
        Ki = 0
        Kd = 0
        
    def run(self,angle,mode):
        sw.start()
        if mode == 1:
            self.rduty = abs(self.rduty)
            self.lduty = abs(self.lduty)
        elif mode == 0:
            self.rduty = 0
            self,lduty = 0
        elif mode == -1:
            self.rduty = -abs(self.rduty)
            self.lduty = -abs(self.lduty)
        
        if self.angle != angle:
            self.angle = angle
            self.e[0] = self.angle
            
        start = time.time()
        while True:
            step.update_quatanion(sw.time())
            sw.restart()
            
            yaw = step.q1.return_yaw()
            delta = Move.Kp*(self.e[0]-self.e[1])+Move.Ki*self.e[0]+Move.Kd*(self.e[0]-2*self.e[1]+self.e[2])
            self.operation += delta
            if self.operation > 200:
                self.operation = 200
            elif self.operation < -200:
                self.operation = -200
            
            if self.operation < 0:
                if self.lduty-operation > 100:
                    a = -operation - (100 - self.rduty)
                    lduty = 100
                    rduty = self.rduty - a
                    self.moter(rduty,lduty)
                else:
                    lduty = self.lduty - operation
                    self.moter(rduty,lduty)
                    
            elif self.operation > 0:
                if self.rduty+operation > 100:
                    a = operation - (100 - self.lduty)
                    rduty = 100
                    lduty =self.lduty - a
                    self.moter(rduty,lduty)
                else:
                    rduty = self.rduty + operation
                    self.moter(rduty,lduty)
            
            step.update_quatanion(sw.time())
            sw.restart()
            yaw = step.q1.return_yaw() - yaw
            
            self.e[2] = self.e[1]
            self.e[1] = self.e[0]
            self.e[0] = self.e[0] - yaw
            
            """指定の角度までちゃんと曲がれたか（一応入れといた）"""
            if time.time() - start > 0.1:
                if e[0] < 5 and -5 < e[0]:
                    sw.reset()
                    return 1
                else:
                    sw.reset()
                    return 0
            
    def motor(self,rduty,lduty):
        pi.digitalWrite(rsw,rduty<0)
        pi.pwmWrite(rmotor,abs(int(rduty))*dconv)
        pi.digitalWrite(lsw,rduty<0)
        pi.pwmWrite(rmotor,abs(int(lduty))*dconv)
        
    def stack(self,before,after,n):
        disttance=np.linarg.norm(before-after)*r*1000 #GPSの緯度経度のデータ、60進法かとおもったら10進法に変換してくれてるっぽいのでそのままrをかけた　単位[m]
        return distance<(n*0.1) #n秒でn*10[cm]も進まなかったらスタックと判断(この値は変えるかも　マジックナンバーにしないほうが良い？)

        
    
    def stack_Escape(self):
        stack_curve = random.uniform(10,80)
        run(0,-1)
        time.sleep(3)
        run(stack_curve,0)
        time.sleep(10)
        run(0,1)
        time.sleep(5)

        
class Camera:
    def __init__(self):
        self.count = 0
        self.camera = picamera.PiCamera()
         
    def capture(self,analyze=False):
        """analyze用に写真を撮るならanalyze = true,analyze用ではないならfalse"""
        if analyze:
            filename = "picture_analze.jpg"     
            camera.capture(filename)
            return filename 
        else:
            filename = "picture" + str(count) + ".jpg"     
            camera.capture(filename)
            self.count += 1
            return filename 

    def analyze(self,picture):
        im = np.array(Image.open(picture))
        pos_array = np.zeros(im.shape[0])
        for i in range(im.shape[0]):
            for j in range(im.shape[1]):
                r_min = 0
                r_max = 0
                if im[i][j][0] >= r_thr:
                    if r_min == 0:
                        r_min = j+1
                    else:
                        r_max = j+1
            pos_array[i] = (r_min+r_max)/2.0
            
        a = pos_array
        """-1000000が帰ってきたらコーンが見つからなかったということー＞ランダムな角度曲がらせる必要がある"""
        if a[a!=0].size <= im.shapa[0]/20:
            return -1000000
        
        a = (a[a != 0]).mean()
        
        """１なら右に曲がれ、−１なら左に曲がれ、0ならそのままで良い"""
        if a > im.shape[1]/2 + ok_dif:
            return K*(a - im.shape[1]/2)
        elif a < im.shape[1]/2 - ok_dif:
            return K*(a - im.shape[1]/2)
        else:
            return 0
        
    def send_Pic(self, picture):
        a = Image.open(picture).resize((int(a.width/2),int(a.height/2)))
        picture = np.array(a)
        for i in range(picture.shape[0]):
            for k in range(picture.shape[1]):
                text = "{}, {}, {}".format(str(picture[i][k][0]), str(picture[i][k][1]), str(picture[i][k][2]))
                ser.write(text.encode())
            ser.write(b" ")
