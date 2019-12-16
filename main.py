import STEP
import wiringpi as pi
import time


step = STEP.STEP()
camera = STEP.Camera()

step.Set_Offset()#シリアル通信を通してパソコンに指示が送信されるからよく見ろ
    
camera.send_Pic(camera.capture())
