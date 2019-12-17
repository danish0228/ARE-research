from time improt sleep
import picamera
from PIL import Image, ImageDraw, ImageFront
import datetime

camera = picamera.PiCamera()
for i in range(1*60*60*1):
 sleep(0.5)
 file_name = "{0:08}.jpg".format(i)
camera.capture("/home/pi/camera/img/"+file_name)
font = ImageFront.truetype('/home/pi/camra/OpenSans-Light.ttf',12)
img = Image.open("/home/pi/camera/img/" + file_name, 'r')
draw = ImageDraw.Draw(img)
draw.text((10,10), str(datetime.datetime.now()),front=front, fill='#F00')
img.save("/home/pi/camera/img/" + file_name)
print(file_name)


 
