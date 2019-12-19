from sense_hat import SenseHat
import time
from math import cos, sin, radians, degrees, sqrt, atan2

"""

  Sense HAT Sensors Display !
  
  Select Temperature, Pressure, or Humidity  with the Joystick
  to visualize the current sensor values on the LED.
  
  Note: Requires sense_hat 2.2.0 or later

"""

sense = SenseHat()

green = (0, 255, 0)
red = (255, 0, 0)
blue = (0, 0, 255)
white = (255, 255, 255)

inCal = (-4,2,43.5,71,93.5,114,137,162,186,214,242.5,269,311,356,360)
outCal = (0,0,30,60,90,120,150,180,210,240,270,300,360,360)
calData = ( (0, -4), \
            (0,  2), \
            (30, 43.5), \
            (60, 71), \
            (90, 93.5), \
            (120, 114), \
            (150, 137), \
            (180, 162), \
            (210, 186), \
            (240, 214), \
            (270, 242.5), \
            (300, 269), \
            (330, 311), \
            (360, 356), \
            (360, 360) )

sense.clear()
sense.set_pixel(0,0,green)
sense.set_pixel(7,0,blue)
sense.set_pixel(0,7,white)
sense.set_pixel(7,7,red)
time.sleep(5)

def show_t():
  sense.show_letter("T", back_colour = red)
  time.sleep(.5)

def show_p():
  sense.show_letter("P", back_colour = green)
  time.sleep(.5)

def show_h():
  sense.show_letter("H", back_colour = blue)
  time.sleep(.5)

def update_screen(angle, show_letter = False):
  
  yorig = 3
  xorig = 3
  ca = cos(radians(angle))
  sa = sin(radians(angle))
  sense.clear()
  
  
  for l in range(-10,0):
    x = int(ca*float(l)/2 + xorig)
    y = int(sa*float(l)/2 + yorig)
    if x >= 0 and x <= 7 and y >= 0 and y <= 7:
      sense.set_pixel(x,y,white)
  for l in range(0,10):
    x = int(ca*float(l)/2 + xorig)
    y = int(sa*float(l)/2 + yorig)
    if x >= 0 and x <= 7 and y >= 0 and y <= 7:
      sense.set_pixel(x,y,red)
  sense.set_pixel(xorig,yorig,green)

####
# Intro Animation
####

show_t()
show_p()
show_h()

# update_screen("temp")

index = 0
sensors = ["temp", "pressure", "humidity"]
startTime = time.time()
logFile=open("/home/pi/Projects/VMS/orientation_raw.csv","w+")
####
# Main game loop
####

sense.clear()
while True:
#   selection = False
#   events = sense.stick.get_events()
#   for event in events:
#     # Skip releases
#     if event.action != "released":
#       if event.direction == "left":
#         index -= 1
#         selection = True
#       elif event.direction == "right":
#         index += 1
#         selection = True
#       if selection:
#         current_mode = sensors[index % 3]
#         update_screen(current_mode, show_letter = True)
#   
#   if not selection:      
#     current_mode = sensors[index % 3]
#     update_screen(current_mode)

  
  elapsedTime = time.time() - startTime
  gyroRaw = sense.get_gyroscope_raw()
  accelRaw = sense.get_accelerometer_raw()
  magRaw = sense.get_compass_raw()
  orientation = sense.get_orientation_degrees()

#   print("0,{0},{x},{y},{z}".format(elapsedTime,**gyroRaw)) # rad/sec
#   print("1,{0},{x},{y},{z}".format(elapsedTime,**accelRaw)) # Gs
#   print("2,{0},{x},{y},{z}".format(elapsedTime,**magRaw))

#   logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime,**gyroRaw)) # rad/sec
#   logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime,**accelRaw)) # Gs
  logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime,**magRaw)) # microT
  

  orientation = sense.get_orientation_degrees()
  if(orientation["pitch"] > 180):
      orientation["pitch"] = orientation["pitch"] - 360
  if(orientation["roll"] > 180):
      orientation["roll"] = orientation["roll"] - 360
      
  print("3,{0},{pitch},{roll},{yaw}".format(elapsedTime,**orientation))
#   yawRaw = orientation["yaw"]
#   for idx in range(len(inCal)):
#       if yawRaw < inCal[idx]:
#           break
#   print(idx)
#   delta = (yawRaw - inCal[idx-1])/(inCal[idx]-inCal[idx-1])
#   print(delta)
#   calYaw = outCal[idx-1] + delta * (outCal[idx] - outCal[idx-1])
#   orientation["yaw"] = calYaw
#   print("ot 3,{0},{1},{2},{3}".format(elapsedTime,orientation["pitch"],orientation["roll"],calYaw))
  logFile.write("3,{0},{pitch},{roll},{yaw}".format(elapsedTime,**orientation))
  
#   
#   xTiltComp = -1*accelRaw["x"]
#   yTiltComp = accelRaw["y"]
#   xyTiltMag = sqrt(xTiltComp**2 + yTiltComp**2)
#   xyTilt = degrees(atan2(xTiltComp/xyTiltMag,yTiltComp/xyTiltMag))
#   print(xyTilt)
#   update_screen(float(-1*orientation["yaw"]))
#   update_screen(float(-1*orientation["pitch"]))
#   update_screen(float(xyTilt)-90.0)
#   time.sleep(0.1)
  
#   #north = sense.get_compass()
#   north = 0
#   print("{0},{1},{2},{3},{4}".format(elapsedTime,orientation["pitch"],orientation["roll"],orientation["yaw"], north))
#   time.sleep(10)
  