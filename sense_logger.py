from sense_hat import SenseHat
import time
from math import cos, sin, radians, degrees, sqrt, atan2

"""

  IMU Data Logger !
  
  Simple logger that records Gyro, Accel, Mag, Temp
  
  Note: Requires sense_hat 2.2.0 or later

"""
logFile=open("/home/pi/Projects/VMS/sense_logger.csv","w+")
sense = SenseHat()

sense.clear()

startTime = time.time()
prevUpdateTime = 0

#
# Added Setup
#
horzMag = 11.8868
xMax = -100
yMax = -100
xMin = 100
yMin = 100
xScale = 1
yScale = 1
xBias = 14.9
yBias = 26.1
xSF = 0.72
ySF = 0.72

for i in range(1,101):
  gyroRaw = sense.get_gyroscope_raw()
  accelRaw = sense.get_accelerometer_raw()
  magRaw = sense.get_compass_raw()
  temp = sense.get_temperature()
####
# Main game loop
####
prevUpdateTime = time.time() - startTime
sense.clear()
while True:
 
  elapsedTime = time.time() - startTime
  gyroRaw = sense.get_gyroscope_raw()
  accelRaw = sense.get_accelerometer_raw()
  magRaw = sense.get_compass_raw()
  temp = sense.get_temperature()

#   print("0,{0},{x},{y},{z}".format(elapsedTime,**gyroRaw)) # rad/sec
#   print("1,{0},{x},{y},{z}".format(elapsedTime,**accelRaw)) # Gs
#   print("2,{0},{x},{y},{z}".format(elapsedTime,**magRaw))


  logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime,**gyroRaw)) # rad/sec
  logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime,**accelRaw)) # Gs
  logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime,**magRaw)) # microT
  logFile.write("7,{0},{1}\r\n".format(elapsedTime,temp))

  if magRaw["x"] > xMax:
      xMax = magRaw["x"]
  if magRaw["y"] > yMax:
      yMax = magRaw["y"]
  if magRaw["x"] < xMin:
      xMin = magRaw["x"]
  if magRaw["y"] < yMin:
      yMin = magRaw["y"]
  orientation = sense.get_orientation_degrees()
  if(orientation["pitch"] > 180):
      orientation["pitch"] = orientation["pitch"] - 360
  if(orientation["roll"] > 180):
      orientation["roll"] = orientation["roll"] - 360
      
  magCalx = xSF *(magRaw["x"] - xBias)
  magCaly = ySF *(magRaw["y"] - yBias)
  orientation["yaw"] = degrees(atan2(-1*magCaly,magCalx))
  logFile.write("3,{0},{pitch},{roll},{yaw}\r\n".format(elapsedTime,**orientation))
  
  selection = False
  events = sense.stick.get_events()
  for event in events:
      # Skip releases
      if event.action != "released":
        if event.direction == "middle":
          xMax = -100
          yMax = -100
          xMin = 100
          yMin = 100
          xScale = 1
          yScale = 1
#         elif event.direction == "right":
#           setTilt += 10
#           selection = True
#         elif event.direction == "up":
#           (drive1,drive2,motorPwm) = drive_motor(25)
#           selection = True
#         elif event.direction == "down":
#           (drive1,drive2,motorPwm) = drive_motor(-25)
#           selection = True
#         elif event.direction == "middle":
#           (drive1,drive2,motorPwm) = drive_motor(0)
#           selection = True
          
  if (elapsedTime - prevUpdateTime)>1.0:
    print("0,{0},{x},{y},{z}".format(elapsedTime,**gyroRaw)) # rad/sec
    print("1,{0},{x},{y},{z}".format(elapsedTime,**accelRaw)) # Gs
    print("2,{0},{x},{y},{z}".format(elapsedTime,**magRaw))
    print("3,{0},{pitch},{roll},{yaw}".format(elapsedTime,**orientation))
    print("7,{0},{1}".format(elapsedTime,temp))
#     xSpan = xMax-xMin
#     ySpan = yMax-yMin
#     xBias = xMin+xSpan/2
#     yBias = yMin+ySpan/2
#     xSF = horzMag/(xSpan/2)
#     ySF = horzMag/(ySpan/2)
#     print("X   : {0} {1}".format(xMin,xMax))
#     print("Y   : {0} {1}".format(yMin,yMax))
#     print("Bias: {0} {1}".format(xBias,yBias))
#     print("SF  : {0} {1}".format(xSF,ySF))
    print("")
    prevUpdateTime = elapsedTime 

