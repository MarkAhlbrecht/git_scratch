from sense_hat import SenseHat
import time
from math import cos, sin, radians, degrees, sqrt, atan2
import PID
import RPi.GPIO as GPIO



"""

  Tilt Level w control system outputs
  
  Select Temperature, Pressure, or Humidity  with the Joystick
  to visualize the current sensor values on the LED.
  
  Note: Requires sense_hat 2.2.0 or later

"""
#
# Setup Motor Control Pins
#
GPIO.setmode(GPIO.BOARD)

GPIO.setup(11, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(7, GPIO.OUT)
pwm=GPIO.PWM(7, 100)
pwm.start(0)
drive1 = 0
drive2 = 0
motorPwm = 0
sense = SenseHat()

green = (0, 255, 0)
red = (255, 0, 0)
blue = (0, 0, 255)
white = (255, 255, 255)

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

def drive_motor(omegaMotor):
  deadBand = 0;
  Kpwm = 1;
  if abs(omegaMotor) > deadBand:
    drive1 = omegaMotor > 0
    drive2 = not drive1
    motorPwm = abs(Kpwm*omegaMotor)
    if motorPwm > 100:
      motorPwm = 100
    #
    # Command Motor Pins
    #
  else:
    drive1 = False
    drive2 = False
    motorPwm = 0
  #
  # Command Motor Pins
  #

  return (drive1,drive2,motorPwm)
  
index = 0
sensors = ["temp", "pressure", "humidity"]
startTime = time.time()
elapsedTime = 0
logFile=open("/home/pi/Projects/VMS/tilt_control.csv","w+")  
sense.clear()
Ksensor = (1/40)
tiltHybrid = 0;
print(Ksensor)
print(1-Ksensor)

setTilt = 0
Kp = -1;
Kd = 0;
Ki = 0;
tiltController = PID.PID()
tiltController.SetKp(Kp)
tiltController.SetKi(Ki)
tiltController.SetKd(Kd)



print("Warming up ...")
while elapsedTime < 10:
    elapsedTime = time.time() - startTime
    drive_motor(100)

print("Aligning ...")
accelRaw = sense.get_accelerometer_raw()
tiltAccel = degrees(atan2(accelRaw["y"],-1*accelRaw["x"]));
tiltInit = tiltAccel
n=1

while elapsedTime < 15:
    elapsedTime = time.time() - startTime
    n=n+1
    accelRaw = sense.get_accelerometer_raw()
    tiltAccel = degrees(atan2(accelRaw["y"],-1*accelRaw["x"]));
    tiltInit = (n-1)/n*tiltInit + (1/n)*tiltAccel
    drive_motor(-100)
    prevTime = elapsedTime

print("Initial Tilt = {0}".format(tiltInit))
tiltHybrid = tiltInit
tiltGyro = tiltInit
time.sleep(1)
prevUpdateTime = elapsedTime
while True:
    
    # Collect, print and log raw data
    elapsedTime = time.time() - startTime
    gyroRaw = sense.get_gyroscope_raw()
    accelRaw = sense.get_accelerometer_raw()
#     magRaw = sense.get_compass_raw()
#     orientation = sense.get_orientation_degrees()
#     print("0,{0},{x},{y},{z}".format(elapsedTime,**gyroRaw)) # rad/sec
#     print("1,{0},{x},{y},{z}".format(elapsedTime,**accelRaw)) # Gs
#   print("2,{0},{x},{y},{z}".format(elapsedTime,**magRaw))
    logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime,**gyroRaw)) # rad/sec
    logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime,**accelRaw)) # Gs
#     logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime,**magRaw)) # microT

    # Calulate Accel and Gyro based tilts
    
    tiltAccel = degrees(atan2(accelRaw["y"],-1*accelRaw["x"]))
    dt = elapsedTime - prevTime
    tiltGyro = tiltGyro + degrees(gyroRaw["z"])*dt
    tiltHybrid = (1-Ksensor)*(tiltHybrid + degrees(gyroRaw["z"])*dt) + Ksensor*tiltAccel

#      print("4,{0},{1},{2},{3}".format(elapsedTime,tiltHybrid,tiltAccel,tiltGyro))
    logFile.write("4,{0},{1},{2},{3}\r\n".format(elapsedTime,tiltHybrid,tiltAccel,tiltGyro))
    
    # Controller
    error = tiltHybrid - setTilt
    ctrlOutput = tiltController.GenOut(error)
#     print("5,{0},{1},{2},{3},{4}".format(elapsedTime,tiltHybrid,setTilt,error,ctrlOutput))
    logFile.write("5,{0},{1},{2},{3},{4}\r\n".format(elapsedTime,tiltHybrid,setTilt,error,ctrlOutput))    
    
    # Motor Drive
#     (drive1,drive2,motorPwm) = drive_motor(ctrlOutput)
    GPIO.output(11, drive1)
    GPIO.output(12, drive2)
    pwm.ChangeDutyCycle(motorPwm)
    #     print("6,{0},{1},{2},{3},{4},{5}".format(elapsedTime,error,ctrlOutput,drive1,drive2,pwm))
    logFile.write("6,{0},{1},{2},{3},{4},{5}\r\n".format(elapsedTime,error,ctrlOutput,drive1,drive2,motorPwm))    
   
    
    selection = False
    events = sense.stick.get_events()
    for event in events:
      # Skip releases
      if event.action != "released":
        if event.direction == "left":
          setTilt -= 10
          selection = True
        elif event.direction == "right":
          setTilt += 10
          selection = True
        elif event.direction == "up":
          (drive1,drive2,motorPwm) = drive_motor(25)
          selection = True
        elif event.direction == "down":
          (drive1,drive2,motorPwm) = drive_motor(-25)
          selection = True
        elif event.direction == "middle":
          (drive1,drive2,motorPwm) = drive_motor(0)
          selection = True
#         if selection:
#           current_mode = sensors[index % 3]
#           update_screen(current_mode, show_letter = True)
  

    if (elapsedTime - prevUpdateTime)>0:
      print("5,{0},{1},{2},{3},{4}".format(elapsedTime,tiltHybrid,setTilt,error,ctrlOutput))
      print("6,{0},{1},{2},{3},{4},{5}".format(elapsedTime,error,ctrlOutput,drive1,drive2,motorPwm))

      prevUpdateTime = elapsedTime
    
    
    
    prevTime = elapsedTime
#     time.sleep(0.5)

GPIO.output(7, False)

pwm.stop()

GPIO.cleanup()
    
  
  
    