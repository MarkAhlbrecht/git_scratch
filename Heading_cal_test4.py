from sense_hat import SenseHat
import time
from math import cos, sin, radians, degrees, sqrt, atan2
import RPi.GPIO as GPIO
from time import sleep

"""

  Heading cal Program
  
  Rotate horizontal wheel 30 degrees and then collect Pihat sensor data
  Repeate for 1 full cycle, 12 spots
  
  Note: Requires sense_hat 2.2.0 or later mounted to Pi.  
  Pi/sense Hat is mounted to a flat, horizontal wheel that is controlled by a motor
  Horizontal wheel should be a level as possible.
  Starting position of PiHat on wheel should be aligned with magnetic north

"""
#
# Setup Motor Control Pins
#
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(7, GPIO.OUT)
#
# inialize motor
#
GPIO.output(11, False)  # Stop motor
GPIO.output(12, False)
pwm=GPIO.PWM(7, 100)
pwm.start(0)
pwm.ChangeDutyCycle(100)
GPIO.output(7, True)

#
# inialize sense hat
#
sense = SenseHat()

#
# inialize varables
#
logFile = open("/home/pi/Projects/VMS/Heading_cal_log.csv","w+")
iloop = 1
startTime = time.time()
elapsedTime = 0
logtime = 0
n = 1
duration = 0.985

#
# warm up IMU and log 2 min of data
#
print("Warming up for 2 min ...")
while elapsedTime < 120:
    elapsedTime = time.time() - startTime
    gyroRaw = sense.get_gyroscope_raw()
    accelRaw = sense.get_accelerometer_raw()
    magRaw = sense.get_compass_raw()
    temp = sense.get_temperature()
    pressure = sense.get_pressure()
    humidity = sense.get_humidity()
    logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime, **gyroRaw))  # rad/sec
    logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime, **accelRaw))  # Gs
    logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime, **magRaw))  # microT
    logFile.write("3,{0},{1},{2},{3}\n".format(elapsedTime, temp, pressure, humidity))  # temp/press/humd
        
#
# Start loop for collecting Data
# Motor sleep time is set to move 30 degrees
#
while (iloop < 13):
    sampleStart = time.time()
    sampleTime = 0
    
    print('Logging 10 second data for loop ' + str(iloop))
    while sampleTime < 10:  # dwell 10 seconds at each 30 degree spot, read and log
        sampleTime = time.time() - sampleStart
        elapsedTime = time.time() - startTime
        gyroRaw = sense.get_gyroscope_raw()
        accelRaw = sense.get_accelerometer_raw()
        magRaw = sense.get_compass_raw()
        temp = sense.get_temperature()
        pressure = sense.get_pressure()
        humidity = sense.get_humidity()
        logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime, **gyroRaw))  # rad/sec
        logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime, **accelRaw))  # Gs
        logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime, **magRaw))  # microT
        logFile.write("3,{0},{1},{2},{3}\n".format(elapsedTime, temp, pressure, humidity))  # temp/press/humd

    print("Advance +30 degrees")
    GPIO.output(11, True)   # start motor
    GPIO.output(12, False)
    
    sampleStart = time.time()
    sampleTime = 0
    while sampleTime < duration:     # Adjust this sleep delay to get as close to 30 degree as possible
        sampleTime = time.time() - sampleStart
        elapsedTime = time.time() - startTime
        gyroRaw = sense.get_gyroscope_raw()
        accelRaw = sense.get_accelerometer_raw()
        magRaw = sense.get_compass_raw()
        temp = sense.get_temperature()
        pressure = sense.get_pressure()
        humidity = sense.get_humidity()
        logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime, **gyroRaw))  # rad/sec
        logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime, **accelRaw))  # Gs
        logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime, **magRaw))  # microT
        logFile.write("3,{0},{1},{2},{3}\n".format(elapsedTime, temp, pressure, humidity))  # temp/press/humd
        
    GPIO.output(11, False)  # Stop motor
    GPIO.output(12, False)
    iloop = iloop+1

# Go Back the other direction
print("Go Back the other direction")
iloop = 0

#
# Start loop for collecting Data
# Motor sleep time is set to move 30 degrees
#
while (iloop < 13):
    sampleStart = time.time()
    sampleTime = 0
    
    print('Logging 10 second data for loop ' + str(iloop))
    while sampleTime < 10:  # dwell 10 seconds at each 30 degree spot, read and log
        sampleTime = time.time() - sampleStart
        elapsedTime = time.time() - startTime
        gyroRaw = sense.get_gyroscope_raw()
        accelRaw = sense.get_accelerometer_raw()
        magRaw = sense.get_compass_raw()
        temp = sense.get_temperature()
        pressure = sense.get_pressure()
        humidity = sense.get_humidity()
        logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime, **gyroRaw))  # rad/sec
        logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime, **accelRaw))  # Gs
        logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime, **magRaw))  # microT
        logFile.write("3,{0},{1},{2},{3}\n".format(elapsedTime, temp, pressure, humidity))  # temp/press/humd
        
    print("Advance +30 degrees")
    GPIO.output(11, False)   # start motor
    GPIO.output(12, True)
    
    sampleStart = time.time()
    sampleTime = 0
    while sampleTime < duration:     # Adjust this sleep delay to get as close to 30 degree as possible
        sampleTime = time.time() - sampleStart
        elapsedTime = time.time() - startTime
        gyroRaw = sense.get_gyroscope_raw()
        accelRaw = sense.get_accelerometer_raw()
        magRaw = sense.get_compass_raw()
        temp = sense.get_temperature()
        pressure = sense.get_pressure()
        humidity = sense.get_humidity()
        logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime, **gyroRaw))  # rad/sec
        logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime, **accelRaw))  # Gs
        logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime, **magRaw))  # microT
        logFile.write("3,{0},{1},{2},{3}\n".format(elapsedTime, temp, pressure, humidity))  # temp/press/humd
        
    GPIO.output(11, False)  # Stop motor
    GPIO.output(12, False)
    iloop = iloop+1

#
# warm up IMU and log 2 min of data
#
print("Collecting 2 min more data ...")
while elapsedTime < 120:
    elapsedTime = time.time() - startTime
    gyroRaw = sense.get_gyroscope_raw()
    accelRaw = sense.get_accelerometer_raw()
    magRaw = sense.get_compass_raw()
    temp = sense.get_temperature()
    pressure = sense.get_pressure()
    humidity = sense.get_humidity()
    logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime, **gyroRaw))  # rad/sec
    logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime, **accelRaw))  # Gs
    logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime, **magRaw))  # microT
    logFile.write("3,{0},{1},{2},{3}\n".format(elapsedTime, temp, pressure, humidity))  # temp/press/humd
        
#
# Start loop for collecting Data
# Motor sleep time is set to move 30 degrees
#
while (iloop < 13):
    sampleStart = time.time()
    sampleTime = 0
    
    print('Logging 10 second data for loop ' + str(iloop))
    while sampleTime < 10:  # dwell 10 seconds at each 30 degree spot, read and log
        sampleTime = time.time() - sampleStart
        elapsedTime = time.time() - startTime
        gyroRaw = sense.get_gyroscope_raw()
        accelRaw = sense.get_accelerometer_raw()
        magRaw = sense.get_compass_raw()
        temp = sense.get_temperature()
        pressure = sense.get_pressure()
        humidity = sense.get_humidity()
        logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime, **gyroRaw))  # rad/sec
        logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime, **accelRaw))  # Gs
        logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime, **magRaw))  # microT
        logFile.write("3,{0},{1},{2},{3}\n".format(elapsedTime, temp, pressure, humidity))  # temp/press/humd

    print("Advance +30 degrees")
    GPIO.output(11, True)   # start motor
    GPIO.output(12, False)
    
    sampleStart = time.time()
    sampleTime = 0
    while sampleTime < duration:     # Adjust this sleep delay to get as close to 30 degree as possible
        sampleTime = time.time() - sampleStart
        elapsedTime = time.time() - startTime
        gyroRaw = sense.get_gyroscope_raw()
        accelRaw = sense.get_accelerometer_raw()
        magRaw = sense.get_compass_raw()
        temp = sense.get_temperature()
        pressure = sense.get_pressure()
        humidity = sense.get_humidity()
        logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime, **gyroRaw))  # rad/sec
        logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime, **accelRaw))  # Gs
        logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime, **magRaw))  # microT
        logFile.write("3,{0},{1},{2},{3}\n".format(elapsedTime, temp, pressure, humidity))  # temp/press/humd
        
    GPIO.output(11, False)  # Stop motor
    GPIO.output(12, False)
    iloop = iloop+1

# Go Back the other direction
print("Go Back the other direction")
iloop = 0

#
# Start loop for collecting Data
# Motor sleep time is set to move 30 degrees
#
while (iloop < 13):
    sampleStart = time.time()
    sampleTime = 0
    
    print('Logging 10 second data for loop ' + str(iloop))
    while sampleTime < 10:  # dwell 10 seconds at each 30 degree spot, read and log
        sampleTime = time.time() - sampleStart
        elapsedTime = time.time() - startTime
        gyroRaw = sense.get_gyroscope_raw()
        accelRaw = sense.get_accelerometer_raw()
        magRaw = sense.get_compass_raw()
        temp = sense.get_temperature()
        pressure = sense.get_pressure()
        humidity = sense.get_humidity()
        logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime, **gyroRaw))  # rad/sec
        logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime, **accelRaw))  # Gs
        logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime, **magRaw))  # microT
        logFile.write("3,{0},{1},{2},{3}\n".format(elapsedTime, temp, pressure, humidity))  # temp/press/humd
        
    print("Advance +30 degrees")
    GPIO.output(11, False)   # start motor
    GPIO.output(12, True)
    
    sampleStart = time.time()
    sampleTime = 0
    while sampleTime < duration:     # Adjust this sleep delay to get as close to 30 degree as possible
        sampleTime = time.time() - sampleStart
        elapsedTime = time.time() - startTime
        gyroRaw = sense.get_gyroscope_raw()
        accelRaw = sense.get_accelerometer_raw()
        magRaw = sense.get_compass_raw()
        temp = sense.get_temperature()
        pressure = sense.get_pressure()
        humidity = sense.get_humidity()
        logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime, **gyroRaw))  # rad/sec
        logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime, **accelRaw))  # Gs
        logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime, **magRaw))  # microT
        logFile.write("3,{0},{1},{2},{3}\n".format(elapsedTime, temp, pressure, humidity))  # temp/press/humd
        
    GPIO.output(11, False)  # Stop motor
    GPIO.output(12, False)
    iloop = iloop+1

#
# Stop motor
#   
sleep(2)
GPIO.output(7, False)
pwm.stop()

#
# Clean up GPIO and SenseHat to exit
#
GPIO.cleanup()
sense.clear()
logFile.close()

