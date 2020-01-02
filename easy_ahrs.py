from sense_hat import SenseHat
import numpy as np
import time
from math import cos, sin, radians, degrees, sqrt, atan2, asin

"""

  Easy AHRS Code
  
  
  
  Note: Requires sense_hat 2.2.0 or later

"""
#
# Configuration
#
#####
# Predetermined Mag Calibraton
####
xBias = 19.31
yBias = 32.12
xSF = 0.705
ySF = 0.693
#
# Log file name
#
logFile=open("/home/pi/Projects/git_scratch/VMS/easy_raw.csv","w+")

#
# AHRS Smoothing flither time constant
tau = 1/40
"""
"   c2euler
"   This routine converts euler measurments to C matrix representations
"   Input: pitch roll yaw
"   Output: C matrix
"""
def euler2C(pitch,roll,yaw):
    C = np.zeros( (3,3) )
    cTheta = cos(pitch)
    sTheta = sin(pitch)
    cPhi = cos(roll)
    sPhi = sin(roll)
    cPsi = cos(yaw)
    sPsi = sin(yaw)
    C[0,0] = cTheta*cPsi
    C[0,1] = -1*cPhi*sPsi + sPhi*sTheta*cPsi
    C[0,2] = sPhi*sPsi + cPhi*sTheta*cPsi
    C[1,0] = cTheta*sPsi
    C[1,1] = cPhi*cPsi + sPhi*sTheta*sPsi
    C[1,2] = -sPhi*cPsi + cPhi*sTheta*sPsi
    C[2,0] = -sTheta
    C[2,1] = sPhi*cTheta
    C[2,2] = cPhi*cTheta
    return C

"""
"   c2euler
"   This routine converts a C matrix represention to euler representation
"   Input: C matrix
"   Output: pitch roll yaw
"""
def c2euler(C):
    pitch = -1*asin(C[2,0])
    yaw = atan2(C[1,0],C[0,0])
    roll = atan2(C[2,1],C[2,2])
    return (pitch,roll,yaw)

"""
"   ortho_norm
"   This routine ortho normalizes a C matrix
"   Input: C matrix
"   Output: Ortho normed C matrix
"""
def ortho_norm(C):
    x = C[:,0]
    y = C[:,1]
    z = C[:,2]
    
    e = x.transpose()
    e = e.dot(y)
    eScalar = e/2
    
    xOrtho = x - e*y
    yOrtho = y - e*x
    zOrtho = np.cross(xOrtho,yOrtho)
    
    xNorm = 0.5*(3 - xOrtho.transpose().dot(xOrtho))*xOrtho
    yNorm = 0.5*(3 - yOrtho.transpose().dot(yOrtho))*yOrtho
    zNorm = 0.5*(3 - zOrtho.transpose().dot(zOrtho))*zOrtho
    
    Con = np.column_stack( (xNorm,yNorm,zNorm) )
    return Con

"""
  Main Program
"""
# Initialization
sense = SenseHat()
sense.clear()

startTime = time.time()
prevUpdateTime = 0

# Warmup Period
# This allows some of the initial crazy samples to bleed out
print("Warming up ...")
for i in range(1,101):
  gyroRaw = sense.get_gyroscope_raw()
  accelRaw = sense.get_accelerometer_raw()
  magRaw = sense.get_compass_raw()
  temp = sense.get_temperature()
  
#
#   Alignment Period
#   Collect a set of data while the unit is motionless to establish
#   initial level and initial gyro bias
alignAccel = np.array([0,0,0])
alignGyro = np.array([0,0,0])
alignMag = np.array([0,0,0])

print("Aligning ...")
alignSamples = 100
for i in range(1,alignSamples+1):
    elapsedTime = time.time() - startTime  
    gyroRaw = sense.get_gyroscope_raw()
    accelRaw = sense.get_accelerometer_raw()
    magRaw = sense.get_compass_raw()
    temp = sense.get_temperature()
    magRaw["x"] = xSF *(magRaw["x"] - xBias)
    magRaw["y"] = ySF *(magRaw["y"] - yBias)

    logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime,**gyroRaw)) # rad/sec
    logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime,**accelRaw)) # Gs
    logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime,**magRaw)) # microT
    logFile.write("7,{0},{1}\r\n".format(elapsedTime,temp))
#     print("0,{0},{x},{y},{z}".format(elapsedTime,**gyroRaw)) # rad/sec
#     print("1,{0},{x},{y},{z}".format(elapsedTime,**accelRaw)) # Gs
#     print("2,{0},{x},{y},{z}".format(elapsedTime,**magRaw))

    alignAccel = alignAccel + \
               1/alignSamples*np.array([accelRaw["x"], accelRaw["y"],accelRaw["z"]])
    alignGyro = alignGyro + \
               1/alignSamples*np.array([gyroRaw["x"], gyroRaw["y"],gyroRaw["z"]])
    alignMag = alignMag + \
               1/alignSamples*np.array([magRaw["x"], magRaw["y"],magRaw["z"]])

# Establish initial prich roll from averaged accel measures
alignPitch = asin(-1*alignAccel[0])
alignRoll = atan2(alignAccel[1],1.0)
C_AL = euler2C(alignPitch,alignRoll,0.0)

# Establish initial heading from horizontal mag measuremetns
magL = C_AL.dot(alignMag.transpose())
yaw = atan2(-1*magL[1],magL[0])

# print(degrees(alignPitch))
# print(degrees(alignRoll))
# print(degrees(yaw))

# Initialize Gyro, Accel, and Filtered to Level transitions
C_AL = euler2C(alignPitch,alignRoll,yaw)
C_GL = euler2C(alignPitch,alignRoll,yaw)
C_FL = euler2C(alignPitch,alignRoll,yaw)

# Initial Gyro bias is the average over sample period
gyroBias = alignGyro

# Print out the results
(p,r,y) = c2euler(C_AL)
print(degrees(p))
print(degrees(r))
print(degrees(y))

# input()
# AHRS Loop

r180 = radians(180)
nr180 = radians(-180)
r360 = radians(360)

deltaTheta = np.array([0, 0 , 0])
accumTheta = np.array([0, 0 , 0])
omega = np.array( [0, 0, 0] )
deltaC = np.ones( (3,3) )

sense.clear()
prevUpdateTime = time.time() - startTime
prevTime = time.time() - startTime

while True:
 
  ######################
  # Fast Loop
  ######################
  elapsedTime = time.time() - startTime
  gyroRaw = sense.get_gyroscope_raw()
  accelRaw = sense.get_accelerometer_raw()
  magRaw = sense.get_compass_raw()
  temp = sense.get_temperature()

  
  logFile.write("0,{0},{x},{y},{z}\r\n".format(elapsedTime,**gyroRaw)) # rad/sec
  logFile.write("1,{0},{x},{y},{z}\r\n".format(elapsedTime,**accelRaw)) # Gs
  logFile.write("2,{0},{x},{y},{z}\r\n".format(elapsedTime,**magRaw)) # microT
  logFile.write("7,{0},{1}\r\n".format(elapsedTime,temp))
  magRaw["x"] = xSF *(magRaw["x"] - xBias)
  magRaw["y"] = ySF *(magRaw["y"] - yBias)
 
  deltaTime = elapsedTime - prevTime
  
#   print("0,{0},{x},{y},{z}".format(elapsedTime,**gyroRaw)) # rad/sec
#   print("1,{0},{x},{y},{z}".format(elapsedTime,**accelRaw)) # Gs
#   print("2,{0},{x},{y},{z}".format(elapsedTime,**magRaw))

# Accumplate Rates
  omega = np.array( (gyroRaw["x"], gyroRaw["y"], gyroRaw["z"]) ) - gyroBias
  deltaTheta = omega*deltaTime # Basic Integration
  accumTheta = accumTheta + deltaTheta 
#   print("----------------------Dt {0} {1} {2}".format(degrees(accumTheta[0]),degrees(accumTheta[1]),degrees(accumTheta[2])))

# Form the deltaC matrix from the angular rotations
# Single taylor series with small angle assumption
  deltaC[0,1] = -1*deltaTheta[2]
  deltaC[0,2] = deltaTheta[1]
  deltaC[1,0] = deltaTheta[2]
  deltaC[1,2] = -1*deltaTheta[0]
  deltaC[2,0] = -1*deltaTheta[1]
  deltaC[2,1] = deltaTheta[0]

# Update the gyro and filter solution
  C_GL = C_GL.dot(deltaC)
  C_FL = C_FL.dot(deltaC)

# Determine the Accel Solution
  accelPitch = asin(-1*accelRaw["x"])
  accelRoll = atan2(accelRaw["y"],1.0)
  accelMag = np.array([magRaw["x"], magRaw["y"],magRaw["z"]])
  C_AL = euler2C(accelPitch,accelRoll,0.0)
#   print(Ca)
  magL = C_AL.dot(accelMag.transpose())
#   print(magL)
  yaw = atan2(-1*magL[1],magL[0])
  C_AL = euler2C(accelPitch,accelRoll,yaw)
#   print("Accel {0} {1} {2}".format(degrees(accelPitch),degrees(accelRoll),degrees(yaw)))
  
  # Update the filtered solution with the data from the
  # Accel based solution
  (pf,rf,yf) = c2euler(C_FL)
  pf = (1-tau)*pf + tau*accelPitch
  rf = (1-tau)*rf + tau*accelRoll
  ey = yf-yaw
  if ey > r180:
    ey = ey - r360
  elif ey < -r180:
    ey = ey + r360
  yf = (1-tau)*yf + tau*(yf - ey)
  if yf > r180:
      yf = yf-r360
  elif yf < -r180:
      yf = yf+r360
  C_FL = euler2C(pf,rf,yf)
#   print("Att --> {0} {1} {2}".format(degrees(pf),degrees(rf),degrees(yf)))

# #
#   selection = False
#   events = sense.stick.get_events()
#   for event in events:
#       # Skip releases
#       if event.action != "released":
#         if event.direction == "middle":
#           xMax = -100
#           yMax = -100
#           xMin = 100
#           yMin = 100
#           xScale = 1
#           yScale = 1
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
#
  prevTime = elapsedTime
  
  
  
  ###############################
  # Slow Loop
  ###############################
  if (elapsedTime - prevUpdateTime)>1.0:
#     print("0,{0},{x},{y},{z}".format(elapsedTime,**gyroRaw)) # rad/sec
#     print("1,{0},{x},{y},{z}".format(elapsedTime,**accelRaw)) # Gs
#     print("2,{0},{x},{y},{z}".format(elapsedTime,**magRaw))
#     print(Cg)
    C_GL = ortho_norm(C_GL)
#     Cf = ortho_norm(Cf)
#     print(Cg)
    (p,r,y) = c2euler(C_GL)
    (pf,rf,yf) = c2euler(C_FL)
    print("Gyro : {0} {1} {2}".format(degrees(p),degrees(r),degrees(y)))
    print("Accel: {0} {1} {2}".format(degrees(accelPitch),degrees(accelRoll),degrees(yaw)))
    print("Filt : {0} {1} {2}".format(degrees(pf),degrees(rf),degrees(yf)))

    print("")
    prevUpdateTime = elapsedTime 



