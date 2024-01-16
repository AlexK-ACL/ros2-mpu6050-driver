 
import smbus
import time
 
PWR_M   = 0x6B
DIV   = 0x19 # Register 25 – Sample Rate Divider SMPRT_DIV
# Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
CONFIG       = 0x1A # This register configures the external Frame Synchronization (FSYNC) EXT_SYNC_SET pin sampling 
# and the Digital Low Pass Filter (DLPF) DLPF_CFG setting for both the gyroscopes and accelerometers.
GYRO_CONFIG  = 0x1B # Register 27 – Gyroscope Configuration - used to trigger gyroscope self-test and configure the gyroscopes’ full scale range
ACCEL_CONFIG = 0x1C # Register 28 – Accelerometer Configuration
INT_EN   = 0x38
ACCEL_X = 0x3B
ACCEL_Y = 0x3D
ACCEL_Z = 0x3F
GYRO_X  = 0x43
GYRO_Y  = 0x45
GYRO_Z  = 0x47
TEMP = 0x41
bus = smbus.SMBus(1)
Device_Address = 0x68   # device address

# Initial calibration offsets 
AxCal=0
AyCal=0
AzCal=0
GxCal=0
GyCal=0
GzCal=0
 
 
def InitMPU():
  bus.write_byte_data(Device_Address, DIV, 7)
  bus.write_byte_data(Device_Address, PWR_M, 1)
  bus.write_byte_data(Device_Address, CONFIG, 0)
  bus.write_byte_data(Device_Address, GYRO_CONFIG, 24) # 24 bits 3-4 -> bin 11 FS_SEL=3 range +-2000 ; bin 10 ->2 range +-1000 ; bin 01 ->1 range +-500 ; bin 00 ->0  range +-250
  bus.write_byte_data(Device_Address, ACCEL_CONFIG, 0) # Same as for gyro: 3 +-16g ; 2 +-8g ; 1 +-4g ; 0 +- 2g
  bus.write_byte_data(Device_Address, INT_EN, 1)
  time.sleep(1)
 
def display(x,y,z):
  x=x*100
  y=y*100
  z=z*100
  x= "%d " %x
  y= "%d " %y
  z= "%d" %z

  print(x,y,z)
 
 
def readMPU(addr):
  high = bus.read_byte_data(Device_Address, addr)
  low = bus.read_byte_data(Device_Address, addr+1)
  value = ((high << 8) | low)
  if(value > 32768):
    value = value - 65536
  return value
def accel():
  x = readMPU(ACCEL_X)
  y = readMPU(ACCEL_Y)
  z = readMPU(ACCEL_Z)
  
  Ax = (x/16384.0-AxCal) 
  Ay = (y/16384.0-AyCal) 
  Az = (z/16384.0-AzCal)
  
  #print "X="+str(Ax)
  display(Ax,Ay,Az)
  time.sleep(.01)
 
def gyro():
  global GxCal
  global GyCal
  global GzCal
  x = readMPU(GYRO_X)
  y = readMPU(GYRO_Y)
  z = readMPU(GYRO_Z)
  Gx = x/131.0 - GxCal
  Gy = y/131.0 - GyCal
  Gz = z/131.0 - GzCal
  #print "X="+str(Gx)
  display(Gx,Gy,Gz)
  time.sleep(.01)
 
def temp():
  tempRow=readMPU(TEMP)
  tempC=(tempRow / 340.0) + 36.53
  tempC="Temp: %.2f" %tempC
  print(tempC)
  time.sleep(.2)
 
def calibrate():
  print("Calibrate....")
  global AxCal
  global AyCal
  global AzCal
  x=0
  y=0
  z=0
  for i in range(50):
    x = x + readMPU(ACCEL_X)
    y = y + readMPU(ACCEL_Y)
    z = z + readMPU(ACCEL_Z)
  x= x/50
  y= y/50
  z= z/50
  AxCal = x/16384.0
  AyCal = y/16384.0
  AzCal = z/16384.0
  
  print(AxCal, AyCal, AzCal)
 
  global GxCal
  global GyCal
  global GzCal
  x=0
  y=0
  z=0
  for i in range(50):
    x = x + readMPU(GYRO_X)
    y = y + readMPU(GYRO_Y)
    z = z + readMPU(GYRO_Z)
  x= x/50
  y= y/50
  z= z/50
  GxCal = x/131.0
  GyCal = y/131.0
  GzCal = z/131.0
 
  print(GxCal, GyCal, GzCal)
 
# --------------------------- Starting point 
print("MPU6050 Interface")
print("Circuit Digest")
time.sleep(2)
InitMPU()
calibrate()
while 1:
  InitMPU()
  for i in range(20):
    temp()
  print("Accel")
  time.sleep(1)
  for i in range(30):
    accel()
  print("Gyro")
  time.sleep(1)
  for i in range(30):
    gyro()
