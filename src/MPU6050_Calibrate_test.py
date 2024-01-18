'''
    Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
  http://www.electronicwings.com
'''
import smbus          # import SMBus module of I2C
from time import sleep, time          # import
from math import pi
# Some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
TEMP_OUT = 0x41

bus = smbus.SMBus(1)   # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

N_Calibrate = 500
Refresh_Period = 0.005 # in sec

# Measurements scaling setup
g = 9.81
AFS_SEL = 0 # Accel scale setting 0 1 2 3
FS_SEL = 0 # Gyro scale setting 0 1 2 3
Acc_SF = 2**AFS_SEL*2
Gyro_SF = 2**FS_SEL*250

print ("Acc_SF=+-",Acc_SF,"g ; Gyro_SF=+-", Gyro_SF, "deg/s")

acc_scale = Acc_SF/32768.0
gyro_scale = Gyro_SF/32768.0

# Initial calibration offsets 
AxCal=0
AyCal=0
AzCal=0
GxCal=0
GyCal=0
GzCal=0

def MPU_Init():
  #write to sample rate register
    #Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    #The accelerometer output rate is 1kHz. This means that for a Sample Rate greater than 1kHz,
  #the same accelerometer sample may be output to the FIFO, DMP, and sensor registers more than once.
  bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
  
  #Write to power management register
    #Upon power up, the MPU-60X0 clock source defaults to the internal oscillator. However, it is highly
  #recommended that the device be configured to use one of the gyroscopes (or an external clock
  #source) as the clock reference for improved stability.
  bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
  
  #Write to Configuration register
    #Ext sync input disabled and DLPF (Digital Low Pass Filter) is default
  bus.write_byte_data(Device_Address, CONFIG, 0)
  
  #Write to Gyro configuration register
  bus.write_byte_data(Device_Address, GYRO_CONFIG, FS_SEL<<3) # Shift 3 bits to the left to set Bit3 and Bit4
  
  #Write to Accel configuration register
  bus.write_byte_data(Device_Address, ACCEL_CONFIG, AFS_SEL<<3) # Shift 3 bits to the left to set Bit3 and Bit4
  
  #Write to interrupt enable register
    #When set to 1, this bit enables the Data Ready interrupt
  bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
  #Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    
    #concatenate higher and lower value
    value = ((high << 8) | low)
        
    #to get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    return value

def calibrate_Acc(N):
  print("Calibrate Acc....")
  global AxCal
  global AyCal
  global AzCal
  x=0
  y=0
  z=0
  # Get the average of 50 measurements
  for i in range(N):
    x = x + read_raw_data(ACCEL_XOUT_H)
    y = y + read_raw_data(ACCEL_YOUT_H)
    z = z + read_raw_data(ACCEL_ZOUT_H)
    sleep(Refresh_Period)
  x= x/N
  y= y/N
  z= z/N

  AxCal = x * acc_scale
  AyCal = y * acc_scale
  AzCal = z * acc_scale - 1 # For vertical axis to be 1g
  print("Calibrate Acc Result....")
  print("AxCal, AyCal, AzCal - in g")
  print(AxCal, AyCal, AzCal)

def calibrate_Gyro(N):
  print("Calibrate Gyro....")
  global GxCal
  global GyCal
  global GzCal
  x=0
  y=0
  z=0
  # Get the average of N measurements
  for i in range(N):
    x = x + read_raw_data(GYRO_XOUT_H)
    y = y + read_raw_data(GYRO_YOUT_H)
    z = z + read_raw_data(GYRO_ZOUT_H)
    sleep(Refresh_Period)
  x= x/N
  y= y/N
  z= z/N
  GxCal = x * gyro_scale
  GyCal = y * gyro_scale
  GzCal = z * gyro_scale
  print("Calibrate Gyro Result....")
  print("GxCal, GyCal, GzCal - deg/sec")
  print(GxCal, GyCal, GzCal)

# -------------------------------------------------------------------------------------------- Script start


MPU_Init()

print("Calibrate Accel - Do not move....")
calibrate_Acc(N_Calibrate)
print("Calibrate Gyro - Do not move....")
calibrate_Gyro(N_Calibrate)
# Initialize
start_time = time()
time_cur = start_time
time_last = start_time
time_last_print = start_time

# Initial angles
dPhi_x = 0
dPhi_y = 0
dPhi_z = 0


print (" Reading Data of Gyroscope and Accelerometer")
print (" Outputs measurements")

while True:
  
  #Read Accelerometer raw value
  acc_x = read_raw_data(ACCEL_XOUT_H) #Read just Gyro for angle test
  acc_y = read_raw_data(ACCEL_YOUT_H)
  acc_z = read_raw_data(ACCEL_ZOUT_H)
  
  #Read Gyroscope raw value
  gyro_x = read_raw_data(GYRO_XOUT_H)
  gyro_y = read_raw_data(GYRO_YOUT_H)
  gyro_z = read_raw_data(GYRO_ZOUT_H)
  
  #Read Temperature raw value
  #temp = read_raw_data(TEMP_OUT)
  #Temp_C = ((temp)/340 + 36.53)
  
  #print ("Raw:")
  #print (" gyro_x=", acc_x, " gyro_y=", gyro_y, " gyro_z=", gyro_z, " acc_x=", acc_x, " acc_y=", acc_y, " acc_z=", acc_z, " temp=", temp)

  # Full scale range at AFS_SEL=0 +/- 2g as per sensitivity scale factor 
  Ax = acc_x * acc_scale - AxCal
  Ay = acc_y * acc_scale - AyCal
  Az = acc_z * acc_scale - AzCal
  # Full scale range +/- 250 degree/sec as per sensitivity scale factor
    # In degrees
  Gx = gyro_x * gyro_scale  - GxCal # longitudinal axis
  Gy = gyro_y * gyro_scale  - GyCal # lateral axis
  Gz = gyro_z * gyro_scale  - GzCal
    # Angle increments
  time_cur = time()
  #dPhi_x = dPhi_x + Gx*Refresh_Period
  #dPhi_y = dPhi_y + Gy*Refresh_Period
  #dPhi_z = dPhi_z + Gz*Refresh_Period
  #time_cur = time_cur + Refresh_Period
    
  Ax_ = Ax * g
  Ay_ = Ay * g
  Az_ = Az * g

  Gx_ = Gx * pi/180
  Gy_ = Gy * pi/180
  Gz_ = Gz * pi/180
    
  if (time_cur - time_last_print > 2):
    print("dPhi_x=%.2f" %dPhi_x, "dPhi_y=%.2f" %dPhi_y, "dPhi_z=%.2f" %dPhi_z, "- in degrees")
    print("time=%.2f" %time_cur)
    time_last_print = time_cur

    print ("In deg/s and g:")
    print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 

    print ("In rad/s and m/s^2:")
    print ("Gx=%.2f" %Gx_, "rad/s", "\tGy=%.2f" %Gy_, "rad/s", "\tGz=%.2f" %Gz_, "rad/s", "\tAx=%.2f m/s/s" %Ax_, "\tAy=%.2f m/s/s" %Ay_, "\tAz=%.2f m/s/s" %Az_)  
    
    #print ("Temperature=%.2f" %Temp_C, u'\u00b0' + "C")
    
  sleep(Refresh_Period)
