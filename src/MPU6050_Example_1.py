'''
    Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
	http://www.electronicwings.com
'''
import smbus					# import SMBus module of I2C
from time import sleep          # import
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

Refresh_Period = 1
# Measurements scaling setup
g = 9.81
AFS_SEL = 0 # Accel scale setting 0 1 2 3
FS_SEL = 0 # Gyro scale setting 0 1 2 3
Acc_SF = 2**AFS_SEL*2
Gyro_SF = 2**FS_SEL*250

print ("Acc_SF=+-",Acc_SF,"g ; Gyro_SF=+-", Gyro_SF, "deg/s")

acc_scale = Acc_SF/32768.0
gyro_scale = Gyro_SF/32768.0

def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, FS_SEL)
	
	#Write to Accel configuration register
	bus.write_byte_data(Device_Address, ACCEL_CONFIG, AFS_SEL)
	
	#Write to interrupt enable register
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


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

print (" Reading Data of Gyroscope and Accelerometer")

while True:
	
	#Read Accelerometer raw value
	acc_x = read_raw_data(ACCEL_XOUT_H)
	acc_y = read_raw_data(ACCEL_YOUT_H)
	acc_z = read_raw_data(ACCEL_ZOUT_H)
	
	#Read Gyroscope raw value
	gyro_x = read_raw_data(GYRO_XOUT_H)
	gyro_y = read_raw_data(GYRO_YOUT_H)
	gyro_z = read_raw_data(GYRO_ZOUT_H)
	
	#Read Temperature raw value
	temp = read_raw_data(TEMP_OUT)
	Temp_C = ((temp)/340 + 36.53)
	
	print ("Raw:")
	print (" gyro_x=", acc_x, " gyro_y=", gyro_y, " gyro_z=", gyro_z, " acc_x=", acc_x, " acc_y=", acc_y, " acc_z=", acc_z, " temp=", temp)

	# Full scale range at AFS_SEL=0 +/- 2g as per sensitivity scale factor 
	Ax = acc_x * acc_scale
	Ay = acc_y * acc_scale
	Az = acc_z * acc_scale
	# Full scale range +/- 250 degree/C as per sensitivity scale factor
	Gx = gyro_x * gyro_scale
	Gy = gyro_y * gyro_scale
	Gz = gyro_z * gyro_scale
	
	print ("In deg/s and g:")
	print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 
	
	Ax = acc_x * acc_scale * g
	Ay = acc_y * acc_scale * g
	Az = acc_z * acc_scale * g

	Gx = gyro_x * gyro_scale * pi/180
	Gy = gyro_y * gyro_scale * pi/180
	Gz = gyro_z * gyro_scale * pi/180

	print ("In rad/s and m/s^2:")
	print ("Gx=%.2f" %Gx, "rad/s", "\tGy=%.2f" %Gy, "rad/s", "\tGz=%.2f" %Gz, "rad/s", "\tAx=%.2f m/s/s" %Ax, "\tAy=%.2f m/s/s" %Ay, "\tAz=%.2f m/s/s" %Az)	
    
	print ("Temperature=%.2f" %Temp_C, u'\u00b0' + "C")
	sleep(Refresh_Period)
