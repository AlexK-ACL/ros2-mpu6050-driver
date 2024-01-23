// Copyright 2021 Takeshi Miura
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "imu_driver/mpu6050_driver.hpp"
#include <rclcpp/timer.hpp>
#include <memory>
#include <string>
#include <cmath>
#include <utility>
#include <wiringPi.h> //"wiringPi/wiringPi.h" // https://github.com/WiringPi/WiringPi
#include <wiringPiI2C.h> //"wiringPi/wiringPiI2C.h"
//#include <i2c/smbus.h>
//#include <linux/i2c-dev.h>
//#include <i2c/smbus.h>

#define SMPLRT_DIV    0x19 // Register 25 – Sample Rate Divider SMPRT_DIV; Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
#define CONFIG       0x1A // Register 26 – Configuration - This register configures the external Frame Synchronization (FSYNC) EXT_SYNC_SET pin sampling and the Digital Low Pass Filter (DLPF) DLPF_CFG setting for both the gyroscopes and accelerometers.
#define PWR_MGMT_1   0x6B  //PWR_MGMT_1
#define PWR_MGMT_2   0x6C  //PWR_MGMT_2
#define GYRO_CONFIG  0x1B // Register 27 – Gyroscope Configuration - used to trigger gyroscope self-test and configure the gyroscopes’ full scale range
// 24 bits 3-4 -> bin 11 FS_SEL=3 range +-2000 ; bin 10 ->2 range +-1000 ; bin 01 ->1 range +-500 ; bin 00 ->0  range +-250
#define ACCEL_CONFIG 0x1C // Register 28 – Accelerometer Configuration
// Same as for gyro: 3 +-16g ; 2 +-8g ; 1 +-4g ; 0 +- 2g
#define INT_ENABLE   0x38

#define TEMP_OUT     0x41

#define ACCEL_X_OUT  0x3b
#define ACCEL_Y_OUT  0x3d
#define ACCEL_Z_OUT  0x3f

#define GYRO_X_OUT   0x43 
#define GYRO_Y_OUT   0x45
#define GYRO_Z_OUT   0x47

#define DEV_ADDR     0x68    // MPU6050 device I2C address 

#define N_CALIB_DEF 500 // Calibration set size

/* struct Quaternion
{
    double w, x, y, z;
}; */

Mpu6050Driver::Mpu6050Driver(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  // creates a parameter with the name timer_period and a default value of 100.
  this->declare_parameter("timer_period", 100); // The parameter type is inferred from the default value
  this->declare_parameter("frame_id", "imu");
  this->declare_parameter("g", 9.81);
  this->declare_parameter("AFS_SEL", 0);
  this->declare_parameter("FS_SEL", 0);
  this->declare_parameter("do_calibration", 0);

  // Get node update timer period
  timer_period = this->get_parameter("timer_period").as_int();
  // Get output messsages frame_id
  frame_id = this->get_parameter("frame_id").as_string();
  // Get g const
  g = this->get_parameter("g").as_double();

  // Get measurements scaling constants
  AFS_SEL = this->get_parameter("AFS_SEL").as_int(); // Accel scale setting 0 1 2 3
  FS_SEL = this->get_parameter("FS_SEL").as_int(); // Gyro scale setting 0 1 2 3

  // Get calibration setting
  do_calibration = this->get_parameter("do_calibration").as_int();

  RCLCPP_INFO(this->get_logger(), "Received parameters: timer_period=%d; g=%f; AFS_SEL=%d; FS_SEL=%d; do_calibration=%d", timer_period, g, AFS_SEL, FS_SEL, do_calibration);

  // Measurements scaling setup
  Acc_SF = pow(2, AFS_SEL) * 2;
  Gyro_SF = pow(2, FS_SEL) * 250;
  acc_scale = (double)Acc_SF / 32768.0;
  gyro_scale = (double)Gyro_SF / 32768.0;
  RCLCPP_INFO(this->get_logger(), "Ranges are set to +-%d g; +-%d deg/s", Acc_SF, Gyro_SF);
  // M_PI/180
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("output", rclcpp::QoS{10});

  auto on_timer_ = std::bind(&Mpu6050Driver::onTimer, this);
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_)>>(
    this->get_clock(), timer_period*1ms, std::move(on_timer_), this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);

  initializeI2C();
  // reset offsets
  AccelOffset[0] = 0;
  AccelOffset[1] = 0;
  AccelOffset[2] = 0;
  GyroOffset[0] = 0;
  GyroOffset[1] = 0;
  GyroOffset[2] = 0;
  
  // Calibration
  
  if (do_calibration >= 1){
    int N_Calibrate;
    if (do_calibration < 100){ // You can set calibration duration starting from 100, otherwise uses default
      N_Calibrate = N_CALIB_DEF;
    }
    else {
      N_Calibrate = do_calibration;
    }
    //
    RCLCPP_INFO(this->get_logger(), "Starting calibration process");
    //
    double offsets[6];
    //long offsetsOld[6];
    float mpuGet[6];
    for (uint8_t j = 0; j < 6; j++) {    // reset calibration array
        offsets[j] = 0;
    }
    for (int i = 0; i < N_Calibrate; i++) {
      // Measuring N_Calibrate times to get mean values
      mpuGet[0] = get2data(fd_, ACCEL_X_OUT);
      mpuGet[1] = get2data(fd_, ACCEL_Y_OUT);
      mpuGet[2] = get2data(fd_, ACCEL_Z_OUT);
      mpuGet[3] = get2data(fd_, GYRO_X_OUT);
      mpuGet[4] = get2data(fd_, GYRO_Y_OUT);
      mpuGet[5] = get2data(fd_, GYRO_Z_OUT);
      for (uint8_t j = 0; j < 6; j++) {
          offsets[j] += (double)mpuGet[j];    // accumulating raw measurements
      }
      delay(10);
    }
    for (uint8_t i = 0; i < 6; i++) {
        offsets[i] = (offsets[i] / N_Calibrate); // meaning values
        //if (i == 2) offsets[i] -= 32768.0/Acc_SF;  
    }
    
    // setting new offsets
    AccelOffset[0] = (float)(offsets[0] * acc_scale);
    AccelOffset[1] = (float)(offsets[1] * acc_scale);
    AccelOffset[2] = (float)(offsets[2] * acc_scale - 1); // Z axis accel should measure 1g, so tying it to that
    GyroOffset[0] = (float)(offsets[3] * gyro_scale);
    GyroOffset[1] = (float)(offsets[4] * gyro_scale);
    GyroOffset[2] = (float)(offsets[5] * gyro_scale);

    RCLCPP_INFO(this->get_logger(), "Calibration ended. Offsets are:");
    RCLCPP_INFO(this->get_logger(), "ax=%.2f; ay=%.2f; az=%.2f; gx=%.2f; gy=%.2f; gz=%.2f",AccelOffset[0],AccelOffset[1],AccelOffset[2],GyroOffset[0],GyroOffset[1],GyroOffset[2]);
    //
  }
  // Initialize quaternion
  Quat.x = 0;
  Quat.y = 0;
  Quat.z = 0;
  Quat.w = 0;
  //
  RCLCPP_INFO(this->get_logger(), "Starting measurements");
}

void Mpu6050Driver::initializeI2C(){
  fd_ = wiringPiI2CSetup(DEV_ADDR);
  if (fd_ == -1){
    RCLCPP_ERROR(this->get_logger(),"ERROR : No device!!");
    //printf("ERROR : No device!!");
  }
  else { // Set MPU6050 configuration registers
    wiringPiI2CWriteReg8(fd_, SMPLRT_DIV, 7);
    wiringPiI2CWriteReg8(fd_, PWR_MGMT_1, 1);
    wiringPiI2CWriteReg8(fd_, CONFIG, 0);
    wiringPiI2CWriteReg8(fd_, GYRO_CONFIG, FS_SEL << 3); // Shift 3 bits to the left to set Bit3 and Bit4
    wiringPiI2CWriteReg8(fd_, ACCEL_CONFIG, AFS_SEL << 3); // Shift 3 bits to the left to set Bit3 and Bit4
    RCLCPP_INFO(this->get_logger(), "Configured MPU6050 registers");
  }
}

void Mpu6050Driver::onTimer()
{
  updateCurrentGyroData();
  updateCurrentAccelData();
  //calcRollPitch(); 
  imuDataPublish();
}

void Mpu6050Driver::updateCurrentGyroData()
{
    gyro_.push_back((get2data(fd_, GYRO_X_OUT) * gyro_scale) - GyroOffset[0]);
    gyro_.push_back((get2data(fd_, GYRO_Y_OUT) * gyro_scale) - GyroOffset[1]);
    gyro_.push_back((get2data(fd_, GYRO_Z_OUT) * gyro_scale) - GyroOffset[2]);
}

void Mpu6050Driver::updateCurrentAccelData()
{
    accel_.push_back((get2data(fd_, ACCEL_X_OUT) * acc_scale) - AccelOffset[0]);
    accel_.push_back((get2data(fd_, ACCEL_Y_OUT) * acc_scale) - AccelOffset[1]);
    accel_.push_back((get2data(fd_, ACCEL_Z_OUT) * acc_scale) - AccelOffset[2]);
}

float Mpu6050Driver::get2data(int fd_, unsigned int reg){
  unsigned int h_value = wiringPiI2CReadReg8(fd_, reg);
  unsigned int l_value = wiringPiI2CReadReg8(fd_, reg+1);
  float value = (h_value << 8) + l_value;
  if (value>=32768) return (value - 65534);  //32768=0x8000, 65534 = 0xFFFF 
  else return value;
}

void Mpu6050Driver::imuDataPublish(){
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id;
  // As stated in https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
  msg.angular_velocity.x = gyro_[0] * (M_PI/180.0); // in rad/s
  msg.angular_velocity.y = gyro_[1] * (M_PI/180.0);
  msg.angular_velocity.z = gyro_[2] * (M_PI/180.0);
  msg.linear_acceleration.x = accel_[0] * g; // in m/s^2
  msg.linear_acceleration.y = accel_[1] * g;
  msg.linear_acceleration.z = accel_[2] * g;
  // Quaternion pose, calculated from integrated Euler angles
  msg.orientation = Quat;
  /* msg.orientation.y = Quat.y;
  msg.orientation.z = Quat.z;
  msg.orientation.w = Quat.w; */
  imu_pub_->publish(msg);
  gyro_.clear();
  accel_.clear();
}

void Mpu6050Driver::calcRollPitch()
{
  //float roll = std::atan(accel_[1]/accel_[2])*57.324;
  //float pitch = std::atan(-accel_[0]/std::sqrt(accel_[1]*accel_[1] + accel_[2]*accel_[2]))*57.324;
  //printf("roll=%.2f ; pitch=%.2f", roll, pitch);
}

geometry_msgs::msg::Quaternion Mpu6050Driver::calcQuaternion(double roll, double pitch, double yaw)
{
    // Abbreviations for the various angular functions

    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}