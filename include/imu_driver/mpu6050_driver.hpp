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

#ifndef IMU_DRIVER__MPU6050_DRIVER_HPP_
#define IMU_DRIVER__MPU6050_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <memory>
#include <string>
#include <iostream>
#include <vector>

class Mpu6050Driver : public rclcpp::Node
{
public:
  Mpu6050Driver(const std::string & node_name, const rclcpp::NodeOptions & options);
  
private:
  void initializeI2C();
  void onTimer();
  void updateCurrentGyroData();
  void updateCurrentAccelData();
  //void calcRollPitch();
  //geometry_msgs::msg::Quaternion calcQuaternion();
  void calcQuaternion();
  void calcAttitude();
  void imuDataPublish();
  float get2data(int fd, unsigned int reg);
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock::SharedPtr clock_;
  std::vector<float> gyro_;
  std::vector<float> accel_;
  // Node parameters
  int timer_period;
  std::string frame_id;
  double g;
  int FS_SEL;
  int AFS_SEL;
  int Acc_SF;
  int Gyro_SF;
  double acc_scale;
  double gyro_scale;
  int do_calibration;
  int flip_longitudial;
  float AccelOffset[3];
  float GyroOffset[3];
  // Roll, Pitch, Yaw, angles are in degrees
  double Roll;
  double Pitch;
  double Yaw;
  // roll, pitch, yaw, angles are in radians
  geometry_msgs::msg::Quaternion Quat;
  // for wiringPiI2C
  int fd_;
};


#endif  // IMU_DRIVER__MPU6050_DRIVER_HPP_
