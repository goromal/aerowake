#pragma once

#include <cmath>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <chrono>

#include <ros/ros.h>
#include <rosflight_msgs/RCRaw.h>
#include "rosflight_sil/ROSflightSimState.h"

#include <rosflight_firmware/udp_board.h>

#include <eigen3/Eigen/Eigen>
#include <random>

#include "geometry-utils-lib/quat.h"

using namespace Eigen;

namespace rosflight_sil
{

class SIL_Board : public rosflight_firmware::UDPBoard
{
private:
  Vector3d inertial_magnetic_field_;

  double imu_update_rate_;

  double gyro_stdev_;
  double gyro_bias_walk_stdev_;
  double gyro_bias_range_;

  double acc_stdev_;
  double acc_bias_range_;
  double acc_bias_walk_stdev_;

  double baro_bias_walk_stdev_;
  double baro_stdev_;
  double baro_bias_range_;

  double mag_bias_walk_stdev_;
  double mag_stdev_;
  double mag_bias_range_;

  double airspeed_bias_walk_stdev_;
  double airspeed_stdev_;
  double airspeed_bias_range_;

  double sonar_stdev_;
  double sonar_max_range_;
  double sonar_min_range_;

  Vector3d gyro_bias_;
  Vector3d acc_bias_;
  Vector3d mag_bias_;
  double baro_bias_;
  double airspeed_bias_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_distribution_;
  std::uniform_real_distribution<double> uniform_distribution_;

  Vector3d gravity_;
  double ground_altitude_;

  ros::NodeHandle* nh_;
  ros::NodeHandle* nh_private_;
  ros::Subscriber rc_sub_;
  rosflight_msgs::RCRaw latestRC_;
  bool rc_received_;

  Vector3d cur_NED_;
  Vector3d cur_uvw_;
  transforms::Quatd cur_q_NED_UAV_;
  Vector3d cur_pqr_;

  Vector3d cur_imu_accel_;
  Vector3d cur_imu_gyro_;

  std::string mav_type_;
  int pwm_outputs_[14];  //assumes maximum of 14 channels

  // Time variables
  uint64_t next_imu_update_time_us_;
  uint64_t imu_update_period_us_;

  void RCCallback(const rosflight_msgs::RCRaw& msg);
  bool motors_spinning();

public:
  SIL_Board();

  // setup
  void init_board(void) override;
  void board_reset(bool bootloader) override;

  // truth setting
  void set_truth(const rosflight_sil::ROSflightSimState& msg);

  // clock
  uint32_t clock_millis() override;
  uint64_t clock_micros() override;
  void clock_delay(uint32_t milliseconds) override;

  // sensors
  void sensors_init() override;
  uint16_t num_sensor_errors(void) override;

  bool new_imu_data() override;
  bool imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us) override;
  void imu_not_responding_error() override;

  bool mag_present(void) override;
  void mag_read(float mag[3]) override;
  void mag_update(void) override {}

  bool baro_present(void) override;
  void baro_read(float *pressure, float *temperature) override;
  void baro_update(void) override {}

  bool diff_pressure_present(void) override;
  void diff_pressure_read(float *diff_pressure, float *temperature) override;
  void diff_pressure_update(void) override {}

  bool sonar_present(void) override;
  float sonar_read(void) override;
  void sonar_update(void) override {}

  // PWM
  // TODO make these deal in normalized (-1 to 1 or 0 to 1) values (not pwm-specific)
  void pwm_init(uint32_t refresh_rate, uint16_t idle_pwm) override;
  void pwm_write(uint8_t channel, float value) override;
  void pwm_disable(void) override;

  //RC
  float rc_read(uint8_t channel) override;
  void rc_init(rc_type_t rc_type) override;
  bool rc_lost(void) override;


  // non-volatile memory
  void memory_init(void) override;
  bool memory_read(void * dest, size_t len) override;
  bool memory_write(const void * src, size_t len) override;

  // LEDs
  void led0_on(void) override;
  void led0_off(void) override;
  void led0_toggle(void) override;

  void led1_on(void) override;
  void led1_off(void) override;
  void led1_toggle(void) override;

  //Backup Memory
  bool has_backup_data(void) override;
  rosflight_firmware::BackupData get_backup_data(void) override;

  bool gnss_present() override;
  void gnss_update() override;

  rosflight_firmware::GNSSData gnss_read() override;
  bool gnss_has_new_data() override;
  rosflight_firmware::GNSSRaw gnss_raw_read() override;

  // Gazebo stuff
  void ros_setup(ros::NodeHandle* nh, ros::NodeHandle* nh_private, std::string mav_type);
  inline int* get_outputs() { return pwm_outputs_; }
};

} // end namespace rosflight_sil
