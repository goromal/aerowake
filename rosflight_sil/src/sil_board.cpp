#include "rosflight_sil/sil_board.h"
#include <fstream>
#include <ros/ros.h>

//#include <iostream>

namespace rosflight_sil {

SIL_Board::SIL_Board() :
  rosflight_firmware::UDPBoard(), cur_NED_(0.0, 0.0, 0.0), cur_uvw_(0.0, 0.0, 0.0),
  cur_q_NED_UAV_(transforms::Quatd::Identity()), cur_pqr_(0.0, 0.0, 0.0),
  cur_imu_accel_(0.0, 0.0, 0.0), cur_imu_gyro_(0.0, 0.0, 0.0)
{ }

void SIL_Board::init_board(void) {}

void SIL_Board::set_truth(const rosflight_sil::ROSflightSimState &msg)
{
  cur_NED_(0) = msg.pos.x;
  cur_NED_(1) = msg.pos.y;
  cur_NED_(2) = msg.pos.z;
  cur_uvw_(0) = msg.vel.x;
  cur_uvw_(1) = msg.vel.y;
  cur_uvw_(2) = msg.vel.z;
  cur_pqr_(0) = msg.w.x;
  cur_pqr_(1) = msg.w.y;
  cur_pqr_(2) = msg.w.z;
  cur_q_NED_UAV_.setW(msg.att.w);
  cur_q_NED_UAV_.setX(msg.att.x);
  cur_q_NED_UAV_.setY(msg.att.y);
  cur_q_NED_UAV_.setZ(msg.att.z);
  cur_imu_accel_(0) = msg.imu_accel.x;
  cur_imu_accel_(1) = msg.imu_accel.y;
  cur_imu_accel_(2) = msg.imu_accel.z;
  cur_imu_gyro_(0) = msg.imu_gyro.x;
  cur_imu_gyro_(1) = msg.imu_gyro.y;
  cur_imu_gyro_(2) = msg.imu_gyro.z;
}

void SIL_Board::ros_setup(ros::NodeHandle* nh, ros::NodeHandle* nh_private, std::string mav_type)
{
  nh_ = nh;
  nh_private_ = nh_private;
  mav_type_ = mav_type;

  std::string bind_host = nh_private_->param<std::string>("bind_host", "localhost");
  int bind_port = nh_private_->param<int>("bind_port", 14525);
  std::string remote_host = nh_private_->param<std::string>("remote_host", bind_host);
  int remote_port = nh_private_->param<int>("remote_port", 14520);

  set_ports(bind_host, bind_port, remote_host, remote_port);

  // Get Sensor Parameters
  gyro_stdev_ = nh_private_->param<double>("gyro_stdev", 0.0); // 0.13);
  gyro_bias_range_ = nh_private_->param<double>("gyro_bias_range", 0.0); // 0.15);
  gyro_bias_walk_stdev_ = nh_private_->param<double>("gyro_bias_walk_stdev", 0.0); // 0.001);

  acc_stdev_ = nh_private_->param<double>("acc_stdev", 0.0); // 1.15);
  acc_bias_range_ = nh_private_->param<double>("acc_bias_range", 0.0); // 0.15);
  acc_bias_walk_stdev_ = nh_private_->param<double>("acc_bias_walk_stdev", 0.0); // 0.001);

  mag_stdev_ = nh_private_->param<double>("mag_stdev", 1.15);
  mag_bias_range_ = nh_private_->param<double>("mag_bias_range", 0.15);
  mag_bias_walk_stdev_ = nh_private_->param<double>("mag_bias_walk_stdev", 0.001);

  baro_stdev_ = nh_private_->param<double>("baro_stdev", 1.15);
  baro_bias_range_ = nh_private_->param<double>("baro_bias_range", 0.15);
  baro_bias_walk_stdev_ = nh_private_->param<double>("baro_bias_walk_stdev", 0.001);

  airspeed_stdev_ = nh_private_->param<double>("airspeed_stdev", 1.15);
  airspeed_bias_range_ = nh_private_->param<double>("airspeed_bias_range", 0.15);
  airspeed_bias_walk_stdev_ = nh_private_->param<double>("airspeed_bias_walk_stdev", 0.001);

  sonar_stdev_ = nh_private_->param<double>("sonar_stdev", 1.15);
  sonar_min_range_ = nh_private_->param<double>("sonar_min_range", 0.25);
  sonar_max_range_ = nh_private_->param<double>("sonar_max_range", 8.0);

  imu_update_rate_ = nh_private_->param<double>("imu_update_rate", 1000.0);
  imu_update_period_us_ = (uint64_t)(1e6/imu_update_rate_);

  // Calculate Magnetic Field Vector (for mag simulation)
  double inclination = nh_private_->param<double>("inclination", 1.14316156541);
  double declination = nh_private_->param<double>("declination", 0.198584539676);
  inertial_magnetic_field_(0) = sin(inclination);
  inertial_magnetic_field_(1) = cos(inclination)*cos(declination);
  inertial_magnetic_field_(2) = cos(inclination)*sin(declination);

  // Get the desired altitude at the ground (for baro simulation)
  ground_altitude_ = nh_private_->param<double>("ground_altitude", 1387.0);

  // Configure Noise
  random_generator_= std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());
  normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
  uniform_distribution_ = std::uniform_real_distribution<double>(-1.0, 1.0);

//  gravity_ = GZ_COMPAT_GET_GRAVITY(world_);
  gravity_(0) = 0.0; // nh_private_->param<double>("gravity_N", 0.0);
  gravity_(1) = 0.0; // nh_private_->param<double>("gravity_E", 0.0);
  gravity_(2) = nh_->param<double>("gravity", 9.80665);

  // Initialize the Sensor Biases
  gyro_bias_(0) = gyro_bias_range_*uniform_distribution_(random_generator_);
  gyro_bias_(1) = gyro_bias_range_*uniform_distribution_(random_generator_);
  gyro_bias_(2) = gyro_bias_range_*uniform_distribution_(random_generator_);
  acc_bias_(0) = acc_bias_range_*uniform_distribution_(random_generator_);
  acc_bias_(1) = acc_bias_range_*uniform_distribution_(random_generator_);
  acc_bias_(2) = acc_bias_range_*uniform_distribution_(random_generator_);
  mag_bias_(0) = mag_bias_range_*uniform_distribution_(random_generator_);
  mag_bias_(1) = mag_bias_range_*uniform_distribution_(random_generator_);
  mag_bias_(2) = mag_bias_range_*uniform_distribution_(random_generator_);
  baro_bias_ = baro_bias_range_*uniform_distribution_(random_generator_);
  airspeed_bias_ = airspeed_bias_range_*uniform_distribution_(random_generator_);

  next_imu_update_time_us_ = 0;
}

void SIL_Board::board_reset(bool bootloader) {}

// clock

uint32_t SIL_Board::clock_millis() // THESE CLOCKS NEED TO GIVE TIME IN REFERENCE TO BOOT=ZERO TO ACTUALLY STREAM
{
    ros::Time now = ros::Time::now();
    double secs = now.toNSec() / 1.0e9;
  uint32_t millis = static_cast<uint32_t>(secs * 1e3);
  return millis;
}

uint64_t SIL_Board::clock_micros()
{
    ros::Time now = ros::Time::now();
    double secs = now.toNSec() / 1.0e9;
  uint32_t micros = static_cast<uint32_t>(secs * 1.0e6);
  return micros;
}

void SIL_Board::clock_delay(uint32_t milliseconds)
{
}

// sensors
/// TODO these sensors have noise, no bias
/// noise params are hard coded
void SIL_Board::sensors_init()
{
  // Initialize the Biases
  gyro_bias_(0) = gyro_bias_range_*uniform_distribution_(random_generator_);
  gyro_bias_(1) = gyro_bias_range_*uniform_distribution_(random_generator_);
  gyro_bias_(2) = gyro_bias_range_*uniform_distribution_(random_generator_);
  acc_bias_(0) = acc_bias_range_*uniform_distribution_(random_generator_);
  acc_bias_(1) = acc_bias_range_*uniform_distribution_(random_generator_);
  acc_bias_(2) = acc_bias_range_*uniform_distribution_(random_generator_);

  // Gazebo coordinates is NWU and Earth's magnetic field is defined in NED, hence the negative signs
  // ...well...not anymore.
//  double inclination = 1.14316156541;
//  double declination = 0.198584539676;
//  inertial_magnetic_field_(0) = sin(inclination);
//  inertial_magnetic_field_(1) = cos(inclination)*cos(declination);
//  inertial_magnetic_field_(2) = cos(inclination)*sin(declination);
  double inclination = nh_private_->param<double>("inclination", 1.14316156541);
  double declination = nh_private_->param<double>("declination", 0.198584539676);
  inertial_magnetic_field_(0) = sin(inclination);
  inertial_magnetic_field_(1) = cos(inclination)*cos(declination);
  inertial_magnetic_field_(2) = cos(inclination)*sin(declination);
}

uint16_t SIL_Board::num_sensor_errors(void)
{
  return 0;
}

bool SIL_Board::new_imu_data()
{
  uint64_t now_us = clock_micros();
  if (now_us >= next_imu_update_time_us_)
  {
    next_imu_update_time_us_ = now_us + imu_update_period_us_;
    return true;
  }
  else
  {
    return false;
  }
}

bool SIL_Board::imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us)
{
    // ACCELS MEASUREMENT
  Vector3d y_acc = cur_imu_accel_;

  // Apply normal noise (only if armed, because most of the noise comes from motors
  if (motors_spinning())
  {
    y_acc(0) += acc_stdev_*normal_distribution_(random_generator_);
    y_acc(1) += acc_stdev_*normal_distribution_(random_generator_);
    y_acc(2) += acc_stdev_*normal_distribution_(random_generator_);
  }

  // Perform Random Walk for biases
  acc_bias_(0) += acc_bias_walk_stdev_*normal_distribution_(random_generator_);
  acc_bias_(1) += acc_bias_walk_stdev_*normal_distribution_(random_generator_);
  acc_bias_(2) += acc_bias_walk_stdev_*normal_distribution_(random_generator_);

  // Add constant Bias to measurement
  y_acc += acc_bias_;

  // Output accels
  accel[0] = static_cast<float>(y_acc(0));
  accel[1] = static_cast<float>(y_acc(1));
  accel[2] = static_cast<float>(y_acc(2));

  if(isnan(accel[0]))
      std::cout << "NAN WARNING IN SIL_BOARD IMU SENSOR" << std::endl;

  (*temperature) = 27.0;

  // GYROS MEASUREMENT
  Vector3d y_gyro = cur_imu_gyro_;

  // Normal Noise from motors
  if (motors_spinning())
  {
    y_gyro(0) += gyro_stdev_*normal_distribution_(random_generator_);
    y_gyro(1) += gyro_stdev_*normal_distribution_(random_generator_);
    y_gyro(2) += gyro_stdev_*normal_distribution_(random_generator_);
  }

  // Random Walk for bias
  gyro_bias_(0) += gyro_bias_walk_stdev_*normal_distribution_(random_generator_);
  gyro_bias_(1) += gyro_bias_walk_stdev_*normal_distribution_(random_generator_);
  gyro_bias_(2) += gyro_bias_walk_stdev_*normal_distribution_(random_generator_);

  // Apply Constant Bias
  y_gyro += gyro_bias_;

  // Output gyros
  gyro[0] = static_cast<float>(y_gyro(0));
  gyro[1] = static_cast<float>(y_gyro(1));
  gyro[2] = static_cast<float>(y_gyro(2));

  (*time_us) = clock_micros();

  return true;
}

void SIL_Board::imu_not_responding_error(void)
{
  ROS_ERROR("[gazebo_rosflight_sil] imu not responding");
}

void SIL_Board::mag_read(float mag[3])
{
  Vector3d noise(mag_stdev_*normal_distribution_(random_generator_),
                 mag_stdev_*normal_distribution_(random_generator_),
                 mag_stdev_*normal_distribution_(random_generator_));

  // Random Walk for bias
  mag_bias_(0) += mag_bias_walk_stdev_*normal_distribution_(random_generator_);
  mag_bias_(1) += mag_bias_walk_stdev_*normal_distribution_(random_generator_);
  mag_bias_(2) += mag_bias_walk_stdev_*normal_distribution_(random_generator_);

  // combine parts to create a measurement
  Vector3d y_mag = cur_q_NED_UAV_.rotp(inertial_magnetic_field_) + mag_bias_ + noise;

  // Convert measurement to NED...or not...
  mag[0] = y_mag(0);
  mag[1] = y_mag(1);
  mag[2] = y_mag(2);
}

bool SIL_Board::mag_present(void)
{
  return true;
}

bool SIL_Board::baro_present()
{
  return true;
}

void SIL_Board::baro_read(float *pressure, float *temperature)
{
  // Invert measurement model for pressure and temperature
  double alt = -cur_NED_(2) + ground_altitude_;

  // Convert to the true pressure reading
  double y_baro = 101325.0f*(float)pow((1.0 - 2.25694e-5 * alt), 5.2553);

  // Add noise
  y_baro += baro_stdev_*normal_distribution_(random_generator_);

  // Perform random walk
  baro_bias_ += baro_bias_walk_stdev_*normal_distribution_(random_generator_);

  // Add random walk
  y_baro += baro_bias_;

//  std::cout << y_baro << std::endl;

  (*pressure) = (float)y_baro;
  (*temperature) = 27.0f;
}

bool SIL_Board::diff_pressure_present(void)
{
  if(mav_type_ == "fixedwing")
    return true;
  else
    return false;
}

void SIL_Board::diff_pressure_read(float *diff_pressure, float *temperature)
{
  static double rho_ = 1.225;
  // Calculate Airspeed

  double Va = cur_uvw_.norm();

  // Invert Airpseed to get sensor measurement
  double y_as = rho_*Va*Va/2.0; // Page 130 in the UAV Book

  // Add noise
  y_as += airspeed_stdev_*normal_distribution_(random_generator_);
  airspeed_bias_ += airspeed_bias_walk_stdev_*normal_distribution_(random_generator_);
  y_as += airspeed_bias_;

  *diff_pressure = y_as;
  *temperature = 27.0;
}

bool SIL_Board::sonar_present(void)
{
  return true;
}

float SIL_Board::sonar_read(void)
{
  double alt = -cur_NED_(2);

  if (alt < sonar_min_range_)
  {
    return sonar_min_range_;
  }
  else if (alt > sonar_max_range_)
  {
    return sonar_max_range_;
  }
  else
    return alt + sonar_stdev_*normal_distribution_(random_generator_);
}

// PWM
void SIL_Board::pwm_init(uint32_t refresh_rate, uint16_t idle_pwm)
{
  rc_received_ = false;
  latestRC_.values[0] = 1500; // x
  latestRC_.values[1] = 1500; // y
  latestRC_.values[3] = 1500; // z
  latestRC_.values[2] = 1000; // F
  latestRC_.values[4] = 1000; // attitude override
  latestRC_.values[5] = 1000; // arm

  for (size_t i = 0; i < 14; i++)
    pwm_outputs_[i] = 1000;

  rc_sub_ = nh_->subscribe("RC", 1, &SIL_Board::RCCallback, this);
}

float SIL_Board::rc_read(uint8_t channel)
{
  if(rc_sub_.getNumPublishers() > 0)
  {
    return static_cast<float>(latestRC_.values[channel]-1000)/1000.0;
  }

  //no publishers, set throttle low and center everything else
  if(channel == 2)
    return 0.0;

  return 0.5;
}

void SIL_Board::pwm_write(uint8_t channel, float value)
{
  pwm_outputs_[channel] = 1000+(uint16_t)(1000*value);
}
void SIL_Board::pwm_disable()
{
  for(int i=0;i<14;i++)
      pwm_write(i,0);
}

bool SIL_Board::rc_lost(void)
{
  return !rc_received_;
}

void SIL_Board::rc_init(rc_type_t rc_type) {}

// non-volatile memory
void SIL_Board::memory_init(void) {}

bool SIL_Board::memory_read(void * dest, size_t len)
{
  std::string directory = "rosflight_memory" + nh_->getNamespace();
  std::ifstream memory_file;
  memory_file.open(directory + "/mem.bin", std::ios::binary);

  if(!memory_file.is_open())
  {
    ROS_ERROR("Unable to load rosflight memory file %s/mem.bin", directory.c_str());
    return false;
  }

  memory_file.read((char*) dest, len);
  memory_file.close();
  return true;
}

bool SIL_Board::memory_write(const void * src, size_t len)
{
  std::string directory = "rosflight_memory" + nh_->getNamespace();
  std::string mkdir_command = "mkdir -p " + directory;
  const int dir_err = system(mkdir_command.c_str());

  if (dir_err == -1)
  {
    ROS_ERROR("Unable to write rosflight memory file %s/mem.bin", directory.c_str());
    return false;
  }

  std::ofstream memory_file;
  memory_file.open(directory + "/mem.bin", std::ios::binary);
  memory_file.write((char*) src, len);
  memory_file.close();
  return true;
}

bool SIL_Board::motors_spinning()
{
  if(pwm_outputs_[2] > 1100)
      return true;
  else
    return false;
}

// LED

void SIL_Board::led0_on(void) { }
void SIL_Board::led0_off(void) { }
void SIL_Board::led0_toggle(void) { }

void SIL_Board::led1_on(void) { }
void SIL_Board::led1_off(void) { }
void SIL_Board::led1_toggle(void) { }

bool SIL_Board::has_backup_data(void)
{
    return false;
}

rosflight_firmware::BackupData SIL_Board::get_backup_data(void)
{
    rosflight_firmware::BackupData blank_data = {0};
    return blank_data;
}

void SIL_Board::RCCallback(const rosflight_msgs::RCRaw& msg)
{
  rc_received_ = true;
  latestRC_ = msg;
}


bool SIL_Board::gnss_present() { return false; }
void SIL_Board::gnss_update() {}

rosflight_firmware::GNSSData SIL_Board::gnss_read()
{
    rosflight_firmware::GNSSData out;
    return out;
}

bool SIL_Board::gnss_has_new_data()
{
    return false;
}
rosflight_firmware::GNSSRaw SIL_Board::gnss_raw_read()
{
    rosflight_firmware::GNSSRaw out;
    return out;
}

} // end namespace rosflight_sil
