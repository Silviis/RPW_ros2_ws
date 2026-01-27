#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cmath>

class Icm20948Node : public rclcpp::Node
{
public:
  Icm20948Node()
  : Node("icm20948_node")
  {
    declare_parameter<std::string>("i2c_device", "/dev/i2c-8");
    declare_parameter<int>("i2c_address", 0x68);
    declare_parameter<int>("rate_hz", 200);

    get_parameter("i2c_device", i2c_device_);
    get_parameter("i2c_address", i2c_addr_);
    get_parameter("rate_hz", rate_hz_);

    openI2C();
    configureSensor();

    pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    timer_ = create_wall_timer(
      std::chrono::microseconds(1'000'000 / rate_hz_),
      std::bind(&Icm20948Node::readAndPublish, this)
    );

    RCLCPP_INFO(get_logger(), "ICM-20948 IMU running at %d Hz", rate_hz_);
  }

  ~Icm20948Node()
  {
    if (fd_ >= 0)
      close(fd_);
  }

private:
  /* ---------------- I2C helpers ---------------- */

  void openI2C()
  {
    fd_ = open(i2c_device_.c_str(), O_RDWR);
    if (fd_ < 0)
      throw std::runtime_error("Failed to open I2C device");

    if (ioctl(fd_, I2C_SLAVE, i2c_addr_) < 0)
      throw std::runtime_error("Failed to set I2C address");
  }

  void writeReg(uint8_t reg, uint8_t val)
  {
    uint8_t buf[2] = {reg, val};
    if (write(fd_, buf, 2) != 2)
      throw std::runtime_error("I2C write failed");
  }

  void readRegs(uint8_t reg, uint8_t *buf, size_t len)
  {
    if (write(fd_, &reg, 1) != 1)
      throw std::runtime_error("I2C reg select failed");
    if (read(fd_, buf, len) != (int)len)
      throw std::runtime_error("I2C read failed");
  }

  /* ---------------- Sensor config ---------------- */

  void selectBank(uint8_t bank)
  {
    writeReg(0x7F, bank << 4);
  }

  void configureSensor()
  {
    selectBank(0);
    writeReg(0x06, 0x01);  // wake up
    usleep(10000);

    selectBank(2);

    // Gyro ±2000 dps, DLPF off
    writeReg(0x01, 0x18);
    writeReg(0x02, 0x00);

    // Accel ±16g, DLPF off
    writeReg(0x14, 0x18);
    writeReg(0x15, 0x00);

    // Sample rate ~200 Hz
    writeReg(0x00, 4);   // gyro divider
    writeReg(0x11, 4);  // accel divider

    selectBank(0);
  }

  /* ---------------- Read & publish ---------------- */

  void readAndPublish()
  {
    uint8_t buf[14];
    readRegs(0x2D, buf, 14);

    int16_t ax = (buf[0] << 8) | buf[1];
    int16_t ay = (buf[2] << 8) | buf[3];
    int16_t az = (buf[4] << 8) | buf[5];

    int16_t gx = (buf[8] << 8) | buf[9];
    int16_t gy = (buf[10] << 8) | buf[11];
    int16_t gz = (buf[12] << 8) | buf[13];

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "imu_link";

    constexpr float ACCEL_SCALE = 16.0f * 9.80665f / 32768.0f;
    constexpr float GYRO_SCALE  = 2000.0f * M_PI / (180.0f * 32768.0f);

    msg.linear_acceleration.x = ax * ACCEL_SCALE;
    msg.linear_acceleration.y = ay * ACCEL_SCALE;
    msg.linear_acceleration.z = az * ACCEL_SCALE;

    msg.angular_velocity.x = gx * GYRO_SCALE;
    msg.angular_velocity.y = gy * GYRO_SCALE;
    msg.angular_velocity.z = gz * GYRO_SCALE;

    // No orientation estimate
    msg.orientation_covariance[0] = -1;

    pub_->publish(msg);
  }

  /* ---------------- Members ---------------- */

  std::string i2c_device_;
  int i2c_addr_;
  int rate_hz_;
  int fd_{-1};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Icm20948Node>());
  rclcpp::shutdown();
  return 0;
}
