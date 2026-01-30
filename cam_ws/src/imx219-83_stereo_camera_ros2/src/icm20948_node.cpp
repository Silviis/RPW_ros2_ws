#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cmath>

static constexpr uint8_t REG_BANK_SEL      = 0x7F;

// Bank 0
static constexpr uint8_t PWR_MGMT_1        = 0x06;
static constexpr uint8_t ACCEL_XOUT_H      = 0x2D;

// Bank 2
static constexpr uint8_t GYRO_SMPLRT_DIV   = 0x00;
static constexpr uint8_t GYRO_CONFIG_1     = 0x01;
static constexpr uint8_t ACCEL_SMPLRT_DIV2 = 0x11;
static constexpr uint8_t ACCEL_CONFIG      = 0x14;

// Config
static constexpr uint8_t GYRO_FS_SEL_250_DPS  = 0x00;
static constexpr uint8_t GYRO_FS_SEL_500_DPS  = 0x02;
static constexpr uint8_t GYRO_FS_SEL_1000_DPS = 0x04;
static constexpr uint8_t GYRO_FS_SEL_2000_DPS = 0x06;

static constexpr uint8_t ACCEL_FS_SEL_2G  = 0x00;
static constexpr uint8_t ACCEL_FS_SEL_4G  = 0x02;
static constexpr uint8_t ACCEL_FS_SEL_8G  = 0x04;
static constexpr uint8_t ACCEL_FS_SEL_16G = 0x06;

// Bank values (IMPORTANT)
static constexpr uint8_t BANK_0 = 0x00;
static constexpr uint8_t BANK_2 = 0x20;

// Scaling
constexpr float ACCEL_LSB_2G   = 16384.0f;
constexpr float ACCEL_LSB_4G   = 8192.0f;
constexpr float ACCEL_LSB_8G   = 4096.0f;
constexpr float ACCEL_LSB_16G  = 2048.0f;  // LSB/g

constexpr float GYRO_LSB_250  = 131.0f;
constexpr float GYRO_LSB_500  = 65.5f;
constexpr float GYRO_LSB_1000 = 32.8f;
constexpr float GYRO_LSB_2000 = 16.4f;     // LSB/dps

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
    init();
    
    // calibrateGyroBias(500);

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

  void init()
  {
    // ---- Bank 0: reset & wake ----
    writeReg(REG_BANK_SEL, BANK_0);
    writeReg(PWR_MGMT_1, 0x80);          // Reset
    usleep(10000);                       // >=10 ms
    writeReg(PWR_MGMT_1, 0x01);          // Run mode (PLL)

    // ---- Bank 2: configure gyro + accel ----
    writeReg(REG_BANK_SEL, BANK_2);

    // Gyro: ±1000 dps, DLPF cfg 6, enable
    writeReg(GYRO_SMPLRT_DIV, 0x07);
    writeReg(GYRO_CONFIG_1, 0x30 | GYRO_FS_SEL_1000_DPS | 0x01);

    // Accel: ±2g, DLPF cfg 6, enable
    writeReg(ACCEL_SMPLRT_DIV2, 0x07);
    writeReg(ACCEL_CONFIG, 0x30 | ACCEL_FS_SEL_4G | 0x01);

    // ---- Back to Bank 0 ----
    writeReg(REG_BANK_SEL, BANK_0);
    usleep(100000);   // let filters settle
  }

  void calibrateGyroBias(int samples = 500)
  {
    int64_t sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < samples; ++i)
    {
      int16_t ax, ay, az, gx, gy, gz;
      readRaw(ax, ay, az, gx, gy, gz);

      sum_x += gx;
      sum_y += gy;
      sum_z += gz;

      usleep(2000); // ~500 Hz sampling
    }

    gyro_bias_x_ = sum_x / (float)samples;
    gyro_bias_y_ = sum_y / (float)samples;
    gyro_bias_z_ = sum_z / (float)samples;

    RCLCPP_INFO(get_logger(),
      "Gyro bias [LSB]: x=%.2f y=%.2f z=%.2f",
      gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
  }

  /* ---------------- Read & publish ---------------- */

  void readRaw(int16_t& ax, int16_t& ay, int16_t& az,
               int16_t& gx, int16_t& gy, int16_t& gz)
  {
    uint8_t buf[14];

    // ALWAYS ensure bank 0
    writeReg(REG_BANK_SEL, BANK_0);

    readRegs(0x2D, buf, 12);

    ax = (buf[0] << 8) | buf[1];
    ay = (buf[2] << 8) | buf[3];
    az = (buf[4] << 8) | buf[5];

    gx = (buf[6] << 8) | buf[7];
    gy = (buf[8] << 8) | buf[9];
    gz = (buf[10] << 8) | buf[11];
  }

  void readAndPublish()
  {
    int16_t ax, ay, az, gx, gy, gz;
    readRaw(ax, ay, az, gx, gy, gz);

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "imu_link";

    msg.linear_acceleration.x = (ax / ACCEL_LSB_4G) * 9.80665f;
    msg.linear_acceleration.y = (ay / ACCEL_LSB_4G) * 9.80665f;
    msg.linear_acceleration.z = (az / ACCEL_LSB_4G) * 9.80665f;

    msg.angular_velocity.x = ((gx - gyro_bias_x_) / GYRO_LSB_1000) * M_PI / 180.0f;
    msg.angular_velocity.y = ((gy - gyro_bias_y_) / GYRO_LSB_1000) * M_PI / 180.0f;
    msg.angular_velocity.z = ((gz - gyro_bias_z_) / GYRO_LSB_1000) * M_PI / 180.0f;

    // No orientation estimate
    msg.orientation_covariance[0] = -1;

    pub_->publish(msg);
  }

  /* ---------------- Members ---------------- */

  std::string i2c_device_;
  int i2c_addr_;
  int rate_hz_;
  int fd_{-1};

  float gyro_bias_x_ = 0.0f;
  float gyro_bias_y_ = 0.0f;
  float gyro_bias_z_ = 0.0f;

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
