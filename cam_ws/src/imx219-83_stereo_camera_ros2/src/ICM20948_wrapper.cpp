#include "ICM20948_wrapper.hpp"
extern "C" {
#include "ICM20948.h"
}




IcmWrapper::IcmWrapper(): Node("IcmWrapper"){
    

	rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

	imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("ICM/imu", qos_profile);
	mag_pub = this->create_publisher<sensor_msgs::msg::MagneticField>("ICM/mag", qos_profile);

    // initialize can reading threads
    i2c_listener_thread_ = std::thread(&IcmWrapper::i2c_read, this);
}

    // destructor
IcmWrapper::~IcmWrapper(){
    // join the thread so it shutsdown once it joins
    if(i2c_listener_thread_.joinable()){
        i2c_listener_thread_.join();
    }
}


void IcmWrapper::i2c_read(){
    
    IMU_EN_SENSOR_TYPE enMotionSensorType;
	IMU_ST_ANGLES_DATA stAngles;
	IMU_ST_SENSOR_DATA stGyroRawData;
	IMU_ST_SENSOR_DATA stAccelRawData;
	IMU_ST_SENSOR_DATA stMagnRawData;

	imuInit(&enMotionSensorType);
	bool status = false;
	if(IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType){
		printf("Motion sersor is ICM-20948\n" );
		status = true;
	}
	else{
		printf("Motion sersor NULL\nNo readout");
	}

	while(status){
		imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
		
		auto msg_imu = sensor_msgs::msg::Imu();
		msg_imu.header.stamp = this->get_clock()->now();
		msg_imu.header.frame_id = "ICM";

		msg_imu.linear_acceleration.x = float(stAccelRawData.fX);
		msg_imu.linear_acceleration.y = float(stAccelRawData.fY);
		msg_imu.linear_acceleration.z = float(stAccelRawData.fZ);
		msg_imu.linear_acceleration_covariance[0] = -1.0;

		msg_imu.angular_velocity.x = float(stGyroRawData.fX);
		msg_imu.angular_velocity.y = float(stGyroRawData.fY);
		msg_imu.angular_velocity.z = float(stGyroRawData.fZ);
		msg_imu.angular_velocity_covariance[0] = -1.0;

		msg_imu.orientation.x = float(stAngles.fRoll);
		msg_imu.orientation.y = float(stAngles.fPitch);
		msg_imu.orientation.z = float(stAngles.fYaw);
		msg_imu.orientation_covariance[0] = -1.0;

		imu_pub->publish(msg_imu);

		auto msg_mag = sensor_msgs::msg::MagneticField();
		msg_imu.header.stamp = this->get_clock()->now();
		msg_imu.header.frame_id = "ICM";

		msg_mag.magnetic_field.x = float(stMagnRawData.fX);
		msg_mag.magnetic_field.y = float(stMagnRawData.fY);
		msg_mag.magnetic_field.z = float(stMagnRawData.fZ);
		msg_mag.magnetic_field_covariance[0] = -1.0;

		mag_pub->publish(msg_mag);

	}
}



int main(int argc, char * argv[]){
    // start and run the node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IcmWrapper>());
    rclcpp::shutdown();
    return 0;
}

