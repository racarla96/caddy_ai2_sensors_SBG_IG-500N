#include <rclcpp/rclcpp.hpp>
#include <unistd.h> // para Linux 
#include <chrono>
#include <sbgCom/sbgCom.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

using namespace std;

class SBGNode : public rclcpp::Node {
private:
  string port = "/dev/sbg";
  int baudrate = 921600;
  string imu_frame_id = "imu";
  string imu_frame_ned_id = imu_frame_id + "_ned";
  string gps_frame_id = "gps";
  int frequency = 500;

  SbgOutput pOutput;
  SbgProtocolHandle protocol_handle_; 
  SbgErrorCode last_error_;

  tf2::Vector3 _vec;
  tf2::Matrix3x3 IMU2ROS;
  tf2::Matrix3x3 NED2ENU;
  tf2::Matrix3x3 _output_matrix;
  tf2::Quaternion _quat;
  tf2::Matrix3x3 _aux;

  double gravity = 9.81;

  // https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html
  //orientation, angular_velocity, linear_acceleration
  const double IMU_COVARIANCES[3] = {0.0174532925, 0.00872664625, 0.049};
  std::shared_ptr<sensor_msgs::msg::Imu> imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
  std::shared_ptr<sensor_msgs::msg::Imu> imu_ned_msg = std::make_shared<sensor_msgs::msg::Imu>();
  const int IMU_OUTPUT_MASK = SBG_OUTPUT_MATRIX | 
                              SBG_OUTPUT_GYROSCOPES |
                              SBG_OUTPUT_ACCELEROMETERS;

  // https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatStatus.html
  // https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html
  std::shared_ptr<sensor_msgs::msg::NavSatFix> gps_msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
  const int GPS_OUTPUT_MASK = SBG_OUTPUT_POSITION |
                              SBG_OUTPUT_NAV_ACCURACY |
                              SBG_OUTPUT_GPS_INFO;  

  const int OUTPUT_MASK = IMU_OUTPUT_MASK |
                          GPS_OUTPUT_MASK;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_ned_pub;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub;
  rclcpp::TimerBase::SharedPtr timer_;

  void periodicTask() {
    sbgProtocolContinuousModeHandle(protocol_handle_);
    sbgGetDefaultOutput(protocol_handle_, &pOutput);

    if (pOutput.outputMask)
    {
      if (pOutput.outputMask & IMU_OUTPUT_MASK) {
        imu_msg->header.stamp = this->now();
        imu_msg->header.frame_id = imu_frame_id;

        imu_ned_msg->header.stamp = this->now();
        imu_ned_msg->header.frame_id = imu_frame_ned_id;

        // Applied a 180 degrees rotation around the X axis to match car standard orientation

        // https://es.mathworks.com/help/map/choose-a-3-d-coordinate-system.html

        //pOutput.stateMatrix[0]
        _output_matrix.setValue(pOutput.stateMatrix[0], pOutput.stateMatrix[1], pOutput.stateMatrix[2],
                      pOutput.stateMatrix[3], pOutput.stateMatrix[4], pOutput.stateMatrix[5],
                      pOutput.stateMatrix[6], pOutput.stateMatrix[7], pOutput.stateMatrix[8]);
        /*
        _aux = IMU2ROS * _output_matrix;
        _aux.getRotation(_quat);

        imu_msg->orientation.w = _quat.w();
        imu_msg->orientation.x = _quat.x();
        imu_msg->orientation.y = _quat.y();
        imu_msg->orientation.z = _quat.z();

        _vec.setValue(pOutput.gyroscopes[0], pOutput.gyroscopes[1], pOutput.gyroscopes[2]);
        _vec = IMU2ROS * _vec;

        imu_msg->angular_velocity.x = _vec.x();
        imu_msg->angular_velocity.y = _vec.y();
        imu_msg->angular_velocity.z = _vec.z();

        _vec.setValue(pOutput.accelerometers[0], pOutput.accelerometers[1], pOutput.accelerometers[2]);
        _vec = IMU2ROS * _vec;

        imu_msg->linear_acceleration.x = _vec.x();
        imu_msg->linear_acceleration.y = _vec.y();
        imu_msg->linear_acceleration.z = _vec.z();
        */
        _output_matrix.getRotation(_quat);

        imu_ned_msg->orientation.w = _quat.w();
        imu_ned_msg->orientation.x = _quat.x();
        imu_ned_msg->orientation.y = _quat.y();
        imu_ned_msg->orientation.z = _quat.z();

        imu_ned_msg->angular_velocity.x = pOutput.gyroscopes[0];
        imu_ned_msg->angular_velocity.y = pOutput.gyroscopes[1];
        imu_ned_msg->angular_velocity.z = pOutput.gyroscopes[2];

        imu_ned_msg->linear_acceleration.x = pOutput.accelerometers[0];
        imu_ned_msg->linear_acceleration.y = pOutput.accelerometers[1];
        imu_ned_msg->linear_acceleration.z = pOutput.accelerometers[2];

        imu_pub->publish(*imu_msg);
        imu_ned_pub->publish(*imu_ned_msg);
      }
      if (pOutput.outputMask & GPS_OUTPUT_MASK) {
        
        gps_msg->header.stamp = this->now();
        gps_msg->header.frame_id = gps_frame_id;

        gps_msg->latitude = pOutput.position[0]; // tambe podem tindre la mesura directa del gps 
        gps_msg->longitude = pOutput.position[1];
        gps_msg->altitude = pOutput.position[2]; 
        
        for (int i = 0; i < 9; i++)
            gps_msg->position_covariance[i] = 0;

        //0 = desconocida, 1 = aproximada, 2 = solo diagonal, 3 = matriz entera
        gps_msg->position_covariance_type = 1;
        for (int i = 0; i <= 8; i+=4) //la covarianza es en metros, no relativa a la lat/long
            gps_msg->position_covariance[i] = pOutput.positionAccuracy; //o al cuadrado??

        if ((pOutput.gpsFlags & 0x03))
            gps_msg->status.status =  0; // we have 3D location
        else 
            gps_msg->status.status = -1; // we don't have enough info
        gps_msg->status.service = 1; //gps

        gps_pub->publish(*gps_msg);
      }
    }    
  }

  // TODO: Imprimir bien el mensaje de error
  bool checkError(string msg) {
  	if (last_error_ != SBG_NO_ERROR){
      RCLCPP_ERROR(this->get_logger(), "Error on SBG node:");
//      RCLCPP_ERROR(this->get_logger(), "Error on SBG node: %s", msg.c_str());
      return true;
    }
    return false;
  }

public:
  SBGNode(const rclcpp::NodeOptions &options) : Node("sbg_node", options){
    this->declare_parameter("port", port);
    this->get_parameter("port", port);

    this->declare_parameter("baudrate", baudrate);
    this->get_parameter("baudrate", baudrate);

    this->declare_parameter("imu_frame_id", imu_frame_id);
    this->get_parameter("imu_frame_id", imu_frame_id);

    this->declare_parameter("gps_frame_id", gps_frame_id);
    this->get_parameter("gps_frame_id", gps_frame_id);

    this->declare_parameter("frequency", frequency);
    this->get_parameter("frequency", frequency);

    last_error_ = sbgComInit(port.c_str(), baudrate, &protocol_handle_);
    if(checkError("sbgComInit")) return;
    usleep(50*1000);    // time_period en microsegundos

    last_error_ = sbgSetDefaultOutputMask(protocol_handle_, OUTPUT_MASK);
    if(checkError("sbgSetDefaultOutputMask")) return;

    last_error_ = sbgSetContinuousMode(protocol_handle_, SBG_CONTINUOUS_MODE_ENABLE, 1);
    if(checkError("sbgSetContinuousMode: SBG_CONTINUOUS_MODE_ENABLE")) return;

    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    imu_ned_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu_ned", 1);
    gps_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", 1);
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / frequency), std::bind(&SBGNode::periodicTask, this));

    // Constants values
    for (int i = 0; i < 9; i++) {
        imu_msg->orientation_covariance[i] = 0;
        imu_msg->angular_velocity_covariance[i] = 0;
        imu_msg->linear_acceleration_covariance[i] = 0;
    }
    
    for (int i = 0; i < 9; i+=3) {
        imu_msg->orientation_covariance[i] = IMU_COVARIANCES[0];
        imu_msg->angular_velocity_covariance[i] = IMU_COVARIANCES[1];
        imu_msg->linear_acceleration_covariance[i] = IMU_COVARIANCES[2];
    }

    for (int i = 0; i < 9; i++) {
        imu_ned_msg->orientation_covariance[i] = 0;
        imu_ned_msg->angular_velocity_covariance[i] = 0;
        imu_ned_msg->linear_acceleration_covariance[i] = 0;
    }
    
    for (int i = 0; i < 9; i+=3) {
        imu_ned_msg->orientation_covariance[i] = IMU_COVARIANCES[0];
        imu_ned_msg->angular_velocity_covariance[i] = IMU_COVARIANCES[1];
        imu_ned_msg->linear_acceleration_covariance[i] = IMU_COVARIANCES[2];
    }

    IMU2ROS.setValue(1.0,  0.0,  0.0,
                     0.0, -1.0,  0.0,
                     0.0,  0.0, -1.0);

    NED2ENU.setValue(0.0,  1.0,  0.0,
                     1.0,  0.0,  0.0,
                     0.0,  0.0, -1.0);

    RCLCPP_INFO(this->get_logger(), "SBG node started");
  }

  // Destructor
  ~SBGNode() {
    last_error_ = sbgSetContinuousMode(protocol_handle_, SBG_CONT_TRIGGER_MODE_DISABLE, 1);
    if(checkError("sbgSetContinuousMode: SBG_CONT_TRIGGER_MODE_DISABLE")) return;

    last_error_ = sbgProtocolClose(protocol_handle_);
    if(checkError("sbgProtocolClose")) return;

    RCLCPP_INFO(this->get_logger(), "SBG node destroyed");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<SBGNode>(options));
  rclcpp::shutdown();
  return 0;   
}