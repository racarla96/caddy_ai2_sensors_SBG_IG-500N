#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <sbgCom/sbgCom.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

using namespace std;

class SBGNode : public rclcpp::Node {
private:
  string port = "/dev/sbg";
  int baudrate = 921600;
  string imu_frame_id = "imu";
  string gps_frame_id = "gps";
  int frequency = 10;

  SbgProtocolHandle protocol_handle_; 

  const int IMU_OUTPUT_MASK = SBG_OUTPUT_QUATERNION | 
                          SBG_OUTPUT_GYROSCOPES |
                          SBG_OUTPUT_ACCELEROMETERS |
                          SBG_OUTPUT_MAGNETOMETERS;

  rclcpp::TimerBase::SharedPtr timer_;


  static void continuousErrorCallbackStatic(SbgProtocolHandleInt *pHandler, SbgErrorCode errorCode, void *pUsrArg)
  {
    reinterpret_cast<SBGNode*>(pUsrArg)->continuousErrorCallback(pHandler, errorCode, pUsrArg);
  }

  /*!
  *	Functon called each time we have an error on a continuous frame.
  *	\param[in]	pHandler			Our sbgCom protocol handler.
  *	\param[in]	errorCode			Error code that have occured during a continuous operation.
  *	\param[in]	pUsrArg				User argument pointer as defined in sbgSetContinuousErrorCallback function.
  */
  void continuousErrorCallback(SbgProtocolHandleInt *pHandler, SbgErrorCode errorCode, void *pUsrArg)
  {
    char errorMsg[256];

    //
    // Convert our error code to a human readable error message
    //
    sbgComErrorToString(errorCode, errorMsg);

    //
    // Display an error message
    //
    RCLCPP_WARN(this->get_logger(), "continuousErrorCallback: We have received the following error %s", errorMsg);
  }

  static void continuousCallbackStatic(SbgProtocolHandleInt *handler, SbgOutput *pOutput, void *pUsrArg)
  {
    reinterpret_cast<SBGNode*>(pUsrArg)->continuousCallback(handler, pOutput, pUsrArg);
  }

  /*!
  *	Functon called each time we have received a new data on a continuous frame.
  *	\param[in]	pHandler			Our sbgCom protocol handler.
  *	\param[in]	pOutput				Pointer on our received data struct. (You don't have the ownership so don't delete it!)
  *	\param[in]	pUsrArg				User argument pointer as defined in sbgSetContinuousModeCallback function.
  */
  void continuousCallback(SbgProtocolHandleInt *handler, SbgOutput *pOutput, void *pUsrArg)
  {
    // TODO: Decode the SBG output and publish the ROS2 message
    if (pOutput)
    {

    }
  }

  void periodicTask() {
    sbgProtocolContinuousModeHandle(protocol_handle_);
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

		sbgSetContinuousErrorCallback(protocol_handle_, continuousErrorCallbackStatic, NULL);
		sbgSetContinuousModeCallback(protocol_handle_, continuousCallbackStatic, NULL);

    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / frequency), std::bind(&SBGNode::periodicTask, this));
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