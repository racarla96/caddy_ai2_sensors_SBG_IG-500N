#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
//#include <tf/Quaternion.h>
#include <tf/tf.h>

#include <boost/thread.hpp>
#include <sbg/sbg.h>


class SbgNode {
    public:
        SbgNode() {
            ros::NodeHandle pn("~");

            pn.param<std::string>("dev", dev_, "/dev/ttyUSB1");
            pn.param<int>("baudrate", baudrate_, 921600 /*115200*/);
            pn.param<int>("imu_frequency_divisor", imu_frequency_divisor_, 1);
            pn.param<double>("gps_period", gps_period_, 10);
  
            pn.param<std::string>("imu_frame_id", imu_frame_id_, "imu");
            pn.param<std::string>("gps_frame_id", gps_frame_id_, "gps");

            ig_ = boost::make_shared<sbg::ig500n>(dev_, baudrate_, imu_frequency_divisor_);
            ig_->init();

            ig_->setImuCallback(boost::bind(&SbgNode::publishImu, this, _1));
            ig_->setGpsCallback(boost::bind(&SbgNode::publishGps, this, _1));

            ig_->setGpsPeriod(gps_period_); 

            imu_pub_ = pn.advertise<sensor_msgs::Imu>("/imu", 1, boost::bind(&SbgNode::imuConnectCallback, this), boost::bind(&SbgNode::imuDisconnectCallback, this));
            gps_pub_ = pn.advertise<sensor_msgs::NavSatFix>("/gps", 1, boost::bind(&SbgNode::gpsConnectCallback, this), boost::bind(&SbgNode::gpsDisconnectCallback, this));
        }

        ~SbgNode() {
            ig_->imu_stop();
            ig_->gps_stop();
            ig_->deinit();
        }

        void imuConnectCallback() {
            ROS_INFO("connect imu callback: %d subscribers", imu_pub_.getNumSubscribers());
            ig_->imu_start();
        }
        
        void imuDisconnectCallback() {
            ROS_INFO("disconnect imu callback: %d subscribers", imu_pub_.getNumSubscribers());
            if (imu_pub_.getNumSubscribers() == 0) {
                ig_->imu_stop();
            }
        }
        
        void gpsConnectCallback() {
            ROS_INFO("connect gps callback: %d subscribers", imu_pub_.getNumSubscribers());
            ig_->gps_start();
        }
        
        void gpsDisconnectCallback() {
            ROS_INFO("disconnect gps callback: %d subscribers", imu_pub_.getNumSubscribers());
            if (gps_pub_.getNumSubscribers() == 0) {
                ig_->gps_stop();
            }
        }

        void publishGps(sbg::Gps *gps) {
            if (gps_pub_.getNumSubscribers() == 0)
                return;
       
            sensor_msgs::NavSatFix msg;

            msg.header.frame_id = gps_frame_id_; //es necessari tindre un frame per a la imu i un altre per al gps?
            msg.header.stamp = ros::Time::now();

            msg.latitude = gps->latitude;
            msg.longitude = gps->longitude;
            msg.altitude = gps->altitude;
            for (int i = 0; i < 9; i++)
                msg.position_covariance[i] = gps->position_covariance[i];
            msg.position_covariance_type = gps->position_covariance_type;
            msg.status.status = gps->fix_status;
            msg.status.service = gps->service_status;

            gps_pub_.publish(msg);
        }

        void publishImu(sbg::Imu *imu) {
            if (imu_pub_.getNumSubscribers() == 0)
                return;

            sensor_msgs::Imu msg;
           
            msg.header.frame_id = imu_frame_id_;
            msg.header.stamp = ros::Time::now();

            tf::Matrix3x3 nedTimu (imu->matrix[0], imu->matrix[3], imu->matrix[6], imu->matrix[1], imu->matrix[4], imu->matrix[7], imu->matrix[2], imu->matrix[5], imu->matrix[8]);
            tf::Matrix3x3 enuTned(0, 1, 0, 1, 0, 0, 0, 0, -1);
            tf::Matrix3x3 imuTros(1,0,0,0,-1,0,0,0,-1);
            tf::Matrix3x3 enuTros = enuTned * nedTimu * imuTros;

            tf::Quaternion q_rotated;
            enuTros.getRotation(q_rotated);            

            msg.orientation.x = q_rotated.x(); //esta canviar la orientacio
            msg.orientation.y = q_rotated.y();
            msg.orientation.z = q_rotated.z();
            msg.orientation.w = q_rotated.w();

            msg.angular_velocity.x = imu->angular_velocity[0];
            msg.angular_velocity.y = -imu->angular_velocity[1]; //canviem el signe de Y/Z per a expresar en el sistema que vol ros
            msg.angular_velocity.z = -imu->angular_velocity[2];
            
            msg.linear_acceleration.x = imu->linear_acceleration[0];
            msg.linear_acceleration.y = -imu->linear_acceleration[1];
            msg.linear_acceleration.z = -imu->linear_acceleration[2];
           
            for (int i = 0; i < 9; i++) {
                msg.orientation_covariance[i] = 0;
                msg.angular_velocity_covariance[i] = 0;
                msg.linear_acceleration_covariance[i] = 0;
            }
            
            for (int i = 0; i < 9; i+=3) {
                msg.orientation_covariance[i] = imu->covariance[0];
                msg.angular_velocity_covariance[i] = imu->covariance[1];
                msg.linear_acceleration_covariance[i] = imu->covariance[2];
            }

            imu_pub_.publish(msg);
        }

    private:

        ros::Publisher imu_pub_;
        ros::Publisher gps_pub_;

        boost::shared_ptr<sbg::ig500n> ig_;

        std::string dev_;
        int baudrate_;

        int imu_frequency_divisor_;
        double gps_period_;

        std::string gps_frame_id_;
        std::string imu_frame_id_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sbg_node");
//    sbg::ig500n ig("", 100, 100);
    SbgNode sbg;
    ros::spin();
    return 0;
}
