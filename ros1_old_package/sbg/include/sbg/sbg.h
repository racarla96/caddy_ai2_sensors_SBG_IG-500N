#include <stdio.h>

#include <string>

#include <boost/function.hpp>
#include <boost/thread.hpp>

#include <sbg/sbgCom.h>

namespace sbg {

    void continuousErrorCallback(SbgProtocolHandleInt *pHandler, SbgErrorCode errorCode, void *pUsrArg);
    void continuousCallback(SbgProtocolHandleInt *handler, SbgOutput *pOutput, void *pUsrArg);

    typedef struct Gps {
        double latitude; // north positive
        double longitude; // east positive
        double altitude; // postive above WGS84

        double position_covariance[9]; //or accuracy?
        int position_covariance_type;

        int fix_status;
        int service_status;
    } Gps;

    typedef struct Imu {
        double matrix[9]; //orientation matrix , column major
        double angular_velocity[3]; // is data alignment (array of size 4) needed?
        double linear_acceleration[3];
        double covariance[3];       // of orientation, angular_velocity and linear_accelerations
//        double orientation[4];      // quaternion = w,x,y,z
//        double euler[3];
    } Imu;

    const int IMU_OUTPUT_MASK = SBG_OUTPUT_MATRIX |       // SBG_OUTPUT_EULER | SBG_OUTPUT_QUATERNION 
                          SBG_OUTPUT_GYROSCOPES |   // SBG_OUTPUT_DELTA_ANGLES
                          SBG_OUTPUT_ACCELEROMETERS;

    const int GPS_OUTPUT_MASK = SBG_OUTPUT_POSITION |
                          SBG_OUTPUT_NAV_ACCURACY |
                          SBG_OUTPUT_GPS_INFO;

    //orientation, angular_velocity, linear_acceleration
    const double IMU_COVARIANCES[3] = {0.0174532925, 0.00872664625, 0.049};

    class ig500n {
        public:
            ig500n(std::string dev, int baudrate, int imu_frequency_divisor)
            {
                protocol_handle_ = NULL;
                initialized_ = false;
                last_error_ = SBG_NO_ERROR;

                imu_started_ = false;
                imu_stop_thread_ = true;
                imu_frequency_divisor_ = imu_frequency_divisor;

                gps_started_ = false;
                gps_stop_thread_ = true;
                gps_period_ = 1000;

                dev_ = std::string(dev);
                baudrate_ = baudrate;

                printf("sbg: dev = %s, baud = %d, freq = %d\n", dev.c_str(), baudrate, imu_frequency_divisor);
            }

            ~ig500n() {}

            void init() {

                //check dev_ and baudrate_?
                last_error_ = sbgComInit(dev_.c_str(), baudrate_, &protocol_handle_);
                if (checkError())
                    return;

                sbgSleep(50); //wait initilization

                last_error_ = sbgSetDefaultOutputMask(protocol_handle_, IMU_OUTPUT_MASK);
                if (checkError())
                    return;
                
                last_error_ = sbgSetContinuousErrorCallback(protocol_handle_, continuousErrorCallback, this);
                if (checkError())
                    return;
                
                last_error_ = sbgSetContinuousModeCallback(protocol_handle_, continuousCallback, this);
                if (checkError())
                    return;

                initialized_ = true;
            }
            
            void deinit() {
                if (!protocol_handle_)
                    return;

                last_error_ = sbgProtocolClose(protocol_handle_);
                if (checkError())
                    return;
                protocol_handle_ = NULL;
            }


            bool imu_start() {
                if (!initialized_ || imu_started_)
                    return false;
                last_error_ = sbgSetContinuousMode(protocol_handle_, SBG_CONTINUOUS_MODE_ENABLE, imu_frequency_divisor_);
                
                if (checkError())
                    return false;

                imu_stop_thread_ = false;
                imu_started_ = true;
                imu_thread_ = boost::thread(&ig500n::imu_run, this);

                return true;
            }

            bool imu_stop() {
                if (!imu_started_)
                    return true;
  
                last_error_ = sbgSetContinuousMode(protocol_handle_, SBG_CONT_TRIGGER_MODE_DISABLE, 1);
                if (checkError())
                    return false;
               
                imu_stop_thread_ = true;
                imu_thread_.join();
                imu_started_ = false;
                return true;
            }

            void imu_run() {
                while (!imu_stop_thread_) {
                    sbgProtocolContinuousModeHandle(protocol_handle_);
                    sbgSleep(10);
                }
            }

            bool gps_start() {
                if (gps_started_)
                    return false;

                gps_stop_thread_ = false;
                gps_started_ = true;
                gps_thread_ = boost::thread(&ig500n::gps_run, this);
                return true;
            }

            bool gps_stop() {
                if (!gps_started_)
                    return false;

                gps_stop_thread_ = true;
                gps_thread_.join();
                gps_started_ = false;
                return true;
            }

            void gps_run() {
                SbgOutput output;
                Gps gps;

                while (!gps_stop_thread_) {
                    last_error_ = sbgGetSpecificOutput(protocol_handle_, GPS_OUTPUT_MASK, &output);
                    if (checkError())
                        continue;

                    if (output.outputMask & GPS_OUTPUT_MASK) { // publicar sempre? o nomes quan se te senyal?

                        gps.latitude = output.position[0]; // tambe podem tindre la mesura directa del gps 
                        gps.longitude = output.position[1];
                        gps.altitude = output.position[2]; 
                        
                        /*printf("flags: %d\n", output.gpsFlags & 0x03);
                        printf("pos accuracy: %g\n", output.positionAccuracy);
                        printf("vel accuracy: %g\n", output.velocityAccuracy);
                        printf("  satellites: %d\n", output.gpsNbSats);
                        */
                        for (int i = 0; i < 9; i++)
                            gps.position_covariance[i] = 0;
                
                        //0 = desconocida, 1 = aproximada, 2 = solo diagonal, 3 = matriz entera
                        gps.position_covariance_type = 1;
                        for (int i = 0; i <= 8; i+=4) //la covarianza es en metros, no relativa a la lat/long
                            gps.position_covariance[i] = output.positionAccuracy; //o al cuadrado??

                        if ((output.gpsFlags & 0x03))
                            gps.fix_status =  0; // we have 3D location
                        else 
                            gps.fix_status = -1; // we don't have enough info
                        gps.service_status = 1; //gps
                        callGpsCallback(&gps);
                    }
                    sbgSleep(gps_period_);
                }
            }

            void setImuCallback(boost::function<void (Imu *)> callback) {
                imu_callback_ = callback;
            }

            void setKillCallback(boost::function<void (unsigned long)> callback) { 
                kill_callback_ = callback;
            }

            void setGpsCallback(boost::function<void (Gps *)> callback) {
                gps_callback_ = callback;
            }

            void callImuCallback(Imu *imu) {
                if (!imu_callback_.empty())
                    imu_callback_(imu);
            }

            void callKillCallback() {
            }
            
            void callGpsCallback(Gps *gps) {
                if (!gps_callback_.empty())
                    gps_callback_(gps);
            }

            void setGpsPeriod(int ms) {
                gps_period_ = ms;
            }

            void setGpsFrequency(double hz) {
                gps_period_ = 1000.0/hz;
            }

        private:
            bool checkError() { //return true if there is an error. maybe error messaging?
                printf("Last error: %d\n", last_error_);
                return last_error_ != SBG_NO_ERROR;
            }

            SbgProtocolHandle protocol_handle_; 
            SbgErrorCode last_error_;

            bool initialized_;
            std::string dev_;
            int baudrate_;

            boost::function<void (Imu *)> imu_callback_;
            boost::function<void (Gps *)> gps_callback_;
            boost::function<void (unsigned long UniqueId)> kill_callback_;

            boost::thread imu_thread_;
            bool imu_started_;
            bool imu_stop_thread_;
            double imu_frequency_divisor_;
            double imu_covariances_[3];

            boost::thread gps_thread_;
            double gps_period_; // ms
            bool gps_stop_thread_;
            bool gps_started_;

    };
    
    void continuousErrorCallback(SbgProtocolHandleInt *pHandler, SbgErrorCode errorCode, void *pUsrArg)
    {
        ig500n *ig = (ig500n *) pUsrArg;

        printf("error callback\n");
    }

    void continuousCallback(SbgProtocolHandleInt *handler, SbgOutput *pOutput, void *pUsrArg)
    {
        ig500n *ig = (ig500n *) pUsrArg;

        Imu imu;
        
/*        if (pOutput->outputMask & SBG_OUTPUT_EULER)
            for (int i = 0; i < 3; i++)
                imu.euler[i] = pOutput->stateEuler[i];

        if (pOutput->outputMask & SBG_OUTPUT_QUATERNION)
            for (int i = 0; i < 4; i++)
                imu.orientation[i] = pOutput->stateQuat[i];
*/
        if (pOutput->outputMask & SBG_OUTPUT_MATRIX) {
            for (int i = 0; i < 9; i++)
                imu.matrix[i] = pOutput->stateMatrix[i];
            imu.covariance[0] = IMU_COVARIANCES[0];
        }
        
        if (pOutput->outputMask & SBG_OUTPUT_GYROSCOPES) {
            for (int i = 0; i < 3; i++)
                imu.angular_velocity[i] = pOutput->gyroscopes[i];
            imu.covariance[1] = IMU_COVARIANCES[1];
        }
        
        if (pOutput->outputMask & SBG_OUTPUT_ACCELEROMETERS) {
            for (int i = 0; i < 3; i++)
                imu.linear_acceleration[i] = pOutput->accelerometers[i];
            imu.covariance[2] = IMU_COVARIANCES[2];
        }
        
        ig->callImuCallback(&imu);
    }
};
