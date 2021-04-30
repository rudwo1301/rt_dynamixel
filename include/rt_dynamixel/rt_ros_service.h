
#ifndef RT_ROS_SERVICE_H_
#define RT_ROS_SERVICE_H_


#include <ros/ros.h>

#include <realtime_tools/realtime_publisher.h>
#include <pthread.h>

//#include "rt_dynamixel_pro.h"
#include "rt_dynamixel_msgs/JointState.h"
#include "rt_dynamixel_msgs/JointSet.h"
#include "rt_dynamixel_msgs/ModeSetting.h"
#include "rt_dynamixel_msgs/MotorSetting.h"

#include "dxl_lists.h"

using namespace DXL_PRO;

// global ----------------------------------------------
extern RTDynamixelPro dxlDevice[4];

extern int nTotalMotors;
extern int nDXLCount[4];
extern dxl_inverse dxlID2Addr[60]; // max ID = 50,

extern dxl_pro_data& dxl_from_id(int id);

// ------------------------------------------------------

// rt_task_proc -----------------------------------------
void *publisher_proc(void *arg);
void *subscribe_proc(void *arg);
void JointStateSubCallBack(const rt_dynamixel_msgs::JointSet);
void *motor_set_proc(void *arg);
// ------------------------------------------------------

class RTROSPublisher
{
public:
    realtime_tools::RealtimePublisher<rt_dynamixel_msgs::JointState> pubState;
    //rosrt::Publisher<rt_dynamixel_msgs::JointState> pubState;
    rt_dynamixel_msgs::JointState* jointMsg;
    pthread_t pubthread;
    RTROSPublisher(ros::NodeHandle &nh);
    virtual ~RTROSPublisher()
    {        
        pthread_exit(&pubthread);
        //rt_task_delete(&rttTaskObject);    
    }
    void start()
    {   
        //pthread_create(&pubthread, NULL, publisher_proc, (void*)this);     
        //pthread_detach(pubthread);
        //rt_task_start(&rttTaskObject, &publisher_proc, (void*)this);
    }

};

class RTROSSubscriber
{
public:
    ros::Subscriber subSetter;
    //rosrt::Subscriber<rt_dynamixel_msgs::JointSet> subSetter;
    pthread_t subthread;
    RTROSSubscriber(ros::NodeHandle &nh);
    virtual ~RTROSSubscriber()
    {        
        pthread_exit(&subthread);
         }

    void start()
    {        
        //pthread_create(&subthread, NULL, subscribe_proc, (void*)this);
        //pthread_detach(subthread);
        //rt_task_start(&rttSubscriber, &subscribe_proc, (void*)this);    
    }

};



class RTROSMotorSettingService
{
private:

    bool modeSwitch(rt_dynamixel_msgs::ModeSettingRequest &req,
                    rt_dynamixel_msgs::ModeSettingResponse &res);

    bool motorSet(rt_dynamixel_msgs::MotorSettingRequest &req,
                    rt_dynamixel_msgs::MotorSettingResponse &res);

public:
    ros::ServiceServer modeServer;
    ros::ServiceServer motorServer;

    rt_dynamixel_msgs::MotorSettingRequest motorRequest;
    rt_dynamixel_msgs::MotorSettingResponse motorResponse;

    RTROSMotorSettingService(ros::NodeHandle &nh);
    virtual ~RTROSMotorSettingService()
    {           }



};



#endif // RT_ROS_SERVICE_H_
