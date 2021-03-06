#include "rt_ros_service.h"

//RTIME control_period = 25e5;

RTROSPublisher::RTROSPublisher(ros::NodeHandle &nh)
{
    pubState.init(nh,"rt_dynamixel/joint_state", 1);
    //pubState.initialize(nh.advertise<rt_dynamixel_msgs::JointState>("rt_dynamixel/joint_state",1),
    //                    1, rt_dynamixel_msgs::JointState());
    
    pthread_attr_t attr;
    sched_param param;
    
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = 10;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    //rt_task_create(&rttTaskObject,"dxl ros pub",0,10,0);
    
    //jointMsg = pubState.allocate();
    jointMsg = new rt_dynamixel_msgs::JointState[1];
    jointMsg->id.resize(nTotalMotors);
    jointMsg->angle.resize(nTotalMotors);
    jointMsg->velocity.resize(nTotalMotors);
    jointMsg->current.resize(nTotalMotors);
    jointMsg->updated.resize(nTotalMotors);

    int _cnt=0;
    for(int i=0;i<4;i++)
        for(int j=0;j<nDXLCount[i];j++)
        {
            jointMsg->id[_cnt++] = dxlLists[i][j].id;
        }
}

RTROSSubscriber::RTROSSubscriber(ros::NodeHandle &nh)
{
    pthread_attr_t attr;
    sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = 9;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    //rt_task_create(&rttSubscriber,"dxl ros sub",0,9,0);
    subSetter = nh.subscribe("rt_dynamixel/joint_set", 5, &JointStateSubCallBack);
    //subSetter.initialize(3,nh,"rt_dynamixel/joint_set");
}


RTROSMotorSettingService::RTROSMotorSettingService(ros::NodeHandle &nh)
{
    modeServer = nh.advertiseService("rt_dynamixel/mode",&RTROSMotorSettingService::modeSwitch,this);
    motorServer = nh.advertiseService("rt_dynamixel/motor_set",&RTROSMotorSettingService::motorSet,this);
}

bool RTROSMotorSettingService::modeSwitch(rt_dynamixel_msgs::ModeSettingRequest &req,
                rt_dynamixel_msgs::ModeSettingResponse &res)
{

    res.result = -1;
    switch (req.mode)
    {
    case rt_dynamixel_msgs::ModeSettingRequest::CONTROL_RUN:
        for(int i=0;i<4;i++)
        {
            dxlDevice[i].bControlWriteEnable = true;
        }
        res.result =  rt_dynamixel_msgs::ModeSettingRequest::CONTROL_RUN;
        break;

    case rt_dynamixel_msgs::ModeSettingRequest::DISABLE:
        for(int i=0;i<4;i++)
        {
            dxlDevice[i].bControlWriteEnable = false;
        }

        // wait for process end
        for(int i=0;i<4;i++)
            while(dxlDevice[i].bControlLoopProcessing) {}
        res.result =  rt_dynamixel_msgs::ModeSettingRequest::DISABLE;
        break;

    case rt_dynamixel_msgs::ModeSettingRequest::SETTING:
        for(int i=0;i<4;i++)
        {
            dxlDevice[i].bControlWriteEnable = false;
        }

        // wait for process end
        for(int i=0;i<4;i++)
            while(dxlDevice[i].bControlLoopProcessing) {}
        res.result =  rt_dynamixel_msgs::ModeSettingRequest::SETTING;

        break;

    default:
        break;
    }


    return true;
}


bool RTROSMotorSettingService::motorSet(rt_dynamixel_msgs::MotorSettingRequest &req,
                rt_dynamixel_msgs::MotorSettingResponse &res)
{
    for(int i=0; i<4; i++)
    {
        dxlDevice[i].bControlLoopEnable = false;
    }
    for(int i=0; i<4; i++)
    {
        while(dxlDevice[i].bControlLoopProcessing) {}
    }


    pthread_t motorsetthread;
    pthread_attr_t attr;
    sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = 7;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    //rt_task_create(&rttMotorSetTask,"dxl motorset service",0,7,T_JOINABLE);
    motorResponse.result = -1;

    motorRequest = req;
    
   // pthread_create(&motorsetthread, &attr, motor_set_proc, NULL);
    //rt_task_start(&rttMotorSetTask, &motor_set_proc, (void*)this);
    int status;

    //pthread_join(motorsetthread, (void**)&status);
    //rt_task_join(&rttMotorSetTask);
    //rt_task_delete(&rttMotorSetTask);
    if(status < 0)
    {
        printf("%s\n", strerror(status));
    }
    res = motorResponse;
    //res.result = req.mode;

    for(int i=0; i<4; i++)
    {
        dxlDevice[i].bControlLoopEnable = true;
    }
    return true;
}

//////////////////////////////////////////////////////
/// \brief publisher_proc
/// \param arg
///
///
///
long pub_period = 50e5;
void *publisher_proc(void *arg)
{
   /* RTROSPublisher* pObj = (RTROSPublisher*)arg;
    int i,j;

    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    //rt_task_set_periodic(NULL, TM_NOW, 50e5);    // 1e6 -> 1ms   5e5 -> 500us

    while (1)
    {
        ts.tv_nsec += pub_period;
        while(ts.tv_nsec >= SEC_IN_NSEC)
        {
            ts.tv_sec++;
            ts.tv_nsec -= SEC_IN_NSEC;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        //rt_task_wait_period(NULL); //wait for next cycle

        // Data set
        for(i=0;i<4;i++)
        {
            dxlDevice[i].mutex_acquire();
        }

        int _cnt = 0;
        for(i=0;i<4;i++)
        {
            for(j=0;j<nDXLCount[i];j++)
            {
                // SI
                pObj->jointMsg->angle[_cnt] = dxlDevice[i][j].position_rad();
                pObj->jointMsg->velocity[_cnt] = dxlDevice[i][j].velocity_radsec();
                pObj->jointMsg->current[_cnt] = dxlDevice[i][j].current_amp();
                pObj->jointMsg->updated[_cnt] = dxlDevice[i][j].updated;
                _cnt++;
            }
        }

        for(i=0;i<4;i++)
        {
            dxlDevice[i].mutex_release();
        }
        ros::NodeHandle nh;
        (pObj->pubState).init(nh, "jointMsg",1);
        
        //pObj->pubState.publish(pObj->jointMsg);
    }
    */
}


long sub_period = 1e6;

void *subscribe_proc(void *arg)
{
  /*  RTROSSubscriber* rtsub = (RTROSSubscriber*)arg;

    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    //rt_task_set_periodic(NULL, TM_NOW, 1e6);    // 1e6 -> 1ms
    int i;
    while(1)
    {
        ts.tv_nsec += sub_period;
        while(ts.tv_nsec >= SEC_IN_NSEC)
        {
            ts.tv_sec++;
            ts.tv_nsec -= SEC_IN_NSEC;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        //rt_task_wait_period(NULL); //wait for next cycle

        ros::spin();
    }
    */
}

void JointStateSubCallBack(const rt_dynamixel_msgs::JointSet rcvMsg)
{
    //rt_dynamixel_msgs::JointSetConstPtr rcvMsg = rtsub->subSetter.poll();
    if(&rcvMsg) // if message recieved ( if not rcvMsg == NULL )
    {
        // Data set
        // ROS_INFO("Sub ")
        int i;
        for(i=0;i<4;i++)
        {
            dxlDevice[i].mutex_acquire();
        }
        
        for(i=0;i< (int)((&rcvMsg)->id.size());i++)
        {
            if(check_vaild_dxl_from_id((&rcvMsg)->id[i]))
            {
                dxl_from_id((&rcvMsg)->id[i]).aim_radian = (&rcvMsg)->angle[i];
            }
        }

        for(i=0;i<4;i++)
        {
            dxlDevice[i].mutex_release();
        }
    }
}

void* motor_set_proc(void *arg)
{
    RTROSMotorSettingService *pObj = (RTROSMotorSettingService*)arg;
    int channel;
    int index;
    int error;
    switch (pObj->motorRequest.mode)
    {
    case rt_dynamixel_msgs::MotorSettingRequest::SET_TORQUE_ENABLE:
        for (int i=0; i<4; i++)
        {
            dxlDevice[i].setAllTorque(pObj->motorRequest.value);
            pObj->motorResponse.result = pObj->motorRequest.mode;
        }
        break;

    case rt_dynamixel_msgs::MotorSettingRequest::SET_GOAL_POSITION:
        if(check_vaild_dxl_from_id(pObj->motorRequest.id))
        {
            channel = dxlID2Addr[pObj->motorRequest.id].channel;
            index = dxlID2Addr[pObj->motorRequest.id].index;
            dxlDevice[channel].setAimRadian(index,pObj->motorRequest.fvalue,&error);
            pObj->motorResponse.result = pObj->motorRequest.mode;
        }
        break;

    case rt_dynamixel_msgs::MotorSettingRequest::GET_HOMING_OFFSET:
        if(check_vaild_dxl_from_id(pObj->motorRequest.id))
        {
            channel = dxlID2Addr[pObj->motorRequest.id].channel;
            index = dxlID2Addr[pObj->motorRequest.id].index;
            dxlDevice[channel].getHomingOffset(index,pObj->motorRequest.value,&pObj->motorResponse.value,&error);
            pObj->motorResponse.result = pObj->motorRequest.mode;
            //pObj->motorResponse
        }
        break;

    case rt_dynamixel_msgs::MotorSettingRequest::SET_HOMING_OFFSET:
        if(check_vaild_dxl_from_id(pObj->motorRequest.id))
        {
            channel = dxlID2Addr[pObj->motorRequest.id].channel;
            index = dxlID2Addr[pObj->motorRequest.id].index;
            pObj->motorResponse.result = pObj->motorRequest.mode;
        }
        break;

    default:
        break;
    }
    
    timespec temp;
    temp.tv_sec = 0;
    temp.tv_nsec = 5e6;
    nanosleep(&temp, NULL);
    //rt_task_sleep(5e6);
}
