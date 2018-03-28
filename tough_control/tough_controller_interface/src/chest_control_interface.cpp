#include <tough_controller_interface/chest_control_interface.h>
#include <tf/transform_listener.h>

#define TO_RADIANS M_PI / 180.0 //goes probably in utils which stores similar math operation parameters

ChestControlInterface::ChestControlInterface(ros::NodeHandle nh):ToughControllerInterface(nh)
{

    chestTrajPublisher_ =
            nh_.advertise<ihmc_msgs::ChestTrajectoryRosMessage>(control_topic_prefix_ +"/chest_trajectory",1,true);
}

ChestControlInterface::~ChestControlInterface()
{
}

void ChestControlInterface::controlChest(float roll , float pitch , float yaw, float time, int execution_mode)
{

    roll  =  roll*TO_RADIANS;
    pitch = pitch*TO_RADIANS;
    yaw   =   yaw*TO_RADIANS;

    tf::Quaternion quatInPelvisFrame;
    quatInPelvisFrame.setRPY(roll,pitch,yaw);
    geometry_msgs::Quaternion quat;
    tf::quaternionTFToMsg(quatInPelvisFrame, quat);

    controlChest(quat, time, execution_mode);
}

void ChestControlInterface::controlChest(geometry_msgs::Quaternion quat, float time, int execution_mode)
{
    ihmc_msgs::ChestTrajectoryRosMessage msg;
    ihmc_msgs::SO3TrajectoryPointRosMessage data;

    data.time = time;
    data.orientation = quat;

    msg.unique_id = ChestControlInterface::id_++;
    msg.execution_mode = execution_mode;

    ihmc_msgs::FrameInformationRosMessage reference_frame;
    reference_frame.trajectory_reference_frame_id = rd_->getPelvisFrameHash();   //Pelvis frame
    reference_frame.data_reference_frame_id = rd_->getPelvisFrameHash();//Pelvis frame

    msg.frame_information = reference_frame;
    msg.taskspace_trajectory_points.push_back(data);

    // publish the message
    chestTrajPublisher_.publish(msg);

}
