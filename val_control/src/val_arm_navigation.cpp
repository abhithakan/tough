#include<val_control/val_arm_navigation.h>
#include<stdlib.h>
#include<visualization_msgs/Marker.h>

int armTrajectory::arm_id = -1;

//add default pose for both arms. the values of joints are different.
armTrajectory::armTrajectory(ros::NodeHandle nh):nh_(nh),
    ZERO_POSE{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    DEFAULT_RIGHT_POSE{-0.2f, 1.2f, 0.7222f, 1.5101f, 0.0f, 0.0f, 0.0f},
    DEFAULT_LEFT_POSE{-0.2f, -1.2f, 0.7222f, -1.5101f, 0.0f, 0.0f, 0.0f},
    NUM_ARM_JOINTS(7){
    //tf_listener_ = new tf2_ros::TransformListener(this->tf_buffer_);
    armTrajectoryPublisher = nh_.advertise<ihmc_msgs::ArmTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/arm_trajectory", 1,true);
    handTrajectoryPublisher = nh_.advertise<ihmc_msgs::HandDesiredConfigurationRosMessage>("/ihmc_ros/valkyrie/control/hand_desired_configuration", 1,true);
    taskSpaceTrajectoryPublisher = nh_.advertise<ihmc_msgs::HandTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/hand_trajectory", 1, true);
    markerPub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1, true);

    //this->armTrajectorySubscriber = nh_.subscribe("/ihmc_ros/valkyrie/output/ha", 20,&ValkyrieWalker::footstepStatusCB, this);

}

armTrajectory::~armTrajectory(){
    //delete tf_listener_;
}


void armTrajectory::appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage &msg, float time, std::vector<float> pos)
{

    for (int i=0;i<NUM_ARM_JOINTS;i++)
    {
        ihmc_msgs::TrajectoryPoint1DRosMessage p;
        ihmc_msgs::OneDoFJointTrajectoryRosMessage t;
        t.trajectory_points.clear();

        p.time = time;
        p.position = pos[i];
        p.velocity = 0;
        p.unique_id = armTrajectory::arm_id;
        t.trajectory_points.push_back(p);
        t.unique_id = armTrajectory::arm_id;
        msg.joint_trajectory_messages[i].trajectory_points.push_back(p);
    }

    return;
}

void armTrajectory::moveToDefaultPose(armSide side)
{
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();

    arm_traj.joint_trajectory_messages.resize(NUM_ARM_JOINTS);
    arm_traj.robot_side = side;
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;
    if(side == RIGHT)
        appendTrajectoryPoint(arm_traj, 1, DEFAULT_RIGHT_POSE);
    else
        appendTrajectoryPoint(arm_traj, 1, DEFAULT_LEFT_POSE);

    armTrajectoryPublisher.publish(arm_traj);

}

void armTrajectory::moveToZeroPose(armSide side)
{
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();

    arm_traj.joint_trajectory_messages.resize(NUM_ARM_JOINTS);
    arm_traj.robot_side = side;
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;

    appendTrajectoryPoint(arm_traj, 2, ZERO_POSE);

    armTrajectoryPublisher.publish(arm_traj);

}

void armTrajectory::moveArmJoints(const armSide side, const std::vector<std::vector<float>> &arm_pose,const float time){

    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();


    arm_traj.joint_trajectory_messages.resize(NUM_ARM_JOINTS);
    arm_traj.robot_side = side;
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;
    for(auto i=arm_pose.begin(); i != arm_pose.end(); i++){
           if(i->size() != NUM_ARM_JOINTS)
           ROS_WARN("Check number of trajectory points");
        appendTrajectoryPoint(arm_traj, time/arm_pose.size(), *i);
    }

    armTrajectoryPublisher.publish(arm_traj);
}


void armTrajectory::moveArmJoints(std::vector<armJointData> &arm_data){

    ihmc_msgs::ArmTrajectoryRosMessage arm_traj_r;
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj_l;

    arm_traj_r.joint_trajectory_messages.clear();
    arm_traj_r.joint_trajectory_messages.resize(NUM_ARM_JOINTS);


    armTrajectory::arm_id--;
    arm_traj_r.unique_id = armTrajectory::arm_id;
    armTrajectory::arm_id--;
    arm_traj_l.joint_trajectory_messages.clear();
    arm_traj_l.joint_trajectory_messages.resize(NUM_ARM_JOINTS);
    arm_traj_l.unique_id = armTrajectory::arm_id;

    for(std::vector<armJointData>::iterator i=arm_data.begin(); i != arm_data.end(); i++){

        if(i->arm_pose.size() != NUM_ARM_JOINTS){
            ROS_INFO("Check number of trajectory points");
            return;
        }

        if(i->side == RIGHT){
            arm_traj_r.robot_side = i->side;
            appendTrajectoryPoint(arm_traj_r, i->time, i->arm_pose);
        }

        else {
            arm_traj_l.robot_side = i->side;
            appendTrajectoryPoint(arm_traj_l, i->time, i->arm_pose);
        }

    }

    armTrajectoryPublisher.publish(arm_traj_r);
    ros::Duration(0.02).sleep();
    armTrajectoryPublisher.publish(arm_traj_l);
}


void armTrajectory::moveArmMessage(const ihmc_msgs::ArmTrajectoryRosMessage &msg){
    this->armTrajectoryPublisher.publish(msg);
    armTrajectory::arm_id--;

}
int armTrajectory::getnumArmJoints() const
{
    return NUM_ARM_JOINTS;
}

void armTrajectory::closeHand(const armSide side)
{
    ihmc_msgs::HandDesiredConfigurationRosMessage msg;
    msg.robot_side = side;
    msg.hand_desired_configuration = msg.CLOSE;
    armTrajectory::arm_id--;
    msg.unique_id = armTrajectory::arm_id;
    this->handTrajectoryPublisher.publish(msg);
}

void armTrajectory::moveArmInTaskSpaceMessage(const armSide side, const ihmc_msgs::SE3TrajectoryPointRosMessage &point, int baseForControl)
{
    ihmc_msgs::HandTrajectoryRosMessage msg;
    msg.robot_side = side;
    msg.base_for_control = baseForControl;
    msg.taskspace_trajectory_points.push_back(point);
    msg.execution_mode = msg.OVERRIDE;
    armTrajectory::arm_id--;
    msg.unique_id = armTrajectory::arm_id;
    taskSpaceTrajectoryPublisher.publish(msg);
}

void armTrajectory::moveArmInTaskSpace(const armSide side, const geometry_msgs::Pose &pose, const float time)
{
  ihmc_msgs::SE3TrajectoryPointRosMessage point;
  poseToSE3TrajectoryPoint(pose, point);
  point.time = time;
  this->moveArmInTaskSpaceMessage(side, point);
  sleep(time+1);
  ROS_INFO("trying rectify");
  rectifyArmPosition(side,pose);
}

void armTrajectory::moveArmInTaskSpace(std::vector<armTaskSpaceData> &arm_data, int baseForControl)
{
  ihmc_msgs::HandTrajectoryRosMessage msg_l;
  ihmc_msgs::HandTrajectoryRosMessage msg_r;

  msg_l.taskspace_trajectory_points.clear();
  msg_r.taskspace_trajectory_points.clear();
  armTrajectory::arm_id--;
  msg_l.unique_id = armTrajectory::arm_id;
  msg_l.base_for_control = baseForControl;
  msg_l.execution_mode = msg_l.OVERRIDE;
  armTrajectory::arm_id--;
  msg_r.unique_id = armTrajectory::arm_id;
  msg_r.base_for_control = baseForControl;
  msg_r.execution_mode = msg_r.OVERRIDE;


  for(std::vector<armTaskSpaceData>::iterator i=arm_data.begin(); i != arm_data.end(); i++){

      if(i->side == RIGHT){
          msg_r.robot_side = i->side;
          ihmc_msgs::SE3TrajectoryPointRosMessage point;
          poseToSE3TrajectoryPoint(i->pose, point);
          point.time = i->time;
          msg_r.taskspace_trajectory_points.push_back(point);
      }

      else {
        msg_l.robot_side = i->side;
        ihmc_msgs::SE3TrajectoryPointRosMessage point;
        poseToSE3TrajectoryPoint(i->pose, point);
        point.time = i->time;
        msg_l.taskspace_trajectory_points.push_back(point);
      }

  }

  taskSpaceTrajectoryPublisher.publish(msg_r);
  ros::Duration(0.02).sleep();
  taskSpaceTrajectoryPublisher.publish(msg_l);
}

void armTrajectory::poseToSE3TrajectoryPoint(const geometry_msgs::Pose &pose, ihmc_msgs::SE3TrajectoryPointRosMessage &point)
{

  point.position.x = pose.position.x;
  point.position.y = pose.position.y;
  point.position.z = pose.position.z;
  point.orientation.w = pose.orientation.w;
  point.orientation.x = pose.orientation.x;
  point.orientation.y = pose.orientation.y;
  point.orientation.z = pose.orientation.z;
  point.unique_id = 255;
  return;
}

void armTrajectory::moveArmTrajectory(const armSide side, const trajectory_msgs::JointTrajectory &traj){

    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();

    arm_traj.joint_trajectory_messages.resize(NUM_ARM_JOINTS);
    arm_traj.robot_side = side;
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;

    for(auto i=traj.points.begin(); i < traj.points.end(); i++){
        appendTrajectoryPoint(arm_traj, *i);
    }
    ROS_INFO("Publishing Arm Trajectory");
    armTrajectoryPublisher.publish(arm_traj);
}

bool armTrajectory::nudgeArm(const armSide side, const direction drct, float nudgeStep){

    geometry_msgs::PoseStamped      world_values;
    geometry_msgs::PoseStamped      palm_values;

    world_values.header.frame_id=VAL_COMMON_NAMES::WORLD_TF;

    std::string target_frame = side == LEFT ? "/leftMiddleFingerPitch1Link" : "/rightMiddleFingerPitch1Link";

    try{
        tf::StampedTransform            tf_palm_values;
        tf_listener_.waitForTransform(VAL_COMMON_NAMES::PELVIS_TF,target_frame, ros::Time(0),ros::Duration(2));
        tf_listener_.lookupTransform(VAL_COMMON_NAMES::PELVIS_TF, target_frame, ros::Time(0),tf_palm_values);

        tf::pointTFToMsg(tf_palm_values.getOrigin(), palm_values.pose.position);
        tf::quaternionTFToMsg(tf_palm_values.getRotation(), palm_values.pose.orientation);
        palm_values.header.frame_id=VAL_COMMON_NAMES::PELVIS_TF;

    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }

    if     (drct == direction::LEFT)     palm_values.pose.position.y += nudgeStep;
    else if(drct == direction::RIGHT)    palm_values.pose.position.y -= nudgeStep;
    else if(drct == direction::UP)       palm_values.pose.position.z += nudgeStep;
    else if(drct == direction::DOWN)     palm_values.pose.position.z -= nudgeStep;
    else if(drct == direction::FRONT)    palm_values.pose.position.x += nudgeStep;
    else if(drct == direction::BACK)     palm_values.pose.position.x -= nudgeStep;

    try{
        tf_listener_.waitForTransform(VAL_COMMON_NAMES::PELVIS_TF,VAL_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
        tf_listener_.transformPose(VAL_COMMON_NAMES::WORLD_TF,palm_values,world_values);
    }
    catch (tf::TransformException ex) {
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }

    moveArmInTaskSpace(side,world_values.pose, 0.0f);
    return true;
}

bool armTrajectory::rectifyArmPosition(const armSide side, const geometry_msgs::Pose &pose, float threshold){

    ROS_INFO("In rectify");
    float x_error,y_error,z_error;
    short  x_count, y_count, z_count;
    x_count = 0;
    y_count = 0;
    z_count = 0;
    geometry_msgs::PoseStamped      palm_values;
    std::string target_frame = side == LEFT ? "/leftMiddleFingerPitch1Link" : "/rightMiddleFingerPitch1Link";

    do {
        ROS_INFO("Inside do loop");
        try{
            tf::StampedTransform            tf_palm_values;
            tf_listener_.waitForTransform(VAL_COMMON_NAMES::WORLD_TF,target_frame, ros::Time(0),ros::Duration(2));
            tf_listener_.lookupTransform(VAL_COMMON_NAMES::WORLD_TF, target_frame, ros::Time(0),tf_palm_values);

            tf::pointTFToMsg(tf_palm_values.getOrigin(), palm_values.pose.position);
            tf::quaternionTFToMsg(tf_palm_values.getRotation(), palm_values.pose.orientation);
            palm_values.header.frame_id=VAL_COMMON_NAMES::WORLD_TF;

        }
        catch (tf::TransformException ex){
            ROS_WARN("%s",ex.what());
            ros::spinOnce();
            return false;
        }

        x_error = std::abs(pose.position.x - palm_values.pose.position.x);
        ROS_INFO("X Error: %f",x_error);
        y_error = pose.position.y - palm_values.pose.position.y;
        ROS_INFO("Y Error: %f",y_error);
        z_error = pose.position.z - palm_values.pose.position.z;

        // Nudge to correct
        if (std::abs(y_error) > threshold && (y_count < 6)){
            ROS_INFO("Nudging in Y");
            if(pose.position.y > palm_values.pose.position.y) nudgeArm(side,direction::LEFT);
            else nudgeArm(side,direction::RIGHT);
            sleep(1);
            y_count++;
        }

       else if (std::abs(z_error) > threshold && z_count < 6){
            ROS_INFO("Nudging in Z");
            if(pose.position.z > palm_values.pose.position.z) nudgeArm(side,direction::UP);
            else nudgeArm(side,direction::DOWN);
            sleep(1);
            z_count++;
        }

        else if (std::abs(x_error) > threshold && x_count < 6){
            ROS_INFO("Nudging in X");
            if(pose.position.x > palm_values.pose.position.x) nudgeArm(side,direction::FRONT);
            else nudgeArm(side,direction::BACK);
            sleep(1);
            x_count++;
        }
        else return true;

    } while(((std::abs(x_error)>threshold) || (std::abs(y_error)>threshold) || (std::abs(z_error)>threshold)) && (x_count+y_count+z_count) < 14);
    return true;
}

void armTrajectory::appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage &msg, trajectory_msgs::JointTrajectoryPoint point)
{

    if(point.positions.size() != NUM_ARM_JOINTS) {
        ROS_WARN("Check number of trajectory points");
        return;
    }

    for (int i=0;i<NUM_ARM_JOINTS;i++)
    {
        ihmc_msgs::TrajectoryPoint1DRosMessage p;

        p.time = point.time_from_start.toSec();
        p.position = point.positions[i];
        p.velocity = point.velocities[i];
        p.unique_id = armTrajectory::arm_id;
        msg.joint_trajectory_messages[i].trajectory_points.push_back(p);
    }

    return;
}


