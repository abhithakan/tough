#include <tough_controller_interface/arm_control_interface.h>
#include "tough_controller_interface/wholebody_control_interface.h"
#include "tough_moveit_planners/tough_cartesian_planner.h"
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_nudgeOffset");
    ros::NodeHandle nh;
    ROS_INFO("Moving the arms");

    ros::Publisher log_pub = nh.advertise<std_msgs::String>(TOUGH_COMMON_NAMES::LOG_TOPIC, 10);
    const auto log_msg = [&log_pub](const std::string &str) {
        std_msgs::String msg;
        msg.data = ros::this_node::getName() + ": " + str;
        log_pub.publish(msg);
        ROS_INFO("%s", msg.data.c_str());
    };

    // wait a reasonable amount of time for the subscriber to connect
    ros::Time wait_until = ros::Time::now() + ros::Duration(0.5);
    while (log_pub.getNumSubscribers() == 0 && ros::Time::now() < wait_until) {
        ros::spinOnce();
        ros::WallDuration(0.1).sleep();
    }


    armTrajectory armTraj(nh);
    wholebodyManipulation wholeBodyController(nh);
    CartesianPlanner right_arm_planner(TOUGH_COMMON_NAMES::RIGHT_ENDEFFECTOR_GROUP, TOUGH_COMMON_NAMES::WORLD_TF);
    CartesianPlanner left_arm_planner(TOUGH_COMMON_NAMES::LEFT_ENDEFFECTOR_GROUP, TOUGH_COMMON_NAMES::WORLD_TF);
    moveit_msgs::RobotTrajectory traj;

    ros::Duration(1).sleep();
    armSide side;
    std::vector<geometry_msgs::Pose> waypoint;
    geometry_msgs::Pose pose;
    if(argc == 6){
        side = std::atoi(argv[1]) == 0 ? armSide::LEFT : armSide::RIGHT;
        if(std::atoi(argv[2]) ==0) armTraj.nudgeArmLocal(side, std::atof(argv[3]),std::atof(argv[4]),std::atof(argv[5]),pose);
        else armTraj.nudgeArmPelvis(side, std::atof(argv[3]),std::atof(argv[4]),std::atof(argv[5]),pose);

        waypoint.clear();
        waypoint.push_back(pose);
        if(side ==armSide::RIGHT)
        {
            if (right_arm_planner->getTrajFromCartPoints(waypoint, traj, false)> 0.98){
                log_msg("Moving right arm to given position");
                wholeBodyController.compileMsg(side, traj.joint_trajectory);
            }
        }
        else
        {
            if (left_arm_planner->getTrajFromCartPoints(waypoint, traj, false)> 0.98){
                log_msg("Moving left arm to given position");
                wholeBodyController.compileMsg(side, traj.joint_trajectory);
            }

        }


    }
    else
    {
        log_msg(" 0 - local | 1 - pelvis, invalid input");
    }
    ros::spinOnce();
    ros::Duration(2).sleep();

    log_msg("Motion complete");
    return 0;
}


