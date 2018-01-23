#include <tough_controller_interface/arm_control_interface.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_demo");
    ros::NodeHandle nh;
    std::string LOG_TOPIC = "/field/log";
    ros::Publisher log_pub = nh.advertise<std_msgs::String>(LOG_TOPIC, 10);
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


    ROS_INFO("Arm demo is about to start");
    ArmControlInterface armTraj(nh);

    //
    //Set the pose of the left arm to extend it to the front
    ArmControlInterface::armJointData l;
    l.side = LEFT;
    l.arm_pose = {1.57f, 1.2f, -1.57f, 0.0f, 0.0f, 0.0f, 0.0f};
    l.time = 2;

    // Set the pose of the right arm to extend it to the front
    ArmControlInterface::armJointData r;
    r.side = RIGHT;
    r.arm_pose = {-1.57f, 1.2f, 1.57f, 0.0f, 0.0f, 0.0f, 0.0f};
    r.time = 2;

    // Combine the left and right arm movements
    std::vector<ArmControlInterface::armJointData> hug_start;
    hug_start.push_back(r);
    hug_start.push_back(l);

    // Set the pose of the left arm to embrace
    ArmControlInterface::armJointData l2;
    l2.side = LEFT;
    l2.arm_pose = {1.57f, 1.2f, -1.57f, -1.1f, 0.0f, 0.0f, 0.0f};
    l2.time = 2;

    // Set the pose of the right arm to embrace
    ArmControlInterface::armJointData r2;
    r2.side = RIGHT;
    r2.arm_pose = {-1.57f, 1.2f, 1.57f, 1.1f, 0.0f, 0.0f, 0.0f};
    r2.time = 2;

    // Combine the left and right arm movements
    std::vector<ArmControlInterface::armJointData> hug_end;
    hug_end.push_back(r2);
    hug_end.push_back(l2);

    // Apply the first set of arm movements
    armTraj.moveArmJoints(hug_start);
    ros::Duration(2.5).sleep();

    // Finish with the last set of arm movements
    armTraj.moveArmJoints(hug_end);

    ros::Duration(2.5).sleep();

    // Move arms to specific points in space
    geometry_msgs::Pose right;
    right.position.x = 0.8;
    right.position.y = 0.4;
    right.position.z = 1.7;
    right.orientation.w = 1.0;

    geometry_msgs::Pose left;
    left.position.x = 2.0;
    left.position.y = 2.0;
    left.position.z = 1.5;
    left.orientation.w = 1.0;

    ArmControlInterface::armTaskSpaceData rts;
    rts.pose = right;
    rts.time = 2.0;
    rts.side = RIGHT;

    ArmControlInterface::armTaskSpaceData lts;
    lts.pose = left;
    lts.time = 2.0;
    lts.side = LEFT;

    std::vector<ArmControlInterface::armTaskSpaceData> ts;
    ts.push_back(rts);
    ts.push_back(lts);


    armTraj.moveArmInTaskSpace(ts);


    ros::spinOnce();
    ros::Duration(2).sleep();

    log_msg("Motion complete");
    return 0;
}

