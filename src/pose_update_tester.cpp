#include "ros/ros.h"
#include <pose_selector/PoseUpdate.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_update_tester");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<pose_selector::PoseUpdate>("pose_selector_update");
    pose_selector::PoseUpdate srv;

    pose_selector::LabeledPose pose_one;
    pose_one.class_id = "chicken";
    pose_one.instance_id = 24;
    pose_one.pose.header.frame_id = "chicken_frame";

    pose_selector::LabeledPose pose_two;
    pose_two.class_id = "dog";
    pose_two.pose.header.frame_id = "dog_frame";

    srv.request.poses.push_back(pose_one);
    srv.request.poses.push_back(pose_two);

    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }

    return 0;
}