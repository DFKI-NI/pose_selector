#include "ros/ros.h"
#include <pose_selector/PoseUpdate.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_update_tester");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<pose_selector::PoseUpdate>("pose_selector_update");
    pose_selector::PoseUpdate srv;

    pose_selector::ObjectPose pose_one;
    pose_one.label = "bottle_4";

    pose_selector::ObjectPose pose_two;
    pose_two.label = "can_6";
 
    srv.request.poses.objects.push_back(pose_one);
    srv.request.poses.objects.push_back(pose_two);

    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }

    return 0;
}