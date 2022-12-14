/*
 * Copyright (c) 2022, Amos Smith (DFKI GmbH) and contributors
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of the the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include <ros/package.h>
#include <object_pose_msgs/ObjectList.h>
#include <vision_msgs/Detection3DArray.h>

class DopeConverter
{
    private:
    bool debug_;
    ros::NodeHandle *nh_;
    std::map<int,std::string> dope_ids_;
    ros::Subscriber dope_sub_;
    ros::Publisher converter_pub_;
    std::mutex connect_mutex_;

    public:
    DopeConverter(ros::NodeHandle *nh)
    {
        ros::NodeHandle pn("~");
        nh_ = nh;
        pn.param("debug", debug_, false);

        //load in dope class ids
        XmlRpc::XmlRpcValue v;
        pn.param("class_ids",v,v);
        ROS_ASSERT(v.getType()==XmlRpc::XmlRpcValue::TypeStruct);

        for(XmlRpc::XmlRpcValue::iterator i = v.begin(); i!=v.end(); ++i)
        {
            ROS_ASSERT(i->second.getType()==XmlRpc::XmlRpcValue::TypeInt);
            dope_ids_[i->second] = i->first;
        }

        if(debug_)
        {
            std::map<int,std::string>::iterator it;
            for(it=dope_ids_.begin(); it!=dope_ids_.end(); ++ it)
            {
                ROS_INFO_STREAM("Dope ID: " << it->first << " Label: " << it->second);
            }
        }

        //converted messages to be published
        std::lock_guard<std::mutex> lock(connect_mutex_);
        ros::SubscriberStatusCallback connect_cb = boost::bind(&DopeConverter::connectCb,this);
        converter_pub_ = nh->advertise<object_pose_msgs::ObjectList>("/dope_converter_poses",1000,connect_cb,connect_cb);
    }

    void dopeCallback(const vision_msgs::Detection3DArray::ConstPtr& msg)
    {
        object_pose_msgs::ObjectList converted_msg;

        converted_msg.header = msg->header;

        std::vector<object_pose_msgs::ObjectPose> converted_poses;

        for(auto i: msg->detections)
        {
            object_pose_msgs::ObjectPose new_pose;

            //search for object label based on dope ID
            std::map<int,std::string>::iterator itr = dope_ids_.find(i.results[0].id);

            if(itr != dope_ids_.end())
            {
                new_pose.class_id = itr->second;
            }else{
                ROS_ERROR_STREAM(i.results[0].id << " does not exist!");
            }

            //set instance id to zero for situations where only one instance per class is present
            ///TODO: Update this if necessary
            new_pose.instance_id = 1;

            //save object pose and bbox size
            new_pose.pose = i.results[0].pose.pose;
            new_pose.size = i.bbox.size;

            converted_poses.push_back(new_pose);
        }

        converted_msg.objects = converted_poses;

        converter_pub_.publish(converted_msg);
    }

    //Callback for connection/disconnections to DOPE converter output topic
    void connectCb()
    {
        if(debug_) ROS_INFO_STREAM("Dope Converter Connect Callback Called");

        std::lock_guard<std::mutex> lock(connect_mutex_);

        //If there are no subscribers to dope converter, then unsubscribe from DOPE messages
        if (converter_pub_.getNumSubscribers() == 0)
        {
            if(debug_) ROS_INFO_STREAM("No DOPE converter subscribers, shutting down DOPE subcriber");
            dope_sub_.shutdown();

        }else if (!dope_sub_)
        {
            if(debug_) ROS_INFO_STREAM("Starting up DOPE subscriber");
            dope_sub_ = nh_->subscribe("/dope_output",1,&DopeConverter::dopeCallback,this);
        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dope_to_object_pose_msgs_converter");
    ros::NodeHandle nh;

    DopeConverter node = DopeConverter(&nh);

    ros::spin();

    return 0;
}
