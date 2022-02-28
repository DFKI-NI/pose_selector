# Pose_selector

This package provides functionality for storing, querying, and modifying pose information of objects in simulation or real-world applications of the Mobipick robot. 

## Messages   

---

The pose selector consists of two message types, ObjectPose and ObjectList (see below). ObjectPose contains a label string, pose, and vectors representing the object size, minimum points, and maximum points (for bounding box). The ObjectList contains header information, an array of ObjectPoses, and a reference pose for all poses in the ObjectList (i.e. this reference pose could be the global pose of the camera, and each ObjectPose is provided in reference to the camera pose).

---

**ObjectPose.msg**
 
    string label  //label of object, e.g. 'screwdriver_1'
    geometry_msgs/Pose pose //pose of object
    geometry_msgs/Vector3 size //size of object
    geometry_msgs/Vector3 min //minimum points of object bounding box
    geometry_msgs/Vector3 max //maximum points of object bounding box

---

**ObjectList.msg**
 
    std_msgs/Header header //header information
    ObjectPose[] objects //array of ObjectPoses
    geometry_msgs/Pose reference_pose //Reference pose for all ObjectPoses in objects


## Services

---

**ClassQuery.srv**

Given a class type as a request, this service will return arrays of ObjectPoses corresponding to all instances of the requested class type.

request: 

    string class_id

response:

    ObjectPose[] poses

---

**ConfigSave.srv**

This service saves all current pose information as file_name.yaml in the /config directory.

request:

    string file_name

response:

    None

---

**PoseDelete.srv**

This service deletes a pose corresponding to the class and instance id provided in the request.

request:

    string class_id
    int64 instance_id

response:

    None

---

**PoseQuery.srv**

This service returns the pose information of the object associated with the requested class and instance IDs.

request:

    string class_id
    int64 instance_id

response:

    ObjectPose pose_query_result

---

**PoseUpdate.srv**

This service updates the current pose information (either creating new entries or updating existing ones) with the poses in the service request.

request:

    ObjectList[] poses 

response:

    None

---

## Nodes

---

### pose_selector_node

The pose_selector_node node provides functionalities to create, update, delete, and save pose information of objects.

**Service Servers**

* **pose_selector_query** (pose_selector::PoseQuery)
* **pose_selector_class_query** (pose_selector::ClassQuery)
* **pose_selector_update** (pose_selector::PoseUpdate)
* **pose_selector_delete** (pose_selector::PoseDelete)
* **pose_selector_save** (pose_selector::ConfigSave)
* **pose_selector_activate** (std_srvs::SetBool)

**Subscribers**

* **pose_sub** (pose_selector::ObjectList)

**Launch Files**

* `test_node.launch`: launches the node with configuration parameters

**Configuration Files**

* `test_config.yaml`: testing configuration file

**Configuration Parameters**

* **debug** (bool default: false, set to true to turn on debug messages)
* **poses** (struct, setup for pose storage. See configuration files for examples)
* **poses/\<name\>/x** (double, object position on x-axis)
* **poses/\<name\>/y** (double, object position on y-axis)
* **poses/\<name\>/z** (double, object position on z-axis)
* **poses/\<name\>/rx** (double, object orientation quaternion x value)
* **poses/\<name\>/ry** (double, object orientation quaternion y value)
* **poses/\<name\>/rz** (double, object orientation quaternion z value)
* **poses/\<name\>/rw** (double, object orientation quaternion w value)
* **poses/\<name\>/size_x** (double, object size on x-axis)
* **poses/\<name\>/size_y** (double, object size on y-axis)
* **poses/\<name\>/size_z** (double, object size on z-axis)
* **poses/\<name\>/min_x** (double, object's bounding box minimum position on x-axis)
* **poses/\<name\>/min_y** (double, object's bounding box minimum position on y-axis)
* **poses/\<name\>/min_z** (double, object's bounding box minimum position on z-axis)
* **poses/\<name\>/max_x** (double, object's bounding box maximum position on x-axis)
* **poses/\<name\>/max_y** (double, object's bounding box maximum position on y-axis)
* **poses/\<name\>/max_z** (double, object's bounding box maximum position on z-axis)

---

### pose_update_tester

This node can be used to test the update functionalities of the pose_selector_node.

**Service Clients**

* **pose_selector_update** (pose_selector::PoseUpdate)

---

## Example Usage

In one terminal, launch the pose_selector_node by executing the following:

```
source devel/setup.bash
roslaunch pose_selector test_node.launch
```

If `debug` parameter is set to `True`, then you should be presented with the current pose information obtained from the configuration file.

While the pose_selector_node is running, execute the following in a new terminal:

```
source devel/setup.bash
rosrun pose_selector update_test
```

After running update_test, you should see an updated list of poses in the pose_selector terminal.

