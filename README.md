# Pose_selector

This package provides functionality for storing, querying, and modifying pose information of objects in simulation or real-world applications of the Mobipick robot. 

## Messages   

---

**LabeledPose.msg**
 
    string class_id  //class of object, e.g. 'screwdriver'
    uint32 instance_id //instance of object, e.g. '2' for 2nd instance
    geometry_msgs/PoseStamped pose //pose of object

---

## Services

---

**ClassQuery.srv**

Given a class type as a request, this service will return arrays of object instance IDs and poses corresponding to all instances of the requested class type.

request: 

    string class_id

response:

    uint32[] instance_ids
    geometry_msgs/PoseStamped[] poses

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

    geometry_msgs/PoseStamped pose_query_result

---

**PoseUpdate.srv**

This service updates the current pose information (either creating new entries or updating existing ones) with the poses in the service request.

request:

    LabeledPose[] poses 

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

**Launch Files**

* `test_node.launch`: launches the node with configuration parameters

**Configuration Files**

* `test_config.yaml`: testing configuration file

**Configuration Parameters**

* **debug** (bool default: false, set to true to turn on debug messages)
* **frame_id** (string default: map, tf frame that all poses should be in)
* **poses** (struct, setup for pose storage. See configuration files for examples)
* **poses/\<name\>/class** (string, object class id)
* **poses/\<name\>/instance** (int, object instance id)
* **poses/\<name\>/x** (double, object position on x-axis)
* **poses/\<name\>/y** (double, object position on y-axis)
* **poses/\<name\>/z** (double, object position on z-axis)
* **poses/\<name\>/rx** (double, object orientation quaternion x value)
* **poses/\<name\>/ry** (double, object orientation quaternion y value)
* **poses/\<name\>/rz** (double, object orientation quaternion z value)
* **poses/\<name\>/rw** (double, object orientation quaternion w value)

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

**TODO**: More examples
