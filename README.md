# POSE SELECTOR

# TODO:
- Use ROS_ASSERT when reading in .yaml file (to ensure parameters are in correct structure)
- Use frame_id param to test if pose updates are in the correct frame
- Use user-defined file name for saving pose configuration (will need to modify update service)
- Update multiple poses with one service call (feed in arrays of object IDs and poses), instead of single update per service call.
- Update poses with no instance information (i.e. add can with pose xxx, pose_selector_node would define the instance id)
- Return instance IDs on class query (as additional service response member)
- Use header/timestamp info on poses?
- Test with mobipick simulator (test scenario needed)
- Update this readme, commit
