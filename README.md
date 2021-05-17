# Fetch-Simulation-Tutorial
A guideline for Fetch robot under the simulation and real-world.

# Fetch Calibration Error
While running Fetch calibration, it suddenly stopped and failed the calibration. The error messages shown up were like below:

~~~
[ INFO] [1621073971.856920297]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[ERROR] [1621073971.856978989]: Found empty JointState message
[ERROR] [1621073971.856997111]: Found empty JointState message
[ERROR] [1621073971.857124835]: Found empty JointState message
[ERROR] [1621073971.857315274]: Found empty JointState message
[ INFO] [1621073971.857459218]: No optimization objective specified, defaulting to PathLengthOptimizationObjective
[ INFO] [1621073971.857596089]: LBKPIECE1: Starting planning with 1 states already in datastructure
[ INFO] [1621073971.884570321]: LBKPIECE1: Created 86 (44 start + 42 goal) states in 71 cells (40 start (40 on boundary) + 31 goal (31 on boundary))
[ INFO] [1621073971.884625587]: Solution found in 0.027092 seconds
[ INFO] [1621073971.884946967]: SimpleSetup: Path simplification took 0.000279 seconds and changed from 61 to 2 states
[ERROR] [1621073971.886808351]: Computed path is not valid. Invalid states at index locations: [ 7 8 ] out of 31. Explanations follow in command line. Contacts are published on /move_group/display_contacts
[ INFO] [1621073971.886924074]: Found a contact between 'base_link' (type 'Robot link') and 'r_gripper_finger_link' (type 'Robot link'), which constitutes a collision. Contact information is not stored.
[ INFO] [1621073971.886945244]: Collision checking is considered complete (collision was found and 0 contacts are stored)
[ INFO] [1621073972.051865736]: Found a contact between 'base_link' (type 'Robot link') and 'r_gripper_finger_link' (type 'Robot link'), which constitutes a collision. Contact information is not stored.
[ INFO] [1621073972.051896884]: Collision checking is considered complete (collision was found and 0 contacts are stored)
[ERROR] [1621073972.052039221]: Completed listing of explanations for invalid states.
[ WARN] [1621073972.052525949]: Unable to move to desired state for sample 37.
~~~

It failed to carry out the calibration at the same step, especially, the sample 37 (in my case). 
- The initial plan was found without any collisions:
  > [ INFO] [1621073971.884625587]: Solution found in 0.027092 seconds
- However, it seems that it failed after the simplication.

I changed the joint velocity scaling factor from its default, 1.0, to 0.5. After this velocity reduction, it was able to calibrate Fetch successfully.
- Package "fetch_calibration" > open a launch file "calibrate.yaml" > set the value "default" as 0.5 at "velocity_factor"
