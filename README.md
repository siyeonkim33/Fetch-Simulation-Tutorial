# Fetch-Simulation-Tutorial
A guideline for Fetch robot under the simulation and real-world.

# Fetch Calibration Error
While running Fetch calibration, it suddenly stopped and failed the calibration. The error messages shown up were like below:

~~~
[ INFO] [1620895638.889939885]: Solution found in 0.034940 seconds
[ INFO] [1620895638.890302669]: SimpleSetup: Path simplification took 0.000332 seconds and changed from 38 to 2 states
[ERROR] [1620895638.892085254]: Computed path is not valid. Invalid states at index locations: [ 7 8 ] out of 31. Explanations follow in command line. Contacts are published on /move_group/display_contacts
###[ INFO] [1620895638.892168182]: Found a contact between 'base_link' (type 'Robot link') and 'r_gripper_finger_link' (type 'Robot link'), which constitutes a collision. Contact information is not stored.
[ INFO] [1620895638.892188698]: Collision checking is considered complete (collision was found and 0 contacts are stored)
[ INFO] [1620895639.058072629]: Found a contact between 'base_link' (type 'Robot link') and 'r_gripper_finger_link' (type 'Robot link'), which constitutes a collision. Contact information is not stored.
[ INFO] [1620895639.058102328]: Collision checking is considered complete (collision was found and 0 contacts are stored)
[ERROR] [1620895639.058198517]: Completed listing of explanations for invalid states.
[ WARN] [1620895639.058547961]: Unable to move to desired state for sample 37.
[ INFO] [1620895639.058781775]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[ERROR] [1620895639.058824946]: Found empty JointState message
[ERROR] [1620895639.058841275]: Found empty JointState message
[ERROR] [1620895639.058906098]: Found empty JointState message
[ERROR] [1620895639.059091341]: Found empty JointState message
[ INFO] [1620895639.059222675]: No optimization objective specified, defaulting to PathLengthOptimizationObjective
[ INFO] [1620895639.059348754]: LBKPIECE1: Starting planning with 1 states already in datastructure
[ INFO] [1620895639.088336496]: LBKPIECE1: Created 128 (72 start + 56 goal) states in 114 cells (71 start (71 on boundary) + 43 goal (43 on boundary))
[ INFO] [1620895639.088391980]: Solution found in 0.029098 seconds
[ INFO] [1620895639.088727269]: SimpleSetup: Path simplification took 0.000298 seconds and changed from 42 to 2 states
~~~
