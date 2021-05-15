# Fetch-Simulation-Tutorial
A guideline for Fetch robot under the simulation and real-world.

# Fetch Calibration Error
While running Fetch calibration, it suddenly stopped and failed the calibration. The error messages shown up were like below:

~~~
[ INFO] [1621068966.879321279]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[ERROR] [1621068966.879418474]: Found empty JointState message
[ERROR] [1621068966.879436620]: Found empty JointState message
[ERROR] [1621068966.879533415]: Found empty JointState message
[ERROR] [1621068966.879721845]: Found empty JointState message
[ INFO] [1621068966.879856008]: No optimization objective specified, defaulting to PathLengthOptimizationObjective
[ INFO] [1621068966.879987411]: LBKPIECE1: Starting planning with 1 states already in datastructure
[ INFO] [1621068966.904600197]: LBKPIECE1: Created 99 (46 start + 53 goal) states in 81 cells (37 start (37 on boundary) + 44 goal (44 on boundary))
[ INFO] [1621068966.904665832]: Solution found in 0.024749 seconds
[ INFO] [1621068966.905007234]: SimpleSetup: Path simplification took 0.000302 seconds and changed from 41 to 2 states
[ERROR] [1621068975.369837941]: Failed to find features before using maximum iterations.
[ WARN] [1621068975.370207691]: led_finder failed to capture features.
[ WARN] [1621068975.370251266]: Failed to capture sample 34.
~~~

It failed to carry out its calibration at a certain step, especially, the sample 36 (in my case).

Additionally, there was a simple disconneting error between joystick and the mobile platform while conducting an arm tuck. Currently, restarting Fetch was a makeshift for resolving this problem. 
