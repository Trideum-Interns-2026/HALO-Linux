# Movement
Lisa was here!

`follow_target.cpp` is a ROS 2 node that moves the PX4 vehicle toward the
detected target and then maintains a configurable separation. It uses the
detector's horizontal image position to move left or right and its estimated
distance to control forward speed. It does not command vertical motion or
change yaw.

The controller requires fresh position, distance, and camera-info messages.
It stops offboard control when the target data becomes stale or invalid. The
vehicle must already be armed and airborne; this program never arms or takes
off. The default speed limits are 0.8 m/s forward and 0.5 m/s laterally, and
the default following distance is 3 m. Tune these ROS parameters only in a
controlled simulation or test environment.

Start PX4/Gazebo and perception first, then run `move` in another container
terminal. Set `PERCEIVE_TARGET=person` in both the perception and movement
terminals to follow the unmarked person model. The default target is the red
sphere; `marked_person` uses the red-sphere detector on the marked person.