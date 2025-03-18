# ROS-Melodic-Fetch-Robot
I would like to document my progress in exploring Fetch simulations on ROS Melodic on Ubuntu 18.04 

### Chapter 0: Getting Started

- ROS (Robot Operating System): ROS is a framework that helps robots communicate internally. It uses a publish-subscribe model where "nodes" (small programs) talk to each other via "topics."
- Gazebo: This is a physics simulator that mimics the real world. The fetch_gazebo package loads Fetch’s 3D model, sensors, and physics properties.
- roslaunch: This command runs multiple ROS nodes at once, defined in the simulation.launch file. It starts Gazebo, spawns Fetch, and sets up communication.

#### Follow these steps to start basic simulation of Fetch Robot: 

0. Follow the ROS Melodic Tutorials for Installation: https://wiki.ros.org/melodic/Installation/Ubuntu

1. Verify ROS Melodic Installation:
```
source /opt/ros/melodic/setup.bash
roscore
```
2. Install required Dependencies:
```
sudo apt update
sudo apt install -y gazebo9 ros-melodic-gazebo-ros ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-msgs ros-melodic-gazebo-plugins
```
3. Check if gazebo runs without errors:
```
gazebo
```
4. Install Fetch simulation packages:
```
sudo apt install -y ros-melodic-fetch-gazebo ros-melodic-fetch-gazebo-demo
```
5. Launch Fetch Simulation:
```
roslaunch fetch_gazebo simulation.launch
```
```
roslaunch fetch_gazebo playground.launch
```
6. Visualize Robot in Rviz:
```
rosrun rviz rviz
```
In RViz:

Set the Fixed Frame to base_link (type it in the left panel under "Global Options").
Click Add (bottom left), select RobotModel, and click OK. This displays the Fetch robot’s model.
Add more displays (e.g., LaserScan or Image) to see sensor data if desired.

![test1](https://github.com/user-attachments/assets/ae6145d1-776d-4700-9c1a-4e7004bf651d)
![test2](https://github.com/user-attachments/assets/3c34ba9f-c0e8-4f36-822f-843724576898)


---

### Chapter 1: Understanding Fetch Simulation with ROS Basics


0. Start Fetch simulation
```
source /opt/ros/melodic/setup.bash
roslaunch fetch_gazebo simulation.launch
```

1. List Active Nodes and Topics:
```
rosnode list
rostopic list
```
Read data from topics:
```
rostopic echo /base_scan
```
2. Control the Robot using keyboard
```
sudo apt install ros-melodic-teleop-twist-keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
![fetch test](https://github.com/user-attachments/assets/310996b0-fad7-472e-a560-11c178b9e83d)


---

### Chapter 2: Writing a ROS Node to Control the Fetch Robot

You’ve successfully run the Fetch simulation (roslaunch fetch_gazebo simulation.launch)
You’re comfortable with the idea of topics (e.g., /cmd_vel for velocity commands)

Now let's try to create a simple **Python script (ROS node) to make the Fetch robot move in a Circle Pattern in Gazebo**.

0. Setup ROS Workspace:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
1. Create a ROS Package and Build it
```
cd ~/catkin_ws/src
catkin_create_pkg my_fetch_control roscpp rospy std_msgs geometry_msgs
```
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
2. Create the ROS Node:
Make a scripts folder
```
cd ~/catkin_ws/src/my_fetch_control
mkdir scripts
cd scripts
```
Write node code:
```
nano move_circle.py
```
```
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

def move_circle():
    # Initialize ROS node
    rospy.init_node('fetch_circle_mover', anonymous=True)
    
    # Create publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Waiting for subscribers...")
    rospy.sleep(2.0)  # Ensure connection to Fetch
    
    # Set loop rate
    rate = rospy.Rate(10)  # 10 Hz

    # Define circle command
    circle_cmd = Twist()
    circle_cmd.linear.x = 0.4  # Forward speed: 0.4 m/s
    circle_cmd.angular.z = 0.5  # Angular speed: 0.5 rad/s (clockwise)
    # Radius = 0.4 / 0.5 = 0.8 meters

    # Run in circles until shutdown
    while not rospy.is_shutdown():
        rospy.loginfo("Moving in a circle: linear.x = %.2f, angular.z = %.2f", 
                      circle_cmd.linear.x, circle_cmd.angular.z)
        pub.publish(circle_cmd)
        rate.sleep()

    # Stop when interrupted
    rospy.loginfo("Stopping")
    pub.publish(Twist())

if __name__ == '__main__':
    try:
        move_circle()
    except rospy.ROSInterruptException:
        pass
```
Build the workspace
```
chmod +x move_circle.py
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
How the code works: 
- Imports: We use rospy for ROS Python and Twist from geometry_msgs.msg for velocity commands.
- Node Initialization: rospy.init_node starts our node.
- Publisher: pub sends Twist messages to /cmd_vel.
- Twist Message:
- linear.x: Forward/backward speed (m/s).
- angular.z: Rotation speed (rad/s).
- Logic: Fetch robot moves in a clockwise circle with a radius of ~0.8 meters.

3. Test the script:
In one terminal run:
```
roslaunch fetch_gazebo simulation.launch
```
In a new terminal run:
```
cd ~/catkin_ws
source devel/setup.bash
rosrun my_fetch_control move_circle.py
```
4. To check the message description of Twist() msg:
```
rosmsg show geometry_msgs/Twist
```
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z

![test 3](https://github.com/user-attachments/assets/d80a03ad-dceb-4b31-b9fb-e94bb2943d0c)

---
### Chapter 3: Learning Topics Actions and Goals


1. Using Terminal to control the Robotic Arm:
```
rostopic pub -1 /arm_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal "{
  goal: {
    trajectory: {
      joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint'],
      points: [
        {
          positions: [0.0, 0.0, 0.0, 01.50, 0.0, 0.50, 0.0],
          velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {secs: 2, nsecs: 0}
        }
      ]
    }
  }
}"

```
2. Controlling the Gripper:

To open the gripper-
```
rostopic pub -1 /gripper_controller/gripper_action/goal control_msgs/GripperCommandActionGoal "goal: {command: {position: 0.1, max_effort: 50.0}}" 
```
To close the gripper-
```
rostopic pub -1 /gripper_controller/gripper_action/goal control_msgs/GripperCommandActionGoal "goal: {command: {position: 0.0, max_effort: 50.0}}"
```
3. To move the Fetch robot-
```
```
4. 



---
### Chapter 4: Controlling Fetch’s Arm and Gripper to Pick Up an Object

1. Understand Fetch’s Arm and Gripper

    - **Arm:** Fetch has a 7-DOF (degrees of freedom) arm with joints like shoulder_pan_joint, elbow_flex_joint, etc.
    - **Gripper:** Controlled via the gripper_controller with an effort (force) command.
    - **Topics:**
        Publishes joint positions (trajectory_msgs/JointTrajectory):
        ```
        /arm_controller/command
        ```
        Publishes gripper commands (control_msgs/GripperCommandActionGoal):
        ```
        /gripper_controller/gripper_action/goal
        ```
2. Spawn an Object in Gazebo

```
cd ~/catkin_ws/src/my_fetch_controll
mkdir -p models/my_box
nano models/my_box/model.sdf
```
```
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="my_box">
    <static>false</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box><size>0.1 0.1 0.1</size></box>
        </geometry>
        <surface>
          <friction>
            <ode><mu>0.6</mu><mu2>0.6</mu2></ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.1 0.1 0.1</size></box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient> <!-- Red color -->
        </material>
      </visual>
      <inertial>
        <mass>0.1</mass> <!-- 100 grams -->
        <inertia>
          <ixx>0.000166</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.000166</iyy><iyz>0</iyz><izz>0.000166</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
```
```
nano models/my_box/model.config
```
```
<?xml version="1.0"?>
<model>
  <name>my_box</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your@email.com</email>
  </author>
  <description>A simple 10cm red box for Fetch to pick up</description>
</model>
```
Edit Package.xml : 

```
nano ~/catkin_ws/src/my_fetch_control/package.xml
```
Example-
```
<package>
  ...
  <export>
    <gazebo_ros gazebo_model_path="${prefix}/models"/>
  </export>
</package>
```
Build the Workspace:
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

To check model path:
```
echo $GAZEBO_MODEL_PATH
```

3. We’ll create a script to Pick up the Object

    - Move the arm to a “reach” position above the box.
    - Close the gripper to grab it.
    - Lift the arm with the box.

    Save this as pick_object.py in ~/catkin_ws/src/my_fetch_control/scripts/:
    ```
    cd ~/catkin_ws/src/my_fetch_control/scripts/
    nano pick_object.py
    ```
    Paste the following code:
    ```
    #!/usr/bin/env python
    # -*- coding: utf-8 -*-
    
    import rospy
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from control_msgs.msg import GripperCommandActionGoal
    
    def pick_object():
        # Initialize ROS node
        rospy.init_node('fetch_pick_object', anonymous=True)
        
        # Publishers
        arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
        gripper_pub = rospy.Publisher('/gripper_controller/gripper_action/goal', 
                                     GripperCommandActionGoal, queue_size=10)
        
        rospy.loginfo("Waiting for subscribers...")
        rospy.sleep(2.0)  # Ensure connections
    
        rate = rospy.Rate(10)
    
        # Joint names for Fetch's arm
        joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'upperarm_roll_joint',
            'elbow_flex_joint',
            'forearm_roll_joint',
            'wrist_flex_joint',
            'wrist_roll_joint'
        ]
    
        # Step 1: Move arm to reach position (above box at x=0.5, y=0, z=0)
        reach_traj = JointTrajectory()
        reach_traj.joint_names = joint_names
        reach_point = JointTrajectoryPoint()
        reach_point.positions = [0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0]  # Rough reaching pose
        reach_point.time_from_start = rospy.Duration(2.0)  # Reach in 2 seconds
        reach_traj.points.append(reach_point)
    
        rospy.loginfo("Reaching for object...")
        arm_pub.publish(reach_traj)
        rospy.sleep(2.5)  # Wait for arm to move
    
        # Step 2: Close gripper
        gripper_goal = GripperCommandActionGoal()
        gripper_goal.goal.command.position = 0.0  # Fully closed (meters)
        gripper_goal.goal.command.max_effort = 50.0  # Force to grip
        rospy.loginfo("Closing gripper...")
        gripper_pub.publish(gripper_goal)
        rospy.sleep(1.0)  # Wait for gripper to close
    
        # Step 3: Lift arm
        lift_traj = JointTrajectory()
        lift_traj.joint_names = joint_names
        lift_point = JointTrajectoryPoint()
        lift_point.positions = [0.0, 0.8, 0.0, 1.2, 0.0, 0.5, 0.0]  # Lift higher
        lift_point.time_from_start = rospy.Duration(2.0)
        lift_traj.points.append(lift_point)
    
        rospy.loginfo("Lifting object...")
        arm_pub.publish(lift_traj)
        rospy.sleep(2.5)  # Wait for lift
    
        rospy.loginfo("Done! Holding position...")
        rospy.spin()
    
    if __name__ == '__main__':
        try:
            pick_object()
        except rospy.ROSInterruptException:
            pass
    ```
    ```
    chmod +x ~/catkin_ws/src/my_fetch_control/scripts/pick_object.py
    ```
    
4. Test the script: 

Start Simulation:
```
roslaunch fetch_gazebo simulation.launch
```
Spawn Box:
```
rosrun gazebo_ros spawn_model -sdf -model my_box -x 0.5 -y 0.0 -z 0.0 -file ~/catkin_ws/src/my_fetch_control/models/my_box/model.sdf
```
Run Script:
```
rosrun my_fetch_control pick_object.py
```
**Expected Outcome**
- Fetch’s arm extends toward the box.
- Gripper closes, hopefully grabbing the box.
- Arm lifts, carrying the box upward.

![test no](https://github.com/user-attachments/assets/ad7782de-fddc-41b9-badc-4c0b0013633b)


(need to modify joint values as gripper is not reaching the object location)







