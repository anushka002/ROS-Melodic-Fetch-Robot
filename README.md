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
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
![test4](https://github.com/user-attachments/assets/73f5103f-5e27-4f69-8aef-43f7daf64b4c)

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

To reset the simultion if Fetch is stuck or collides:
```
rosservice call /gazebo/reset_simulation "{}"
```

---
### Chapter 5: Perception with Fetch’s Head Camera

Use Fetch’s RGB camera (/head_camera/rgb/image_raw) to detect the red box we created and estimate its position relative to Fetch. 
We’ll use OpenCV for simple color-based detection.

1. To install OpenCV:
```
sudo apt-get install python-opencv
python -c "import cv2; print(cv2.__version__)" # Verify installation
```
2. Open Gazebo simulation and Spawn Box:
```
roslaunch fetch_gazebo simulation.launch
```
```
rosrun gazebo_ros spawn_model -sdf -model my_box_0 -x 0.8 -y 0.0 -z 0.0 -file ~/catkin_ws/src/my_fetch_control/models/my_box/model.sdf # close to fetch
rosrun gazebo_ros spawn_model -sdf -model my_box_1 -x 8.0 -y 0.0 -z 0.0 -file ~/catkin_ws/src/my_fetch_control/models/my_box/model.sdf # far from fetch
```
3. Understand Topics for Fetch's Camera

- Topic: /head_camera/rgb/image_raw
- Message Type: sensor_msgs/Image
- Details: Fetch’s head camera outputs RGB images.
- Goal: Process these images to find the red box’s centroid (x, y in image space).

4. Write the Detection Script
    We’ll create detect_box.py to:
    
    - Subscribe to the camera feed.
    - Convert ROS images to OpenCV format.
    - Detect the red box by color.
    - Output the box’s pixel coordinates.
    ```
    cd ~/catkin_ws/src/my_fetch_control/scripts/
    touch detect_box.py
    nano detec_box.py
    ```
    ```
    #!/usr/bin/env python
    # -*- coding: utf-8 -*-
    
    import rospy
    import cv2
    import numpy as np
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    
    def image_callback(msg):
        # Convert ROS Image message to OpenCV format
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    
        # Convert BGR to HSV for color detection
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
        # Define red color range in HSV
        lower_red1 = np.array([0, 120, 70])    # Lower bound for red (Hue 0-10)
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])  # Upper bound for red (Hue 170-180)
        upper_red2 = np.array([180, 255, 255])
    
        # Create masks for red
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2  # Combine both red ranges
    
        # Find contours (handle OpenCV 3.x return values)
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Get the largest contour (assume it’s the box)
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:  # Filter small noise
                # Calculate centroid
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    rospy.loginfo("Box centroid: (x=%d, y=%d)", cX, cY)
    
                    # Draw centroid on image for visualization
                    cv2.circle(cv_image, (cX, cY), 5, (0, 255, 0), -1)
        
        # Show the image (optional, for debugging)
        cv2.imshow("Camera Feed", cv_image)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)
    
    def detect_box():
        rospy.init_node('box_detector', anonymous=True)
        rospy.Subscriber('/head_camera/rgb/image_raw', Image, image_callback)
        rospy.loginfo("Starting box detection...")
        rospy.spin()
        cv2.destroyAllWindows()
    
    if __name__ == '__main__':
        try:
            detect_box()
        except rospy.ROSInterruptException:
            pass
    ```
    Make it executable:
    ```
    chmod +x ~/catkin_ws/src/my_fetch_control/scripts/detect_box.py
    ```
5. Run detection:
```
rosrun my_fetch_control detect_box.py
```
Output (Detecting Box 2):
![test5](https://github.com/user-attachments/assets/8ed6f3be-46cc-4a1a-a002-b30e7ba3a178)

6. Move Fetch's head to Visualize the closer box:
```
rostopic pub /head_controller/point_head/goal control_msgs/PointHeadActionGoal "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'base_link'
goal_id: {stamp: {secs: 0, nsecs: 0}, id: ''}
goal:
  target:
    header:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: 'base_link'
    point: {x: 0.5, y: 0.0, z: 0.0}
  pointing_frame: 'head_tilt_link'
  pointing_axis: {x: 1.0, y: 0.0, z: 0.0}
  min_duration: {secs: 1, nsecs: 0}
  max_velocity: 1.0" -1
```
Output: (Detecting Box 1)
![test6](https://github.com/user-attachments/assets/6e0afda1-07b1-4aa9-9660-ad0ff4c49410)

7. Summary
- Camera Topic: Used /head_camera/rgb/image_raw for RGB images from Fetch’s head camera.
- OpenCV Integration: Processed images with OpenCV to detect the red box.
- Color Detection: Converted BGR to HSV, defined red ranges (Hue 0–10, 170–180), and created a mask.
- Contour Finding: Identified the box’s contour and calculated its centroid (cX, cY) in pixel coordinates.
- ROS-OpenCV Bridge: Used CvBridge to convert ROS Image messages to OpenCV format.
- Head Control: Adjusted Fetch’s head with /head_controller/point_head/goal to aim at the box (tilt ~1.18 rad, 67°).
- Visualization: Displayed camera feed and mask with OpenCV windows, marked centroid with a green dot.
- Output: Logged box centroid (e.g., Box centroid: (x=320, y=240)).

---

### Chapter 6: Advanced Manipulation with MoveIt!

