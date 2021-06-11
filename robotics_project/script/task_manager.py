#!/usr/bin/env python

from __future__ import print_function
from six.moves import input

import sys
import copy
import time, math
import rospy
import actionlib
import moveit_commander
from moveit_commander.exception import MoveItCommanderException
import moveit_msgs.msg
import geometry_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv import Empty

try:
  from math import pi, tau, dist, fabs, cos
except: # For Python 2 compatibility
  from math import pi, fabs, cos, sqrt
  tau = 2.0*pi
  def dist(p, q):
    return sqrt(sum((p_i - q_i)**2.0 for p_i, q_i in zip(p,q)))
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import numpy as np
import csv
import os
from os.path import join


#=====================================
#======    PARAMETER
#=====================================
LINEAR_VEL = 0.11
ANGULAR_VEL = 0.3
STOP_DISTANCE = 0.35
LIDAR_ERROR = 0.015
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

#=====================================
#======    Messages
#=====================================
project_info_msg = """
----------------------------------------------------------
2021 Spring Semester Robotics Term Project
----------------------------------------------------------
Project Description
1. Initialize Robot
2. Get Target Object Information from Customer
3. Move to Storage Box
4. Find Target Object
5. Pick up Target Object
6. Move to Customer
7. Place Object to Customer Hand
*Repeat 2~7 Process Until Program Down

Press Ctrl-D to exit at any time
    
"""
localization_info_msg = """
=====================================
Information for Localization
=====================================
1. Go Forward until Minimum Distance Reached
2. Rotate Predefined Angle
*repeat 1~2 process for +180, +90, -180, -90

"""
target_info_msg = """
Press Matched key for Each Target Object 

object_1 : "q"
object_2 : "w"
object_3 : "e"

"""
TARGET_INFO = {
  "q": "object_1",
  "w": "object_2",
  "e": "object_3"
}

STORAGE_BOX_POSITION = [0, -1.75, 270] # x, y, angle(GAZEBO)
# STORAGE_BOX_POSITION = [0, 0, 135] # x, y, angle (REAL)

SCRIPT_ROOT = os.path.dirname(os.path.abspath(__file__))
XZ_WORKSPACE_FILE = join(SCRIPT_ROOT, "available_workspace.csv")

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if the values in two lists are within a tolerance of each other.
  For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle 
  between the identical orientations q and -q is calculated correctly).
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
    x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
    # Euclidean distance
    d = dist((x1, y1, z1), (x0, y0, z0))
    # phi = angle between orientations
    cos_phi_half = fabs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)
    return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

  return True

class TurtleState:
  pose = Pose()
  rpy = [0,0,0]
  is_stop = True

class TargetBox:
  box_1 = 1
  box_2 = 2
  box_3 = 3
  box_4 = 4

class GraspState:
  upper_box_grasp = [0.3, 0, 0.3] # x, y, z
  down_box_grasp = [0.3, 0, 0]
  
  left_box_approach = [0.2, 0.2]
  right_box_approach = [0.2, -0.2]




class RobotManager(object):
  def __init__(self, localization=True):
    super(RobotManager, self).__init__()
    
    moveit_commander.roscpp_initialize(sys.argv)
    

    #=====================================
    #======    Loading Manipulation
    #=====================================
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    arm_group = moveit_commander.MoveGroupCommander("arm")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = arm_group.get_planning_frame()
    print("============ Arm Planning frame: %s" % planning_frame)
    # We can also print the name of the end-effector link for this group:
    eef_link = arm_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)
    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")                         
    
    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.arm_group = arm_group
    self.gripper_group = gripper_group
    
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

    self.available_workspace = self.get_available_workspace()
    self.go_to_home_pose()

    #=====================================
    #======    Loading Turtelbot 
    #=====================================
    self.turtle_state = TurtleState()

    # topic publisher
    self.turtle_cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # action client
    self.turtle_move_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    
    if localization==True:
      self.localization()
    else:
      pass
  #region Manipulator
  def get_available_workspace(self):
    available_space = []
    with open(XZ_WORKSPACE_FILE, "r") as f:
      reader = csv.reader(f)
      for row in reader:
        available_space.append(map(float, row))
    available_space = np.array(available_space)
    
    return available_space

  def go_to_home_pose(self):
    self.arm_group.set_named_target("home")
    self.arm_group.go(wait=True)
    self.arm_group.stop()
    self.arm_group.clear_pose_targets()

    print("============ Printing Home State of Robot")
    print(self.robot.get_current_state())
    print("")              

  def go_to_joint_state(self, target_joint_state):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    arm_group = self.arm_group
    print("============ Printing current joint state")
    print(arm_group.get_current_joint_values())
    print("")
    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
    ## thing we want to do is move it to a slightly better configuration. 
    ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
    # We get the joint values from the group and change some of the values:
    joint_goal = arm_group.get_current_joint_values()
    
    for i in range(4):
      joint_goal[i] = target_joint_state[i]

    joint_goal[0] = 0
    joint_goal[1] = -tau/8
    joint_goal[2] = 0
    joint_goal[3] = -tau/4

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    arm_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    arm_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = arm_group.get_current_joint_values()
    print("============ Printing after joint state")
    print(current_joints)
    print("")

    return all_close(joint_goal, current_joints, 0.01)

  def go_to_xyz_goal(self, target_xyz):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    arm_group = self.arm_group
    print("============ Printing current eef pose")
    current_pose = arm_group.get_current_pose().pose
    print(current_pose)
    print("")

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    
    print("============ Printing target eef xyz")
    print(target_xyz)
    print("")
    arm_group.set_position_target(target_xyz)

    ## Now, we call the planner to compute the plan and execute it.
    plan = arm_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    arm_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    arm_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = arm_group.get_current_pose().pose
    print("============ Printing after eef pose")
    print(current_pose)
    print("")

  def go_to_xyz_goal_with_constraint(self, target_xyz):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    arm_group = self.arm_group
    print("============ Printing current eef pose")
    current_pose = arm_group.get_current_pose().pose
    print(current_pose)

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    target_pose = geometry_msgs.msg.Pose()
    target_pose.orientation.x = 0
    target_pose.orientation.y = 0
    target_pose.orientation.z = 0
    target_pose.orientation.w = 1.0
    target_pose.position.x = target_xyz[0]
    target_pose.position.y = target_xyz[1]
    target_pose.position.z = target_xyz[2]

    print("============ Printing target eef pose")
    print(target_pose)
    arm_group.set_pose_target(target_pose)

    ## Now, we call the planner to compute the plan and execute it.
    plan = arm_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    arm_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    arm_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = arm_group.get_current_pose().pose
    print("============ Printing after eef pose")
    print(current_pose)
    print("")

    return all_close(target_pose, current_pose, 0.01)

  def get_nearest_available_point(self, target_point):
    print("============ Printing Nearest Point for Target Point: ")
    print("Target Point: ")
    print("x: {}\ny: {}\nz: {}\n".format(*target_point))

    dif_vec = self.available_workspace - target_point
    dif_distance = map(np.linalg.norm, dif_vec)
    min_idx = np.argmin(dif_distance)
    nearest_point = self.available_workspace[min_idx]
    difference_vector = dif_vec[min_idx]
    print("Nearest Point: ")
    print("x: {}\ny: {}\nz: {}\n".format(*nearest_point))

    print("Difference Vector: ")
    print("x: {}\ny: {}\nz: {}\n".format(*difference_vector))

    return nearest_point, difference_vector

  def open_gripper(self):
    print("Open Gripper!\n")
    self.gripper_group.go([0.01]*2, wait=True)
    self.gripper_group.stop()
  
  def close_gripper(self):
    print("Close Gripper!\n")
    self.gripper_group.go([-0.01]*2, wait=True)
    self.gripper_group.stop()
  
  def _calculate_xz_configuration(self):
    available_space = []
    with open("test2.csv", "r") as f:
      reader = csv.reader(f)
      print(type(reader))
      for row in reader:
        available_space.append(map(float, row))
    available_space = np.array(available_space)
    min_x = np.min(available_space[:,0])
    max_x = np.max(available_space[:,0])
    min_z = np.min(available_space[:,2])
    max_z = np.max(available_space[:,2])
    print("min_x: {}".format(min_x))
    print("max_x: {}".format(max_x))
    print("min_z: {}".format(min_z))
    print("max_z: {}".format(max_z))
    
    # test1 step = 0.05
    # test2 step
    # x: 0.0327777777778
    # z: 0.0377777777778
    # test3 step
    # x: 0.00810339506173
    # z: 0.00923456790124
    before_x = 0.0327777777778
    before_z = 0.0377777777778
    points_num = 40
    print((max_x + before_x - min_x)/points_num)
    print((max_z + before_z * 2 - min_z)/points_num)
    x_space = np.linspace(min_x, max_x + before_x , points_num)
    z_space = np.linspace(min_z - before_z, max_z + before_z, points_num)
    new_available_space = []
    for x in x_space:
      for z in z_space:
        goal_pose =[x,
                    0,
                    z]
        ret = self.go_to_xyz_goal_with_constraint(goal_pose)
        print(ret)
        if ret:
          new_available_space.append(goal_pose)
        else:
          pass
    
    with open("test3.csv", "w") as f:
      write = csv.writer(f)
      write.writerows(new_available_space)
  #endregion

  #region Turtlebot
  def localization(self):
    print(localization_info_msg)
    rospy.wait_for_service("global_localization",timeout=2.0)
    GL=rospy.ServiceProxy("global_localization",Empty)
    
    
    GL()
    
    angle_list = [100, -100, 50, -50]
    for angle in angle_list:
      # print("Try Angle {}".format(angle))
      self.turtle_stop()
      # Go Forward
      t0 = time.time()
      t1 = time.time()
      lidar_distances = self.get_scan_data()
      min_distance = min(lidar_distances)
      if min_distance > SAFE_STOP_DISTANCE:
        self.set_turtlebot_velocity(linear_velocity=[LINEAR_VEL,0,0],
                                    angular_velocity=[0,0,0])
        while (t1 - t0) < 5:
          t1 = time.time()
          lidar_distances = self.get_scan_data()
          min_distance = min(lidar_distances)
          if min_distance < SAFE_STOP_DISTANCE:
            self.set_turtlebot_velocity(linear_velocity=[0,0,0],
                                        angular_velocity=[0,0,0])
            break
      else:
        pass
      self.turtle_stop()
      # Rotate angle
      self.turtle_state_update()
      start_rad = self.turtle_state.rpy[2]
      delta_rad = 0
      objective_rad = angle * np.pi / 180
      if objective_rad > 0:
        angular_vel = ANGULAR_VEL
      elif objective_rad < 0:
        angular_vel = -ANGULAR_VEL
      else:
        angular_vel = 0
      self.set_turtlebot_velocity(linear_velocity=[0,0,0],
                                  angular_velocity=[0,0,angular_vel])
      while True:
        self.turtle_state_update()
        delta_rad = self.turtle_state.rpy[2] - start_rad
        if abs(delta_rad) > abs(objective_rad):
          break
      self.turtle_stop()
    self.turtle_stop()
    
    rospy.wait_for_service("/move_base/clear_costmaps",timeout=2.0)
    MC=rospy.ServiceProxy("/move_base/clear_costmaps",Empty)
    MC()
    
    print('End initialize')

  def move_to_goal(self, goal_x, goal_y, goal_angle, relate_frame="map"):
    self.turtle_move_client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = relate_frame
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y

    goal_rad = goal_angle * np.pi / 180
    goal_quat = quaternion_from_euler(0, 0, goal_rad)
    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.x = goal_quat[0]
    goal.target_pose.pose.orientation.y = goal_quat[1]
    goal.target_pose.pose.orientation.z = goal_quat[2]
    goal.target_pose.pose.orientation.w = goal_quat[3]

    # Sends the goal to the action server.
    self.turtle_move_client.send_goal(goal)
    
    # Waits for the server to finish performing the action.
    wait = self.turtle_move_client.wait_for_result()
    
    # Result of executing the action
    if not wait:
      rospy.logerr("Action server not available!")
      rospy.signal_shutdown("Action server not available!")
    else:
      return self.turtle_move_client.get_result()

  def set_turtlebot_velocity(self, linear_velocity, angular_velocity):
    twist = Twist()
    twist.linear.x = linear_velocity[0]
    twist.linear.y = linear_velocity[1]
    twist.linear.z = linear_velocity[2]
    twist.angular.x = angular_velocity[0]
    twist.angular.y = angular_velocity[1]
    twist.angular.z = angular_velocity[2]
    
    self.turtle_cmd_pub.publish(twist)

  def turtle_state_update(self):
    msg = rospy.wait_for_message('/odom', Odometry)
    orientation_q = msg.pose.pose.orientation
    
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    

    if all_close(msg.pose.pose, self.turtle_state.pose, 0.001):
      self.turtle_state.is_stop = True
    else:
      self.turtle_state.is_stop = False
    self.turtle_state.pose = msg.pose.pose
    self.turtle_state.rpy = euler_from_quaternion(orientation_list)

  def turtle_stop(self):
    self.turtle_state_update()
    while not self.turtle_state.is_stop:
      self.set_turtlebot_velocity(linear_velocity=[0,0,0],
                                  angular_velocity=[0,0,0])
      self.turtle_state_update()
    
  @staticmethod
  def get_scan_data():
    scan = rospy.wait_for_message('scan', LaserScan)
    scan_filter = []

    samples = len(scan.ranges)  # The number of samples is defined in 
                                # turtlebot3_<model>.gazebo.xacro file,
                                # the default is 360.
    samples_view = 1            # 1 <= samples_view <= samples

    if samples_view > samples:
      samples_view = samples

    if samples_view is 1:
      scan_filter.append(scan.ranges[0])

    else:
      left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
      right_lidar_samples_ranges = samples_view//2

      left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
      right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
      scan_filter.extend(left_lidar_samples + right_lidar_samples)

    for i in range(samples_view):
      if scan_filter[i] == float('Inf'):
        scan_filter[i] = 3.5
      elif math.isnan(scan_filter[i]):
        scan_filter[i] = 0
    
    return scan_filter
  #endregion

class CameraManager(object):
  def __init__(self):
    super(CameraManager, self).__init__()

  def find_target_object(self, target_object):
    target_point = [0.3, 0.5, 0.3]
    return TargetBox.box_1

def main():
  try:
    rospy.init_node('task_manager', anonymous=True)
    print(project_info_msg)
    print("============ 1. Initialize robot")
    robot = RobotManager(localization=True) # turtlebot with open manipulator
    cam = CameraManager() # realsense
    while True:
      input("Press Enter to Continue the Project ")
      print("============ 2. Get Target Object Information from Customer")
      target_key = input(target_info_msg)
      target_object = TARGET_INFO[target_key]
      print("Try to Find {}".format(target_object))

      print("============ 3. Move to Storage Box")
      robot.move_to_goal(goal_x=STORAGE_BOX_POSITION[0],
                         goal_y=STORAGE_BOX_POSITION[1],
                         goal_angle=STORAGE_BOX_POSITION[2])

      print("============ 4. Find Target Object")
      #TODO: Find Target objects x, y, z (or Grasp Point)
      target_box = cam.find_target_object(target_object)
      print("Find {} in {} Box".format(target_object, target_box))

      print("============ 5. Pick up Target Object")
      #TODO: approach to target object(until availbale to grasp)
      input("Approach to Box {}".format(target_box))
      if target_box in [1, 3]:
        target_movement = GraspState.left_box_approach
      else:
        target_movement = GraspState.right_box_approach
      robot.move_to_goal(goal_x=target_movement[0], 
                         goal_y=target_movement[1],
                         goal_angle=0,
                         relate_frame=robot.planning_frame
                         )

      #TODO: Grasp pose
      input("Set Manipulator to Grasp {} Box".format(target_box))
      if target_box in [1, 2]:
        target_point, _ = robot.get_nearest_available_point(GraspState.upper_box_grasp)
      else:
        target_point, _ = robot.get_nearest_available_point(GraspState.down_box_grasp)
      robot.go_to_xyz_goal_with_constraint(target_point)

      #TODO: Grasp
      robot.close_gripper()
      input("Press Enter to Continue the Project ")

      print("============ 6. Move to Customer")
      #TODO: get x, y position of Customer
      #TODO: move to x, y
      input("Press Enter to Continue the Project ")

      print("============ 7. Place Object to Customer Hand")
      #TODO: Find Hand and place object to customer
      input("Press Enter to Continue the Project ")

  except rospy.ROSInterruptException:
      return
  except KeyboardInterrupt:
      return
  

if __name__=="__main__":
  main()
  rospy.spin()

