#!/usr/bin/env python
# use moveit_commander (the Python MoveIt user interfaces )
from math import pi
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion


## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupTutorial(object):
  """MoveGroupTutorial"""
  def __init__(self):
    super(MoveGroupTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('move_group_tutorial_ur5', anonymous=True)
 
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator" #group_name = "arm" 
    group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    reference_frame = 'base_link'
    group.set_pose_reference_frame(reference_frame)
    ee_link = group.get_end_effector_link()
    
    group.set_end_effector_link(ee_link)

    group.set_planner_id("RRTConnect")
    group.set_planning_time(10)  
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.reference_frame = reference_frame
    self.end_effector_link = ee_link

    self.origin_degree = [0, 0, 0]
    self.target_angle = 0
    self.target_x = 0
    self.target_y = 0

  def pose(self):
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] =  pi * 0.5
    joint_goal[1] = -pi * 0.5
    joint_goal[2] =  pi * 0.5
    joint_goal[3] = -pi * 0.5
    joint_goal[4] = -pi * 0.5
    joint_goal[5] = -pi * 0.5    
    group.go(joint_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    current_joints = group.get_current_joint_values()
    origin_orientation =  group.get_current_pose().pose.orientation
    print("origin_orientation", origin_orientation)
    origindegree =  euler_from_quaternion([origin_orientation.x, origin_orientation.y, origin_orientation.z, origin_orientation.w]) 

    
    self.origin_degree[0] = origindegree[0]/3.14*180.0
    self.origin_degree[1] = origindegree[1]/3.14*180.0
    self.origin_degree[2] = origindegree[2]/3.14*180.0
    print("origindegree:", self.origin_degree)
    return all_close(joint_goal, current_joints, 0.01)

  def pose1(self):
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 91.46*radian
    joint_goal[1] = -51.03*radian
    joint_goal[2] = 8.27*radian
    joint_goal[3] = -89.93*radian
    joint_goal[4] = -85.34*radian
    joint_goal[5] = -111.76*radian    
    group.go(joint_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    current_joints = group.get_current_joint_values()
    origin_orientation =  group.get_current_pose().pose.orientation
    origindegree =  euler_from_quaternion([origin_orientation.x, origin_orientation.y, origin_orientation.z, origin_orientation.w])
    print("origindegree:", origindegree)

    
    self.origin_degree[0] = origindegree[0]/3.14*180.0
    self.origin_degree[1] = origindegree[1]/3.14*180.0
    self.origin_degree[2] = origindegree[2]/3.14*180.0
    return all_close(joint_goal, current_joints, 0.01)

  def pose2(self):
    group = self.group
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = -101.31*radian
    joint_goal[1] = -106.51*radian
    joint_goal[2] = 91.08*radian
    joint_goal[3] = -73.98*radian
    joint_goal[4] = -93.54*radian
    joint_goal[5] = -111.76*radian    
    group.go(joint_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    current_joints = group.get_current_joint_values()
    origin_orientation =  group.get_current_pose().pose.orientation
    origindegree =  euler_from_quaternion([origin_orientation.x, origin_orientation.y, origin_orientation.z, origin_orientation.w]) 

    
    self.origin_degree[0] = origindegree[0]/3.14*180.0
    self.origin_degree[1] = origindegree[1]/3.14*180.0
    self.origin_degree[2] = origindegree[2]/3.14*180.0
    return all_close(joint_goal, current_joints, 0.01)

  def pose3(self):
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.13469
    pose_goal.position.y = -0.32845   
    pose_goal.position.z = 0.21182
    pose_goal.orientation.x = 0.13469
    pose_goal.orientation.y = -0.32845 
    pose_goal.orientation.z = 0.21182
    pose_goal.orientation.w = 0.70439
    group.set_pose_target(pose_goal, self.end_effector_link)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

  def pose5(self):
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 134.69
    pose_goal.position.y = -328.45   
    pose_goal.position.z = 211.82
    
    quaternion = quaternion_from_euler(1.547 ,0.05, 0.101 )
    group.set_pose_target(pose_goal, self.end_effector_link)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

  def pose4(self):
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.11000
    pose_goal.position.y = 0.47000
    pose_goal.position.z = 0.50000

    quaternion = quaternion_from_euler(np.radians(-180),np.radians(0), np.radians(90) )   #roll_angle(z), pitch_angle, yaw_angle  



    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    group.set_pose_target(pose_goal, self.end_effector_link)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
  
  

def main():
  try:
    print("============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ...")
    tutorial = MoveGroupTutorial()
  
    input()
    print('================================')
    print("pose1")
    tutorial.pose()
    #input()
    #print('================================')
    #print("pose2")
    #tutorial.pose2()
    input()
    print('================================')
    print("pose4")
    tutorial.pose4()
    #input()
    #print('================================')
    #print("pose")
    #tutorial.pose()

    
    print("============ Python tutorial demo complete!")
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  radian=0.017
  rospy.init_node('Strategy')
  main()
