#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import moveit_commander

from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import DisplayTrajectory


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


    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)


    elif type(goal) is Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = moveit_commander.conversions.pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = moveit_commander.conversions.pose_to_list(goal)
        # Euclidean distance
        d = np.sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip((x1, y1, z1), (x0, y0, z0))))
        # phi = angle between orientations
        cos_phi_half = np.abs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= np.cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterface(object):

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.init_variables()
        self.show_variable()
        

    def init_variables(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        self.pub_display_trajectory = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=5)
       
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

        self.goal_pose = None


    def show_variable(self):
        # We can get the name of the reference frame for this robot:
        print("============ Planning frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        print("============ End effector link: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")
    

    def go_to_joint_state(self, goal_joints):
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        actual_joints = self.move_group.get_current_joint_values()
        actual_joints[0] = goal_joints[0] 
        actual_joints[1] = goal_joints[1] 
        actual_joints[2] = goal_joints[2] 
        actual_joints[3] = goal_joints[3] 
        actual_joints[4] = goal_joints[4] 
        actual_joints[5] = goal_joints[5] 
        actual_joints[6] = goal_joints[6] 

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(actual_joints, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(actual_joints, current_joints, 0.01)


    def go_to_target(self, pose_goal):
        #Planning to a Pose Goal
        self.move_group.set_pose_target(pose_goal)
        
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets() 

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    

    def go_to_target_by_cartesian_path(self, waypoints):
        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
    
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  
        # waypoints to follow, eef_step, jump_threshold

        return plan, fraction
    

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)


    def display_trajectory(self, plan):

        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.pub_display_trajectory.publish(display_trajectory)
    

    def wait_for_state_update(self, obj_is_known=False, obj_is_attached=False, timeout=4):

        start = rospy.get_time()
        seconds = rospy.get_time()
        
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([self.obj_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self.obj_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (obj_is_attached == is_attached) and (obj_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    
    def add_object_to_scen(self, object):
        self.scene.add_box(object["name"], object["pose"], size=object["size"])

        self.obj_name = object["name"]
        return self.wait_for_state_update(obj_is_known=True)


    def attach_obj(self, obj_name, grasping_group):

        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, obj_name, touch_links=touch_links)

        self.obj_name = obj_name
        return self.wait_for_state_update(obj_is_known=False, obj_is_attached=True)
    
    
    def detach_obj(self, obj_name):
        ## We can also detach and remove the object from the planning scene:
        self.scene.remove_attached_object(self.eef_link, name= obj_name)
        return self.wait_for_state_update(obj_is_known=True, obj_is_attached=False)

    def remove_obj(self, obj_name):
        ## We can remove the box from the world.
        self.scene.remove_world_object(obj_name)
        return self.wait_for_state_update(obj_is_known=False, obj_is_attached=False)
