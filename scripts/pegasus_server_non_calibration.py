#! /usr/bin/env python3

import copy
import numpy as np
from numpy.linalg import norm
import rospy
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ag_gripper_driver.srv import Pegasus, PegasusResponse
import yaml
import os

class MotorDriverROSWrapper:

    def __init__(self):
        self.gripper_closed = False
        self.controller_pub = rospy.Publisher(
            '/ag_gripper/joint_trajectory', JointTrajectory, queue_size=1)

        # echo joint states and save as zero_offset
        init_state_msgs = rospy.wait_for_message(
            '/ag_gripper/joint_states', JointState)
        self.q_zero = np.array(init_state_msgs.position)
        rospy.loginfo("q_zero: {}".format(self.q_zero))

        self.motor_names = ['cutter', 'gripper']

        # read in yaml file
        absolute_path = os.path.dirname(__file__)
        # print(absolute_path)
        relative_path = "../config/positions.yaml"
        full_path = os.path.join(absolute_path, relative_path)

        with open(full_path) as file:
            positions = yaml.load(file, Loader=yaml.FullLoader)
            # import pdb; pdb.set_trace()
            self.open_pos = np.array([positions['cutter']['open'], positions['gripper']['open']])
            rospy.loginfo("open_pos: {}".format(self.open_pos))
            self.close_pos = np.array([positions['cutter']['close'], positions['gripper']['close']])
            rospy.loginfo("close_pos: {}".format(self.close_pos))
            self.reset_pos = copy.copy(self.q_zero)
            self.close_pos = self.close_pos - np.array(positions['grab_offset'])

        # close gripper and cutter msg
        self.close_msg = JointTrajectory()
        self.close_msg.joint_names = self.motor_names
        close_jtp = JointTrajectoryPoint()
        close_jtp.positions = self.close_pos
        close_jtp.time_from_start = rospy.Duration.from_sec(1.0)
        self.close_msg.points.append(close_jtp)

        # open gripper and cutter msg
        self.open_msg = JointTrajectory()
        self.open_msg.joint_names = self.motor_names
        open_jtp = JointTrajectoryPoint()
        open_jtp.positions = self.open_pos
        open_jtp.time_from_start = rospy.Duration.from_sec(1.0)
        self.open_msg.points.append(open_jtp)

        # close and open cutter individually during harvesting procedure
        self.close_cutter_harvest_msg = JointTrajectory()
        self.close_cutter_harvest_msg.joint_names = self.motor_names
        close_cutter_harvest_jtp = JointTrajectoryPoint()
        close_cutter_harvest_jtp.positions = np.array([self.close_pos[0], self.close_pos[1]])
        close_cutter_harvest_jtp.time_from_start = rospy.Duration.from_sec(0.2)
        self.close_cutter_harvest_msg.points.append(close_cutter_harvest_jtp)
        
        self.open_cutter_harvest_msg = JointTrajectory()
        self.open_cutter_harvest_msg.joint_names = self.motor_names
        open_cutter_harvest_jtp = JointTrajectoryPoint()
        open_cutter_harvest_jtp.positions = np.array([self.open_pos[0], self.close_pos[1]])
        open_cutter_harvest_jtp.time_from_start = rospy.Duration.from_sec(0.5)  
        self.open_cutter_harvest_msg.points.append(open_cutter_harvest_jtp)
    
        # start service
        self.service = rospy.Service('/gripper_service', Pegasus, self.run)
        rospy.spin()
        
        
    def run(self, req):
        if req.command == 0:
            #harvest procedure
            rospy.loginfo("------ HARVESTING PROCEDURE INITIATED ------")
            rospy.loginfo("Opening gripper and cutter....")
            self.controller_pub.publish(self.open_msg)
            rospy.sleep(3.0)
            
            rospy.loginfo("Closing gripper....")
            q = rospy.wait_for_message(
            '/ag_gripper/joint_states', JointState)
    
            self.close_gripper_msg = JointTrajectory()
            self.close_gripper_msg.joint_names = self.motor_names
            close_gripper_jtp = JointTrajectoryPoint()
            close_gripper_jtp.positions = np.array([q.position[0], self.close_pos[1]])
            close_gripper_jtp.time_from_start = rospy.Duration.from_sec(1.0)
            self.close_gripper_msg.points.append(close_gripper_jtp)
            self.controller_pub.publish(self.close_gripper_msg)
            
            rospy.sleep(3.0)
            rospy.loginfo("Cutting 3x....")
            self.controller_pub.publish(self.close_cutter_harvest_msg)
            rospy.sleep(1.0)
            self.controller_pub.publish(self.open_cutter_harvest_msg)
            rospy.sleep(3.0)
            self.controller_pub.publish(self.close_cutter_harvest_msg)
            rospy.sleep(1.0)
            self.controller_pub.publish(self.open_cutter_harvest_msg)
            rospy.sleep(3.0)
            self.controller_pub.publish(self.close_cutter_harvest_msg)
            rospy.sleep(1.0)
            self.controller_pub.publish(self.open_cutter_harvest_msg)
            rospy.sleep(3.0)
            rospy.loginfo("------ HARVESTING PROCEDURE COMPLETED ------")
            
            return PegasusResponse(1)
        
        elif req.command == 1:
            #open everything
            rospy.loginfo("Opening gripper and cutter....")
            self.controller_pub.publish(self.open_msg)
            rospy.sleep(3.0)
            state = rospy.wait_for_message(
            '/ag_gripper/joint_states', JointState)
            formatted_state = np.array(state.position)
            rospy.loginfo("open_pos: {}".format(formatted_state))

            return PegasusResponse(1)
        

        elif req.command == 2:
            #close everything
            rospy.loginfo("Closing gripper and cutter....")
            self.controller_pub.publish(self.close_msg)
            rospy.sleep(3.0)
            state = rospy.wait_for_message(
            '/ag_gripper/joint_states', JointState)
            formatted_state = np.array(state.position)
            rospy.loginfo("close_pos: {}".format(formatted_state))

            return PegasusResponse(1)
        
        elif req.command == 3:
            #open cutter
            rospy.loginfo("Opening cutter....")
            q = rospy.wait_for_message(
            '/ag_gripper/joint_states', JointState)

            self.open_cutter_msg = JointTrajectory()
            self.open_cutter_msg.joint_names = self.motor_names
            open_cutter_jtp = JointTrajectoryPoint()
            open_cutter_jtp.positions = np.array([self.open_pos[0], q.position[1]])
            open_cutter_jtp.time_from_start = rospy.Duration.from_sec(1.0)
            self.open_cutter_msg.points.append(open_cutter_jtp)

            self.controller_pub.publish(self.open_cutter_msg)

            return PegasusResponse(1)
        
        elif req.command == 4:
            #close cutter
            rospy.loginfo("Closing cutter....")
            q = rospy.wait_for_message(
            '/ag_gripper/joint_states', JointState)

            # open and close cutter individually
            self.close_cutter_msg = JointTrajectory()
            self.close_cutter_msg.joint_names = self.motor_names
            close_cutter_jtp = JointTrajectoryPoint()
            close_cutter_jtp.positions = np.array([self.close_pos[0], q.position[1]])
            close_cutter_jtp.time_from_start = rospy.Duration.from_sec(1.0)
            self.close_cutter_msg.points.append(close_cutter_jtp)

            self.controller_pub.publish(self.close_cutter_msg)

            return PegasusResponse(1)
        
        elif req.command == 5:
            #open gripper
            rospy.loginfo("Opening gripper....")
            q = rospy.wait_for_message(
            '/ag_gripper/joint_states', JointState)

            self.open_gripper_msg = JointTrajectory()
            self.open_gripper_msg.joint_names = self.motor_names
            open_gripper_jtp = JointTrajectoryPoint()
            open_gripper_jtp.positions = [q.position[0], self.open_pos[1]]
            open_gripper_jtp.time_from_start = rospy.Duration.from_sec(1.0)
            self.open_gripper_msg.points.append(open_gripper_jtp)
            self.controller_pub.publish(self.open_gripper_msg)

            return PegasusResponse(1)
        
        elif req.command == 6:
            #close gripper
            rospy.loginfo("Closing gripper....")
            q = rospy.wait_for_message(
            '/ag_gripper/joint_states', JointState)
    
            self.close_gripper_msg = JointTrajectory()
            self.close_gripper_msg.joint_names = self.motor_names
            close_gripper_jtp = JointTrajectoryPoint()
            close_gripper_jtp.positions = np.array([q.position[0], self.close_pos[1]])
            close_gripper_jtp.time_from_start = rospy.Duration.from_sec(1.0)
            self.close_gripper_msg.points.append(close_gripper_jtp)
            self.controller_pub.publish(self.close_gripper_msg)

            return PegasusResponse(1)
        
        elif req.command == 7:
            
            rospy.loginfo("TESTING")
            for i in range(10):
                self.controller_pub.publish(self.open_msg)
                rospy.sleep(3.0)
                self.controller_pub.publish(self.close_msg)
                rospy.sleep(3.0)
            return PegasusResponse(1)


        else:
            rospy.loginfo("INCORRECT INPUT, NO ACTION TAKEN")

            return PegasusResponse(0)
                



if __name__ == "__main__":
    rospy.init_node("gripper_driver")
    rospy.loginfo("gripper driver started")
    motor_driver_wrapper = MotorDriverROSWrapper()
    
