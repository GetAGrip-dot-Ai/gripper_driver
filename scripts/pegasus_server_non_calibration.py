#! /usr/bin/env python3

import copy
import numpy as np
from numpy.linalg import norm
import rospy
from std_msgs.msg import Bool, Int16
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ag_gripper_driver.srv import Pegasus, PegasusResponse
from dynamixel_workbench_msgs.msg import DynamixelStateList

GRIP_THRESHOLD = 1300 # TODO tune this

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

        self.motor_names = ['mx28', 'mx64']

        # -------- TUNED VALUES
        # 1) comment out section below
        # 2) run this script
        # 3) from printed q_zero, increment until tuned values are found

        # hand tuned, calibration will override these values
        self.open_pos = np.array([0.2, -2]) # first is mx28, second is mx64
        self.close_pos = np.array([1.1, -0.8])
        self.reset_pos = copy.copy(self.q_zero)

        self.grab_offset_from_closed = np.array([0.1, 0.0]) # dont wann squeeze the pep to deth
        self.close_pos = self.close_pos - self.grab_offset_from_closed

        # -------- TUNED VALUES --------- #
        # do not calibrate 
        # self.calibrate()

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
        open_cutter_harvest_jtp.positions = np.array([self.close_pos[0], self.open_pos[1]])
        open_cutter_harvest_jtp.time_from_start = rospy.Duration.from_sec(0.5)  
        self.open_cutter_harvest_msg.points.append(open_cutter_harvest_jtp)
        
        # sub to read dynamixel current
        self.motor_states_sub = rospy.Subscriber("/ag_gripper/dynamixel_state", DynamixelStateList, self.current_callback)
        self.grip_current = rospy.Publisher("/ag_gripper/grip_current", Int16, queue_size=1)
        self.motor_currents = {id: 0 for id in self.motor_names}
        # thresholds are the limits for a successful grip/cut
        self.motor_thresholds = {
            'mx28': 1500,
            'mx64': 1500
        }
        
        # publisher to say if gripper is gripping something
        self.grip_publisher = rospy.Publisher("/ag_gripper/has_grip", Bool, queue_size=1)
        self.grip_timer = rospy.Timer(rospy.Duration(0.1), self.publish_grip)
        
        # start service
        self.service = rospy.Service('/gripper_service', Pegasus, self.run)
        rospy.spin()
        
    def current_callback(self, msg):
        
        for motor in msg.dynamixel_state:
            
            id = motor.name
            self.motor_currents[id] = motor.present_current
            
        self.grip_current.publish(self.motor_currents['mx28'])
            
    def publish_grip(self, event):
        
        if self.motor_currents['mx28'] > GRIP_THRESHOLD:
            self.grip_publisher.publish(True)
        else:
            self.grip_publisher.publish(False)
        
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
            close_gripper_jtp.positions = np.array([self.close_pos[0], q.position[1]])
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
            open_cutter_jtp.positions = np.array([q.position[0], self.open_pos[1]])
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
            close_cutter_jtp.positions = np.array([q.position[0], self.close_pos[1]])
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
            open_gripper_jtp.positions = [self.open_pos[0], q.position[1]]
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
            close_gripper_jtp.positions = np.array([self.close_pos[0], q.position[1]])
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
    
