#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, Int32MultiArray, Int32
from compute import IK_4dof
import time
import math
from rospy_tutorials.msg import Floats


ik = IK_4dof(12.0, 14, 15.1, 6)


def coordinates_callback(msg):
    # print(msg)
    dx = 1.7
    dy = 1.0
    clearance = 4.0
    pick_x = msg.data[0] - dx
    pick_y = msg.data[1] - dy
    pick_z = msg.data[2]

    place_x = msg.data[3] - dx
    place_y = msg.data[4] - dy
    place_z = msg.data[5]

    pick_theta1, pick_theta2, pick_theta3, pick_theta4 = ik((pick_x, pick_y, pick_z))
    pick_clearance_theta1, pick_clearance_theta2, pick_clearance_theta3, pick_clearance_theta4 = ik((pick_x, pick_y, pick_z + clearance))
    place_theta1, place_theta2, place_theta3, place_theta4 = ik((place_x, place_y, place_z))
    place_clearance_theta1, place_clearance_theta2, place_clearance_theta3, place_clearance_theta4 = ik((place_x, place_y, place_z+clearance))

    
    #home
    home_theta1 = 0.0
    home_theta2 = 90.0
    home_theta3 = 90.0
    home_theta4 = 0.0

    # pick and place sequence
    gripper_open_coords = [home_theta1, home_theta2, home_theta3, home_theta4, 180.0]
    home_to_pick = [math.ceil(pick_theta1), math.ceil(pick_theta2), math.ceil(pick_theta3), math.ceil(pick_theta4), math.ceil(180.0)]
    close_gripper_pick_loc = [pick_theta1, pick_theta2, pick_theta3, pick_theta4, 0.0]
    pick_to_clearance = [pick_clearance_theta1, pick_clearance_theta2, pick_clearance_theta3, pick_clearance_theta4, 0.0]
    pick_to_place_clearance = [place_clearance_theta1, place_clearance_theta2, place_clearance_theta3, place_clearance_theta4, 0.0]
    place = [place_theta1, place_theta2, place_theta3, place_theta4, 180.0]
    place_to_clearance = [place_clearance_theta1, place_clearance_theta2, place_clearance_theta3, place_clearance_theta4, 180.0]
    place_to_home = [home_theta1, home_theta2, home_theta3, home_theta4, 180.0]

    entire_sequence = [gripper_open_coords,
                        home_to_pick,
                        close_gripper_pick_loc,
                        pick_to_clearance,
                        pick_to_place_clearance,
                        place, 
                        place_to_clearance,
                        place_to_home]
    # print("entire sequence : ", entire_sequence)
    

    # publish pnp sequence to arduino
    pnp_sequence = Floats()
    # pnp_sequence.data = home_to_pick
    pnp_sequence.data = [66.0, 52.0, 117.0, 150.0, 180.0]
    to_arduino_pub.publish(pnp_sequence)
    print("test seq : ", pnp_sequence.data)
    print("message published to arduino")
    # [66, 52, 117, 150, 180]

    testing = Int32()
    testing.data = 10000
    test_pub.publish(testing)

    print("-------------\n")

    time.sleep(2.0)
    

if __name__=="__main__":
    rospy.init_node("ik_pub_node")

    to_arduino_pub = rospy.Publisher("/coords_to_arduino",
                                     Floats,
                                     queue_size=10)
    
    test_pub = rospy.Publisher("/testing_arduino",
                                Int32,
                                queue_size=10)

    sub = rospy.Subscriber("/robot_frame_coords",
                            Float64MultiArray,
                            coordinates_callback)
    
    rospy.spin()