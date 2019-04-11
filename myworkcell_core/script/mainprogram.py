#!/usr/bin/env python


import Tkinter as tk
import rospy
from std_srvs.srv import Trigger
from std_msgs.msg import Int32

object_pub = rospy.Publisher('/main_program/Object', Int32, queue_size=1, latch=True)
rospy.init_node('node_name')

def set_object(object):
    objectdata = Int32()
    objectdata.data = object
    object_pub.publish(objectdata)

def set_robot():
    print("Sending command to robot")
    rospy.wait_for_service('move_robot')
    try:
        move_robot_client = rospy.ServiceProxy('move_robot', Trigger)
        resp = move_robot_client()
        if(resp.success):
             print("Send successfully")
        else:
             print("Service call failed: %s"%resp.message)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

 
top = tk.Tk()
# Code to add widgets will go here...
frame = tk.Frame(top)
frame.pack()

button1 = tk.Button(frame, text="Head & Shoulders", command=lambda:set_object(1))
button1.pack(side=tk.LEFT)
button2 = tk.Button(frame, text="Koko Krunch", command=lambda:set_object(2))
button2.pack(side=tk.LEFT)
button3 = tk.Button(frame, text="Butter Cookies", command=lambda:set_object(3))
button3.pack(side=tk.LEFT)
button4 = tk.Button(frame, text="Febreze", command=lambda:set_object(4))
button4.pack(side=tk.LEFT)
button5 = tk.Button(frame, text="Gillette Venus", command=lambda:set_object(5))
button5.pack(side=tk.LEFT)
button6 = tk.Button(frame, text="Honey Stars", command=lambda:set_object(6))
button6.pack(side=tk.LEFT)
button7 = tk.Button(frame, text="Kitkat", command=lambda:set_object(7))
button7.pack(side=tk.LEFT)
button8 = tk.Button(frame, text="Kitkat Box", command=lambda:set_object(8))
button8.pack(side=tk.LEFT)
button9 = tk.Button(frame, text="Maggi", command=lambda:set_object(9))
button9.pack(side=tk.LEFT)
button10 = tk.Button(frame, text="Milo Drink", command=lambda:set_object(10))
button10.pack(side=tk.LEFT)
button11 = tk.Button(frame, text="Milo Pack", command=lambda:set_object(11))
button11.pack(side=tk.LEFT)
button12 = tk.Button(frame, text="Nivea", command=lambda:set_object(12))
button12.pack(side=tk.LEFT)
button13 = tk.Button(frame, text="Oreo", command=lambda:set_object(13))
button13.pack(side=tk.LEFT)
button14 = tk.Button(frame, text="Wild about Nerds", command=lambda:set_object(14))
button14.pack(side=tk.LEFT)
button15 = tk.Button(frame, text="Move Robot", fg="red", command=set_robot)
button15.pack(side=tk.LEFT)

top.mainloop()
