#!/usr/bin/env python3
import tkinter as tk
from tkinter import *
import rclpy
import math
from rclpy.node import Node 
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import time

'''
['shoulder', 'elbow', 'wrist_angle', 'wrist_rotate', 'waist', 'gripper', 'left_finger', 'right_finger']
array('d', [2.2232538805511126e-06, 4.084800743964223e-05, -1.9578201611025747e-06, 2.447309914899165e-05, -9.896119212982057e-08, -0.04349679512560023, 0.015000102349000125, -0.014999993934774712])
'''        
class vx250_custom_controller(Node):
    def __init__(self):
        super().__init__("vx250_custom_controller")
        self.get_logger().info("The controller has been initialized")
        self.publisher_ = self.create_publisher(JointTrajectory,"vx250/arm_controller/joint_trajectory",10)

    def publish_action_position(self,position_list:list)->bool:
        msg = JointTrajectory()
        msg.joint_names = ['waist','shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
        msg.points = []
        a = JointTrajectoryPoint()
        for i in range(0,len(position_list)):
            position_list[i] = float(math.radians(position_list[i]))
        
        a.positions = position_list
        msg.points.append(a)
        print(msg.points)
        self.publisher_.publish(msg = msg)
        return True

def validate(obj,waist:float, shoulder:float, elbow:float, wrist:float, wrist_angle:float):
    
    if obj.publish_action_position([waist,shoulder,elbow,wrist,wrist_angle]):
        print("Command has been Published")
    else:
        print("Command Has failed Check Values")
        


def main(args = None):

    rclpy.init()
    node = vx250_custom_controller()
    root = Tk()
    root.title("VX250 controller")
    root.geometry("370x250")
    

    Waist_lbl = Label(root,text="Waist\t")
    Waist_lbl.grid(column=0,row = 0)

    Waist_txt = tk.Entry(root,width="20")
    Waist_txt.grid(column=2,row = 0)
    Waist_txt.insert(END,'0')

    Shoulder_lbl = Label(root,text="Shoulder\t")
    Shoulder_lbl.grid(column=0,row = 1)


    Shoulder_txt = tk.Entry(root,width="20")
    Shoulder_txt.grid(column=2,row = 1)
    Shoulder_txt.insert(END, '0')

    Elbow_lbl = Label(root,text="Elbow\t")
    Elbow_lbl.grid(column=0,row = 2)

    Elbow_txt = tk.Entry(root,width="20")
    Elbow_txt.grid(column=2,row = 2)
    Elbow_txt.insert(END, '0')

    wrist_lbl = Label(root,text="wrist\t")
    wrist_lbl.grid(column=0,row = 3)

    wrist_txt = tk.Entry(root,width="20")
    wrist_txt.grid(column=2,row = 3)
    wrist_txt.insert(END,'0')

    wrist_rotate_lbl = Label(root,text="wrist_rotate")
    wrist_rotate_lbl.grid(column=0,row = 4)

    wrist_rotate_txt = tk.Entry(root,width="20")
    wrist_rotate_txt.grid(column=2,row = 4)
    wrist_rotate_txt.insert(END,'0')

    submit = Button(root,text = "Execute",command = lambda: validate(node, waist=float(Waist_txt.get()),shoulder=float(Shoulder_txt.get()),
                                                             elbow=float(Elbow_txt.get()),wrist=float(wrist_txt.get()),wrist_angle=float(wrist_rotate_txt.get())))
    
    submit.grid(row = 5,column = 1)

    root.mainloop()

    rclpy.shutdown()

main()