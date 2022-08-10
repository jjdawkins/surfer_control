#!/usr/bin/python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from surfer_msgs.msg import Status


class SurferController(Node):

    def __init__(self):
        super().__init__('surfer_controller')
        self.namespace = self.get_namespace()
        self.declare_parameter('Kp_xy',50)
        self.declare_parameter('Ki_xy',5)
        self.declare_parameter('Kd_xy',0)
        self.declare_parameter('Kp_yaw',100)
        self.declare_parameter('Ki_yaw',10)
        self.declare_parameter('Kd_yaw',0)

        self.Kp_xy = self.get_parameter('Kp_xy').get_parameter_value().double_value
        self.Ki_xy = self.get_parameter('Ki_xy').get_parameter_value().double_value
        self.Kd_xy = self.get_parameter('Kd_xy').get_parameter_value().double_value        
        self.Kp_yaw = self.get_parameter('Kp_yaw').get_parameter_value().double_value
        self.Ki_yaw = self.get_parameter('Ki_yaw').get_parameter_value().double_value
        self.Kd_yaw = self.get_parameter('Kd_yaw').get_parameter_value().double_value

        self.Kp_xy = 10.0
        self.Ki_xy = 0.0
        self.Kd_xy = 0.0        
        self.Kp_yaw = 10.0
        self.Ki_yaw = 0.0
        self.Kd_yaw = 0

        self.des_vel_x = 0
        self.des_vel_y = 0
        self.des_vel_yaw = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel_yaw = 0
        self.x_err_int = 0
        self.y_err_int = 0
        self.yaw_err_int = 0

        self.cmd_vel_sub = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)
        self.odom_sub = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.status_sub = self.create_subscription(Status,'status',self.status_callback,10)


        self.force_pub = self.create_publisher(Twist,'cmd_force',10)

        self.dt = 0.1  # seconds
        self.switch_time = (self.get_clock().now().nanoseconds)*(1e-9)
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.i = 0
        self.dir = 1
        self.status = Status()

 
        self.last_cmd_time = (self.get_clock().now().nanoseconds)*(1e-9)

    def status_callback(self,msg):
        self.status = msg

    def cmd_vel_callback(self,msg):
        self.last_cmd_time = (self.get_clock().now().nanoseconds)*(1e-9)
        self.des_vel_x = msg.linear.x
        self.des_vel_y = msg.linear.y
        self.des_vel_yaw = msg.angular.z

    def odom_callback(self,msg):
        self.vel_x = msg.twist.twist.linear.x
        self.vel_y = msg.twist.twist.linear.y
        self.vel_yaw = msg.twist.twist.angular.z


    def timer_callback(self):
        cmd_msg = Twist()
        vel_x_err = self.des_vel_x - self.vel_x
        vel_y_err = self.des_vel_y - self.vel_y
        vel_yaw_err = self.des_vel_yaw - self.vel_yaw

        #print(vel_x_err,vel_y_err,vel_yaw_err)

        self.x_err_int = self.x_err_int + vel_x_err*self.dt
        self.y_err_int = self.y_err_int + vel_y_err*self.dt
        self.yaw_err_int = self.yaw_err_int + vel_yaw_err*self.dt

        cmd_fx = self.Kp_xy*vel_x_err + self.Ki_xy*self.x_err_int
        cmd_fy = self.Kp_xy*vel_y_err + self.Ki_xy*self.y_err_int
        cmd_mz = self.Kp_yaw*vel_yaw_err + self.Ki_yaw*self.yaw_err_int


        cmd_msg.linear.x = cmd_fx
        cmd_msg.linear.y = cmd_fy
        cmd_msg.angular.z = cmd_mz
        #print(self.Kp_xy,self.Kp_xy,self.Kp_yaw)

        self.force_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)

    surf_ctrl = SurferController()
    rclpy.spin(surf_ctrl)
    surf_ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
