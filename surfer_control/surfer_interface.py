#!/usr/bin/python3

import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class SurferInterface(Node):

    def __init__(self):
        super().__init__('surfer_interface')
        self.namespace = self.get_namespace()
        self.pub_mot_fr = self.create_publisher(Float64, self.namespace+'/propeller_joint_fr/cmd_thrust', 10)
        self.pub_mot_fl = self.create_publisher(Float64, self.namespace+'/propeller_joint_fl/cmd_thrust', 10)
        self.pub_mot_br = self.create_publisher(Float64, self.namespace+'/propeller_joint_br/cmd_thrust', 10)
        self.pub_mot_bl = self.create_publisher(Float64, self.namespace+'/propeller_joint_bl/cmd_thrust', 10)

        self.force_sub = self.create_subscription(Twist,'cmd_force',self.force_callback,10)

        timer_period = 0.1  # seconds
        self.switch_time = (self.get_clock().now().nanoseconds)*(1e-9)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.dir = 1
        self.cmd_fx = 0
        self.cmd_fy = 0
        self.cmd_mz = 0
        self.f_fl = 0.0
        self.f_bl = 0.0
        self.f_br = 0.0
        self.f_fr = 0.0
        self.spd_fl = 0
        self.spd_bl = 0
        self.spd_br = 0
        self.spd_fr = 0
        self.thrust_coeff = 0.05
        self.cmd_timeout = 1.0
        self.last_cmd_time = (self.get_clock().now().nanoseconds)*(1e-9)

    def force_callback(self,msg):
        self.last_cmd_time = (self.get_clock().now().nanoseconds)*(1e-9)
        self.cmd_fx = msg.linear.x
        self.cmd_fy = msg.linear.y
        self.cmd_mz = msg.angular.z


    def thrust_to_speed(self,F):
        spd = 0
        if(F < 0):
            spd = (-1)*math.sqrt(abs(F/self.thrust_coeff))

        else:
            spd = math.sqrt(abs(F/self.thrust_coeff))

        return spd

    def timer_callback(self):
        msg = Float64()

        self.f_fl = 0.3536*self.cmd_fx - 0.3536*self.cmd_fy - 2.6315*self.cmd_mz
        self.f_bl = 0.3536*self.cmd_fx + 0.3536*self.cmd_fy - 2.6315*self.cmd_mz
        self.f_br = 0.3536*self.cmd_fx - 0.3536*self.cmd_fy + 2.6315*self.cmd_mz
        self.f_fr = 0.3536*self.cmd_fx + 0.3536*self.cmd_fy + 2.6315*self.cmd_mz


        if((self.get_clock().now().nanoseconds)*(1e-9)-self.last_cmd_time > self.cmd_timeout):
            self.f_fl = 0.0
            self.f_bl = 0.0 
            self.f_br = 0.0
            self.f_fr = 0.0


        # If the command hasn't timed out set speed values otherwise set to zero.
        '''
        if((self.get_clock().now().nanoseconds)*(1e-9)-self.last_cmd_time < self.cmd_timeout):
            self.spd_fl = self.thrust_to_speed(self.f_fl)
            self.spd_bl = self.thrust_to_speed(self.f_bl)
            self.spd_br = self.thrust_to_speed(self.f_br)
            self.spd_fr = self.thrust_to_speed(self.f_fr)
        else:
            self.spd_fl = 0.0
            self.spd_bl = 0.0
            self.spd_br = 0.0
            self.spd_fr = 0.0
        '''

        #print(self.spd_fl,self.spd_bl,self.spd_br,self.spd_fr)


        msg.data = self.f_fl 
        self.pub_mot_fl.publish(msg)
        msg.data = self.f_bl
        self.pub_mot_bl.publish(msg)
        msg.data = self.f_br
        self.pub_mot_br.publish(msg)
        msg.data = self.f_fr
        self.pub_mot_fr.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    surf_iface = SurferInterface()
    rclpy.spin(surf_iface)
    surf_iface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
