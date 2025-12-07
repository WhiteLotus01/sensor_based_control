# controller_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from math import sin, cos, atan2, pi, sqrt
 
class LyapunovController(Node):
    def __init__(self):
        super().__init__('lyapunov_controller')
        # Parameters
        self.R = 2.0
        self.omega_r = 0.11
        self.k1 = 1.0
        self.k2 = 3.0
        self.k3 = 2.0
        self.mode = 'eight'  # 'circle' or 'eight' (for figure 8)


        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.t = 0.0  # sim time for reference trajectory
 
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = atan2(siny_cosp, cosy_cosp)
 
    def control_loop(self):
        dt = 0.05
        self.t += dt

        if self.mode == 'circle':
            xd = self.R * cos(self.omega_r * self.t)
            yd = self.R * sin(self.omega_r * self.t)
            xdot = -self.R * self.omega_r * sin(self.omega_r * self.t)
            ydot = self.R * self.omega_r * cos(self.omega_r * self.t)
            xddot = -self.R * self.omega_r**2 * cos(self.omega_r * self.t)
            yddot = -self.R * self.omega_r**2 * sin(self.omega_r * self.t)
        else:  # Figure 8
            xd = self.R * sin(self.omega_r * self.t)
            yd = self.R * sin(2*self.omega_r*self.t)/2
            xdot = self.R*self.omega_r*cos(self.omega_r*self.t)
            ydot = self.R*self.omega_r*cos(2*self.omega_r*self.t)
            xddot = -self.R*self.omega_r**2*sin(self.omega_r*self.t)
            yddot = -self.R*self.omega_r**2*sin(2*self.omega_r*self.t)
 
        thetad = atan2(ydot, xdot)
        vd = sqrt(xdot**2 + ydot**2)
        wd = (xdot*yddot - ydot*xddot)/(xdot**2 + ydot**2)
 
        # Errors in robot frame
        ex = cos(self.theta)*(xd - self.x) + sin(self.theta)*(yd - self.y)
        ey = -sin(self.theta)*(xd - self.x) + cos(self.theta)*(yd - self.y)
        etheta = (thetad - self.theta + pi) % (2*pi) - pi
 
        # Lyapunov control
        v = vd * cos(etheta) + self.k1 * ex
        w = wd + self.k2 * vd * ey + self.k3 * sin(etheta)
 
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)
 
def main(args=None):
    rclpy.init(args=args)
    node = LyapunovController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()