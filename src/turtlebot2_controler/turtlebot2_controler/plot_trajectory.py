# plot_trajectory.py
 
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from math import cos, sin, pi
 
class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
 
        # Paramètres identiques à ceux du contrôleur
        self.R = 0.45
        self.omega_r = 0.11
        self.mode = 'circle'  # 'circle' ou 'eight'
 
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.update_plot)  # 20 Hz
 
        # Stockage des positions
        self.x_real = []
        self.y_real = []
        self.t_data = []
 
        self.t = 0.0
 
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.x_real.append(x)
        self.y_real.append(y)
 
    def update_plot(self):
        dt = 0.05
        self.t += dt
        self.t_data.append(self.t)
 
        # Calcul trajectoire théorique
        xd_list = []
        yd_list = []
 
        for t in self.t_data:
            if self.mode == 'circle':
                xd = self.R * cos(self.omega_r * t)
                yd = self.R * sin(self.omega_r * t)
            else:  # figure-8
                xd = self.R * sin(self.omega_r * t)
                yd = self.R * sin(2*self.omega_r*t)/2
            xd_list.append(xd)
            yd_list.append(yd)
 
        # Tracé
        plt.clf()
        plt.plot(xd_list, yd_list, 'r--', label='Trajectoire théorique')
        plt.plot(self.x_real, self.y_real, 'b-', label='Trajectoire réelle')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.axis('equal')
        plt.legend()
        plt.pause(0.001)  # Rafraîchissement
 
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    plt.ion()
    plt.figure()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()