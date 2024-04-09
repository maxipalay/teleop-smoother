# TODO:
# Allow multiple plots (reset variables)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from std_srvs.srv import Empty
import numpy as np
import matplotlib.pyplot as plt

class Plotter(Node):
    def __init__(self, ):
        super().__init__("teleop_smoother_plotter")

        self.declare_parameter("raw_vel_topic", descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="Raw vel topic."))
        self.declare_parameter("smooth_vel_topic", descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="Smooth vel topic."))
        self.declare_parameter("loop_frequency", 50.0, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Loop frequency for acceleration updates (Hz)."))
        
        
        self.timer_frequency = self.get_parameter(
            "loop_frequency").get_parameter_value().double_value
        self.raw_vel_topic = self.get_parameter(
            "raw_vel_topic").get_parameter_value().string_value
        self.smooth_vel_topic = self.get_parameter(
            "smooth_vel_topic").get_parameter_value().string_value
        
        self.timer_dt = 1.0 / \
            self.get_parameter(
                "loop_frequency").get_parameter_value().double_value
        
        self.sub_raw = self.create_subscription(Twist, self.raw_vel_topic, self.raw_cmd_vel_cb, 10)
        self.sub_smooth = self.create_subscription(Twist, self.smooth_vel_topic, self.smooth_cmd_vel_cb, 10)

        # timer for control loop
        self.timer_control = self.create_timer(self.timer_dt, self.loop_cb)

        # create services
        self.capture_service = self.create_service(
            Empty, "capture_data", self.capture_callback)
        self.plot_service = self.create_service(
            Empty, "plot_data", self.plot_callback)

        # capturing data
        self.capturing_data = False

        # store values
        self.smooth_values = []
        self.raw_values = []
        self.time_values = []
        self.time_now = None

        self.latest_raw_cmd = Twist()
        self.latest_smooth_cmd = Twist()


    def loop_cb(self,):
        if self.capturing_data:
            time_now = self.get_clock().now().nanoseconds
            self.raw_values.append([self.latest_raw_cmd.linear.x,
                                    self.latest_raw_cmd.linear.y,
                                    self.latest_raw_cmd.linear.z,
                                    self.latest_raw_cmd.angular.x,
                                    self.latest_raw_cmd.angular.y,
                                    self.latest_raw_cmd.angular.z])       
            self.smooth_values.append([self.latest_smooth_cmd.linear.x,
                                       self.latest_smooth_cmd.linear.y,
                                       self.latest_smooth_cmd.linear.z,
                                       self.latest_smooth_cmd.angular.x,
                                       self.latest_smooth_cmd.angular.y,
                                       self.latest_smooth_cmd.angular.z]) 
            self.time_values.append(time_now)

    def raw_cmd_vel_cb(self, msg):
        self.latest_raw_cmd = msg

    def smooth_cmd_vel_cb(self, msg):
        self.latest_smooth_cmd = msg

    def capture_callback(self, request, response):
        self.capturing_data = not self.capturing_data
        return response
    
    def plot_callback(self, request, response):
        smooth_values_mat = np.matrix(self.smooth_values)
        raw_values_mat = np.matrix(self.raw_values)
        time_arr = np.array(self.time_values)
        plt.figure()
        plt.title('smooth vs. raw velocities')
        for i in range(6):
            plt.plot(time_arr, smooth_values_mat[:,i])
        
        for i in range(6):
            plt.plot(time_arr, raw_values_mat[:,i])
        
        plt.legend(['smooth lin x','smooth lin y','smooth lin z','smooth ang x','smooth ang y','smooth ang z','raw lin x','raw lin y','raw lin z','raw ang x','raw ang y','raw ang z'])
        plt.show()

        return response
    

def main(args=None):
    rclpy.init(args=args)
    plotter = Plotter()
    rclpy.spin(plotter)
    rclpy.shutdown()


if __name__ == '__main__':
    main()