import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType

class Smoother(Node):
    def __init__(self, ):
        super().__init__("teleop_smoother")

        self.declare_parameter("linear_acceleration", descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Maximum linear acceleration [m/s^2] value."))
        self.declare_parameter("angular_acceleration", descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Maximum angular acceleration [deg/s^2] value."))
        self.declare_parameter("loop_frequency", 50.0, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Loop frequency for acceleration updates (Hz)."))
        self.declare_parameter("input_vel_topic", descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="Input topic."))
        self.declare_parameter("output_vel_topic", descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="Output topic."))
        
        self.max_linear_acceleration = self.get_parameter(
            "linear_acceleration").get_parameter_value().double_value
        self.max_angular_acceleration = self.get_parameter(
            "angular_acceleration").get_parameter_value().double_value
        
        self.timer_frequency = self.get_parameter(
            "loop_frequency").get_parameter_value().double_value
        self.input_topic = self.get_parameter(
            "input_vel_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter(
            "output_vel_topic").get_parameter_value().string_value
        
        self.timer_dt = 1.0 / \
            self.get_parameter(
                "loop_frequency").get_parameter_value().double_value
        
        self.sub_tilt = self.create_subscription(Twist, self.input_topic, self.cmd_vel_cb, 10)
        
        self.pub_vel = self.create_publisher(Twist, self.output_topic, 10)

        self.out_velocity = Twist()     # track current output velocity
        self.set_velocity = Twist()    # track setpoint output velocity

        # timer for control loop
        self.timer_control = self.create_timer(self.timer_dt, self.loop_cb)

    def loop_cb(self,):
        # perform the velocity updates
        self.update_velocities()
        self.pub_vel.publish(self.out_velocity)

    def update_velocities(self,):
        # we assume this is called at self.timer_dt intervals
        self.out_velocity.linear.x = self.update_linear(self.set_velocity.linear.x, self.out_velocity.linear.x)
        self.out_velocity.linear.y = self.update_linear(self.set_velocity.linear.y, self.out_velocity.linear.y)
        self.out_velocity.linear.z = self.update_linear(self.set_velocity.linear.z, self.out_velocity.linear.z)

        self.out_velocity.angular.x = self.update_angular(self.set_velocity.angular.x, self.out_velocity.angular.x)
        self.out_velocity.angular.y = self.update_angular(self.set_velocity.angular.y, self.out_velocity.angular.y)
        self.out_velocity.angular.z = self.update_angular(self.set_velocity.angular.z, self.out_velocity.angular.z)

    def update_linear(self, set_vel, out_vel):
        if out_vel < set_vel:   # if the current velocity is smaller (positive)
            out_vel += self.max_linear_acceleration * self.timer_dt
            if out_vel > set_vel:
                out_vel = set_vel
        else:   # if the velocity is negative
            out_vel -= self.max_linear_acceleration * self.timer_dt
            if out_vel < set_vel:
                out_vel = set_vel
        return out_vel
    
    def update_angular(self, set_vel, out_vel):
        # if the current velocity is smaller than the target velocity
        if out_vel < set_vel:   # if the current velocity is smaller (positive)
            out_vel += self.max_angular_acceleration * self.timer_dt
            if out_vel > set_vel:
                out_vel = set_vel
        else:   # if the velocity is negative
            out_vel -= self.max_angular_acceleration * self.timer_dt
            if out_vel < set_vel:
                out_vel = set_vel
        return out_vel

    def cmd_vel_cb(self, msg):
        self.set_velocity = msg

def main(args=None):
    rclpy.init(args=args)
    smoother = Smoother()
    rclpy.spin(smoother)
    rclpy.shutdown()


if __name__ == '__main__':
    main()