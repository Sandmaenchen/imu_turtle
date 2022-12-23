import numpy as np;
import quaternion;
import time;
import rclpy;
from rclpy.node import Node;
from quaternion_interface.msg import Quaternion;
from std_msgs.msg import String;
from geometry_msgs.msg import Twist;

class TurtleController(Node):

    def __init__(self):
        super().__init__('turtle_controller');

        self.quat0a = np.quaternion(1,0,0,0); #reference quaternion
        self.quat1a = np.quaternion(1,0,0,0); #current quaternion
        self.quat0b = np.quaternion(1,0,0,0);
        self.quat1b = np.quaternion(1,0,0,0);
        self.x_vel_default = 2.0;
        self.y_vel_default = 2.0;
        self.z_ang_default = 1.0;
        self.velocity = Twist();
        self.velocity.linear.x = 0.0;
        self.velocity.linear.y = 0.0;
        self.velocity.angular.z = 0.0;
        
        self.callback_group1 = rclpy.callback_groups.MutuallyExclusiveCallbackGroup();
        self.callback_group2 = rclpy.callback_groups.MutuallyExclusiveCallbackGroup();
        self.callback_group3 = rclpy.callback_groups.MutuallyExclusiveCallbackGroup();

        self.publisher_ = self.create_publisher(Twist, 'turtlesim1/turtle1/cmd_vel', qos_profile=10, callback_group=self.callback_group3);
        timer_period = 0.1;  # seconds
        timer_period_quick = 0.05; #seconds
        self.timer_publish = self.create_timer(timer_period, self.publish_velocity_input, callback_group=self.callback_group2);
        self.timer_compute = self.create_timer(timer_period, self.compute_velocity, callback_group=self.callback_group2);
        self.timer_copy = self.create_timer(timer_period_quick, self.copy_quaternions, callback_group=self.callback_group2);

        self.subscription1 = self.create_subscription(
            Quaternion,
            'topic1',
            self.topic1_callback,
            10, callback_group=self.callback_group1);
        self.subscription1;

        self.subscription2 = self.create_subscription(
            String,
            'topic2',
            self.topic2_callback,
            10, callback_group=self.callback_group1);
        self.subscription2;

    def topic1_callback(self, msg):
        while not (self.callback_group1.beginning_execution(self.subscription1)):
            time.sleep(0.0001);
        self.quat1a = np.quaternion(msg.qw,msg.qx,msg.qy,msg.qz);
        self.callback_group1.ending_execution(self.subscription1);

    def topic2_callback(self, msg):
        while not (self.callback_group1.beginning_execution(self.subscription2)):
            time.sleep(0.0001);
        self.quat0a = np.copy(self.quat1a);
        self.callback_group1.ending_execution(self.subscription2);
        
    def copy_quaternions(self):
        while not (self.callback_group2.beginning_execution(self.timer_copy)):
            time.sleep(0.0001);
        self.quat0b = np.copy(self.quat0a);
        self.quat1b = np.copy(self.quat1a);
        self.callback_group2.ending_execution(self.timer_copy);

    def publish_velocity_input(self):
        while not (self.callback_group2.beginning_execution(self.timer_publish)):
            time.sleep(0.0001);
        self.publisher_.publish(self.velocity);
        self.get_logger().info(f"Publishing velocity: \n\t linear.x: {self.velocity.linear.x};" +
        f"\n\t linear.y: {self.velocity.linear.y};" +
        f"\n\t angular.z: {self.velocity.angular.z}");
        self.callback_group2.ending_execution(self.timer_publish);

    def compute_velocity(self):
        while not (self.callback_group2.beginning_execution(self.timer_compute)):
            time.sleep(0.0001);
        try: #just in case IMU outputs zero-quaternion (division by zero..)
            q0 = self.quat0b/np.sqrt(self.quat0b.norm()); #quaternion.norm() actually produces SQUARE of the Euclidean norm!
            q1 = self.quat1b/np.sqrt(self.quat1b.norm()); #quaternion.norm() actually produces SQUARE of the Euclidean norm!
            #qr = np.multiply(q1,q0.conjugate()); #use this if q0 is not the reference orientation
            qr = np.multiply(q0.conjugate(),q1); #use this if q0 is the reference orientation

            xangle = np.arctan2((2*qr.y*qr.z + 2*qr.w*qr.x),(2*pow(qr.w,2) + 2*pow(qr.z,2) - 1)) * 180 / np.pi;
            yangle = -1*np.arcsin(2*qr.x*qr.z - 2*qr.w*qr.y) * 180 / np.pi;
            zangle = np.arctan2((2*qr.x*qr.y + 2*qr.w*qr.z),(2*pow(qr.w,2) + 2*pow(qr.x,2) - 1)) * 180 /np.pi;

            self.velocity.linear.x = -1 * self.x_vel_default * xangle / 90; 
            self.velocity.linear.y = 1 * self.y_vel_default * yangle / 90; #should not be -1
            self.velocity.angular.z = 1 * self.z_ang_default * zangle / 90; #should not be -1
        except:
            pass;
        self.callback_group2.ending_execution(self.timer_compute);
            


def main(args=None):
    rclpy.init(args=args);
    
    executor = rclpy.executors.MultiThreadedExecutor();

    turtle_controller = TurtleController();

    rclpy.spin(turtle_controller,executor);

    turtle_controller.destroy_node();
    rclpy.shutdown();


if __name__ == '__main__':
    main()