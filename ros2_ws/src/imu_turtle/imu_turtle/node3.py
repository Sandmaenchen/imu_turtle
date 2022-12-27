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

        self.quat0a = np.quaternion(1,0,0,0); #reference quaternion; handled in callback groups 1 and 2
        self.quat1a = np.quaternion(1,0,0,0); #current quaternion; handled in callback groups 1 and 2
        self.quat0b = np.quaternion(1,0,0,0); #reference quaternion; handled in callback groups 3 and 4
        self.quat1b = np.quaternion(1,0,0,0); #current quaternion; handled in callback groups 3 and 4
        self.x_vel_default = 2.0;
        self.y_vel_default = 2.0;
        self.z_ang_default = 1.0;
        self.velocity = Twist();
        self.velocity.linear.x = 0.0;
        self.velocity.linear.y = 0.0;
        self.velocity.angular.z = 0.0;
        
        self.callback_group1 = rclpy.callback_groups.MutuallyExclusiveCallbackGroup();
        self.callback_group2 = rclpy.callback_groups.ReentrantCallbackGroup();
        self.callback_group3 = rclpy.callback_groups.MutuallyExclusiveCallbackGroup();
        self.callback_group4 = rclpy.callback_groups.MutuallyExclusiveCallbackGroup();
        
        self.publisher0a_ = self.create_publisher(Quaternion, 'topic0a', qos_profile=10, callback_group=self.callback_group2);
        self.publisher0b_ = self.create_publisher(Quaternion, 'topic0b', qos_profile=10, callback_group=self.callback_group2);
        self.publisher_ = self.create_publisher(Twist, 'turtlesim1/turtle1/cmd_vel', qos_profile=10, callback_group=self.callback_group4);

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
        
        self.subscription0a = self.create_subscription(
            Quaternion,
            'topic0a',
            self.topic0a_callback,
            10, callback_group=self.callback_group3);
        self.subscription0a;
        
        self.subscription0b = self.create_subscription(
            Quaternion,
            'topic0b',
            self.topic0b_callback,
            10, callback_group=self.callback_group3);
        self.subscription0b;
        

    def topic1_callback(self, msg):
        self.quat1a.w; = msg.qw;
        self.quat1a.x; = msg.qx;
        self.quat1a.y; = msg.qy;
        self.quat1a.z; = msg.qz;
        self.publisher0a_.publish(msg);

    def topic2_callback(self, msg):
        msg_b = Quaternion();
        msg_b.qw; = self.quat1a.w;
        msg_b.qx; = self.quat1a.x;
        msg_b.qy; = self.quat1a.y;
        msg_b.qz; = self.quat1a.z;
        self.publisher0b_.publish(msg_b);
        
    def topic0a_callback(self,msg):
        self.quat1b.w; = msg.qw;
        self.quat1b.x; = msg.qx;
        self.quat1b.y; = msg.qy;
        self.quat1b.z; = msg.qz;      
        compute_velocity();
        publish_velocity_input();
        
    def topic0b_callback(self,msg):
        self.quat0b.w; = msg.qw;
        self.quat0b.x; = msg.qx;
        self.quat0b.y; = msg.qy;
        self.quat0b.z; = msg.qz;
        

    def publish_velocity_input(self):
        self.publisher_.publish(self.velocity);
        self.get_logger().info(f"Publishing velocity: \n\t linear.x: {self.velocity.linear.x};" +
        f"\n\t linear.y: {self.velocity.linear.y};" +
        f"\n\t angular.z: {self.velocity.angular.z}");

    def compute_velocity(self):
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


def main(args=None):
    rclpy.init(args=args);
    
    executor = rclpy.executors.MultiThreadedExecutor();

    turtle_controller = TurtleController();

    rclpy.spin(turtle_controller,executor);

    turtle_controller.destroy_node();
    rclpy.shutdown();


if __name__ == '__main__':
    main()