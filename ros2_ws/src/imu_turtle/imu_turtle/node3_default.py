import numpy as np;
import quaternion;
import rclpy;
from rclpy.node import Node;
from quaternion_interface.msg import Quaternion;
from std_msgs.msg import String;
from geometry_msgs.msg import Twist;

# This is my initial solution. This uses SingleThreadExecutor by default, not good
# In practice, this solution leads to situations where topic2_callback does not get executed
# because the only thread is executing other callback functions while keyboard is pressed

class TurtleController(Node):

    def __init__(self):
        super().__init__('turtle_controller');

        self.quat0 = np.quaternion(1,0,0,0); #reference quaternion
        self.quat1 = np.quaternion(1,0,0,0); #current quaternion
        self.x_vel_default = 2.0;
        self.y_vel_default = 2.0;
        self.z_ang_default = 1.0;
        self.velocity = Twist();
        self.velocity.linear.x = 0.0;
        self.velocity.linear.y = 0.0;
        self.velocity.angular.z = 0.0;

        self.publisher_ = self.create_publisher(Twist, 'turtlesim1/turtle1/cmd_vel', 10);
        timer_period = 0.1;  # seconds
        self.timer = self.create_timer(timer_period, self.publish_velocity_input);

        self.subscription1 = self.create_subscription(
            Quaternion,
            'topic1',
            self.topic1_callback,
            10);
        self.subscription1;

        self.subscription2 = self.create_subscription(
            String,
            'topic2',
            self.topic2_callback,
            10);
        self.subscription2;

    def topic1_callback(self, msg):
        self.quat1 = np.quaternion(msg.qw,msg.qx,msg.qy,msg.qz);
        self.compute_velocity();

    def topic2_callback(self, msg):
        self.quat0 = np.quaternion(self.quat1.w,self.quat1.x,self.quat1.y,self.quat1.z);
        self.compute_velocity();

    def publish_velocity_input(self):
        self.publisher_.publish(self.velocity);
        self.get_logger().info(f"Publishing velocity: \n\t linear.x: {self.velocity.linear.x};" +
        f"\n\t linear.y: {self.velocity.linear.y};" +
        f"\n\t angular.z: {self.velocity.angular.z}");

    def compute_velocity(self):
        try: #just in case IMU outputs zero-quaternion (division by zero..)
            q0 = self.quat0/np.sqrt(self.quat0.norm()); #quaternion.norm() actually produces SQUARE of the Euclidean norm!
            q1 = self.quat1/np.sqrt(self.quat1.norm()); #quaternion.norm() actually produces SQUARE of the Euclidean norm!
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

    turtle_controller = TurtleController();

    rclpy.spin(turtle_controller);

    turtle_controller.destroy_node();
    rclpy.shutdown();


if __name__ == '__main__':
    main()