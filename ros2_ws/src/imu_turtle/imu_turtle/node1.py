import sys;
#import serial;
import re;
import rclpy;
from rclpy.node import Node;
#sys.path.insert(0, '<your_path>/imu_turtle/ros2_ws/src');
import serial as serial0; #for some reason, rosdep no longer complains about importing the installed package? no local copy needed.. (2022-12-22)
from quaternion_interface.msg import Quaternion;

class QuaternionPublisher(Node):

    def __init__(self):
        super().__init__('quaternion_publisher');
        self.publisher_ = self.create_publisher(Quaternion, 'topic1', 10);
        self.ser = serial0.Serial('/dev/ttyACM0', 9600, timeout=1); #change to correct port
        self.ser.reset_input_buffer();
        timer_period = 0.05;  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback);
        self.pat = re.compile(r'-?0\.\d+');

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('ascii').rstrip();
            try:
                qvals = self.pat.findall(line);
                qw = float(qvals[0]);
                qx = float(qvals[1]);
                qy = float(qvals[2]);
                qz = float(qvals[3]);
                msg = Quaternion();
                msg.qw = qw;
                msg.qx = qx;
                msg.qy = qy;
                msg.qz = qz;
                self.publisher_.publish(msg);
                self.get_logger().info('qw: {}, qx: {}, qy: {}, qz:{};'.format(msg.qw,msg.qx,msg.qy,msg.qz));
            except:
                self.get_logger().info('data not found;');



def main(args=None):
    rclpy.init(args=args);

    quaternion_publisher = QuaternionPublisher();

    rclpy.spin(quaternion_publisher);

    quaternion_publisher.destroy_node();
    rclpy.shutdown();


if __name__ == '__main__':
    main()

