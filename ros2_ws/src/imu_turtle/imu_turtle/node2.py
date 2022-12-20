from pynput import keyboard;
import rclpy;
from rclpy.node import Node;
from std_msgs.msg import String;

class KeyboardPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_publisher');
        self.publisher_ = self.create_publisher(String, 'topic2', 10);
        self.msg = String();
        self.msg.data = 'Recalibrate';

    def spin(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            #listener.start(); #commented; listener apparently gets started somewhere 'earlier'
            while rclpy.ok() and listener.running:
                rclpy.spin_once(self, timeout_sec=0.1);
    
    def on_press(self, key):
        if key == keyboard.Key.space:
            self.publisher_.publish(self.msg);
            self.get_logger().info('Publishing: "%s"' % self.msg.data);

    def on_release(self, key):
        pass;

def main(args=None):
    rclpy.init(args=args)

    KeyboardPublisher().spin();

    rclpy.shutdown();


if __name__ == '__main__':
    main()






