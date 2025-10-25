import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from my_can_msgs.msg import Frame
import random


class CANPublisher(Node):

    def __init__(self):
        super().__init__('can_publisher')
        self.publisher_ = self.create_publisher(Frame, 'can_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        msg = Frame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "can_bus_0"
        msg.id = 0x100 + self.i  # fake CAN ID
        msg.is_rtr = False
        msg.is_extended = False
        msg.is_error = False
        msg.dlc = 8
        # generate 8 random bytes
        msg.data = [random.randint(0, 255) for _ in range(8)]

        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing CAN Frame: id={msg.id}, data={msg.data}')
        self.i += 1



def main(args=None):
    rclpy.init(args=args)

    can_publisher = CANPublisher()

    rclpy.spin(can_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()