import imp
from motor_control_py.roboclaw_3 import Roboclaw
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


class Motor_Controller:

        def __init__(self, motor_id, dev_name, encoder_ticks, ):
               self.motor_id = motor_id
               self.device_name = dev_name
               self.parameters ={} 



class Motor_Controller_Node(Node):
        def __init__(self):
                super().__init__("Motor_Control")
                self.velocity_publisher = self.create_publisher(UInt8,'velocity',10)
                timer_period = 1 # seconds
                self.timer = self.create_timer(timer_period, self.timer_callback)
                self.i = 0

        def motor_test(self):
                
                for x in range (5):
                        msg = UInt8()
                        msg.data = x
                        self.velocity_publisher.publish(msg)
                        self.get_logger().info('Publishing: "%u" Index: "%d"' % (msg.data, self.i))

        def timer_callback(self):
                self.motor_test()
                self.i += 1 


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Motor_Controller_Node()

    
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()