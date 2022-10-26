
from turtle import speed
from motor_control_py.roboclaw_3 import Roboclaw
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from functools import partial


update_frequency_hz = 1
address = 0x80
class Motor_Controller:

        def __init__(self, motor_id):
               self.motor_id = motor_id
               self.parameters_dic ={
               self.motor_id+'_dev_name': None ,
               self.motor_id+ '_baudrate':None,
               self.motor_id + '_ticks_per_rotation': None
               } 

               



class Motor_Controller_Node(Node):
        def __init__(self,motor_list):
                super().__init__("Motor_Control")
                
                #Initialize motor controllers
                for motor in motor_list:
                        #Declare Parameters for each parameter in parameter dictionary 
                        for key in motor.parameters_dic:
                                self.declare_parameter(key,None)      
                                # Update parameters dictionary according to Config file
                                motor.parameters_dic[key] = self.get_parameter(key)
                        
                        
                        #  Roboclaw Definition
                        dev = str(motor.parameters_dic[motor.motor_id + '_dev_name'].value)
                        baudrate = motor.parameters_dic[motor.motor_id + '_baudrate'].value
                        print(dev,'__',baudrate)
                        motor.roboclaw_obj = Roboclaw(dev,baudrate)
                       
                        motor.roboclaw_obj.Open()
                        
                        # Create Publishers
                        motor.speed_publisher = self.create_publisher(Int64,motor.motor_id+'_speed',10)
                        motor.encoder_publisher = self.create_publisher(Int64,motor.motor_id+'_encoder',10)

                        #Set Up Publisher Timers 
                        speed_pub_partial = partial(self.speed_pub_timer_callback,p_motor= motor)
                        motor.speed_pub_timer = self.create_timer(1/float(update_frequency_hz), speed_pub_partial)
                        
                        encoder_pub_partial = partial(self.encoder_pub_timer_callback,p_motor= motor)
                        motor.encoder_pub_timer = self.create_timer(1/float(update_frequency_hz), encoder_pub_partial)
                        
                        #Create Subscribers
                        set_speed_partial = partial(self.set_speed_subscriber_callback,p_motor = motor)
                        motor.set_speed_subscriber = self.create_subscription(Int64,motor.motor_id + '_set_speed', set_speed_partial,10)

                        
        
                        
                

        

        def speed_pub_timer_callback(self,p_motor):
                speed1 = p_motor.roboclaw_obj.ReadSpeedM1(address)
                if(speed1[0]==1):
                        msg = Int64()
                        msg.data = speed1[1]
                        p_motor.speed_publisher.publish(msg)
                        #self.get_logger().info(p_motor.motor_id + ": publishing speed: %d" %msg.data )
                else:
                        self.get_logger().error('Error Reading Speed: ' + p_motor.motor_id)

        def encoder_pub_timer_callback(self,p_motor):
                enc1 = p_motor.roboclaw_obj.ReadEncM1(address)
                if(enc1[0]==1):
                        msg = Int64()
                        msg.data = enc1[1]
                        p_motor.encoder_publisher.publish(msg)
                        #self.get_logger().info(p_motor.motor_id + ": publishing encoder: %d" %msg.data )
                else:
                        self.get_logger().error('Error Reading Encoder: ' + p_motor.motor_id)
                

        def set_speed_subscriber_callback(self,msg,p_motor):
                
                speed_t = msg.data
                p_motor.roboclaw_obj.SpeedM1(address,speed_t)
                self.get_logger().info('MSG: "%d" ' % msg.data + "Motor: "+ p_motor.motor_id)
                

def main(args=None):
    rclpy.init(args=args)
    motorFR = Motor_Controller('FR')
    motorFL = Motor_Controller('FL')
    motor_list = [motorFR,motorFL]
    minimal_publisher = Motor_Controller_Node(motor_list)

    
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()