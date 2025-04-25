#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import Jetson.GPIO as GPIO

MOTOR_PIN = 32  
SERVO_PIN = 15 

class PWMController(Node):
    def __init__(self):
        super().__init__('pwm_controller')

        self.subscription = self.create_subscription(
            Twist, 
            '/cmd_vel',  
            self.cmd_vel_callback, 
            10 
        )

        self.servo_pwm_pub = self.create_publisher(Float32, '/servo_pwm', 10)

        self.get_logger().info("Resetting GPIO state...")
        GPIO.setwarnings(False)  

        self.get_logger().info("Initializing GPIO...")
        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(MOTOR_PIN, GPIO.OUT, initial=GPIO.HIGH)
        self.motor_pwm = GPIO.PWM(MOTOR_PIN, 50)  # 50Hz for motor ESC
        self.motor_pwm.start(18/256)  

        GPIO.setup(SERVO_PIN, GPIO.OUT, initial=GPIO.HIGH)
        self.servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz for servo
        self.servo_pwm.start(6)  

    def cmd_vel_callback(self, msg):
        motor_percentage = (msg.linear.x + 1) / 2 * 100  
        servo_percentage = ((msg.angular.z + 1) / 2) * 100 

        motor_pwm = 13 + (motor_percentage / 100) * (24 - 13)  
        servo_pwm = 3 + (servo_percentage / 100) * (9 - 3)  

        min_motor_duty = (13 / 256) * 100  
        mid_motor_duty = (18 / 256) * 100  
        max_motor_duty = (24 / 256) * 100  

        min_servo_duty = 9 
        max_servo_duty = 3  

        if msg.linear.x == 0:
            motor_duty_cycle = mid_motor_duty  
        else:
            motor_duty_cycle = min_motor_duty + ((motor_pwm - 13) / (24 - 13)) * (max_motor_duty - min_motor_duty)

        servo_duty_cycle = min_servo_duty + ((servo_pwm - 3) / (9 - 3)) * (max_servo_duty - min_servo_duty)

        self.motor_pwm.ChangeDutyCycle(motor_duty_cycle)
        self.servo_pwm.ChangeDutyCycle(servo_duty_cycle)

        # Publish the servo PWM value
        self.servo_pwm_pub.publish(Float32(data = servo_duty_cycle))

    def cleanup(self):
        self.motor_pwm.stop()
        self.servo_pwm.stop()
        GPIO.cleanup()
        self.get_logger().info("Cleaned up GPIO")

def main(args=None):
    rclpy.init(args=args)
    node = PWMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
