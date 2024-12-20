#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from kobuki_ros_interfaces.msg import BumperEvent
import time

print("bumping :: START...")

def start():
    # Initialize ROS node with ROS client
    rclpy.init()
    aNode = Node("bumping")
    talker = Robot(aNode)

    # Start infinite loop
    rclpy.spin(aNode)

    # Clean everything and switch the light off
    aNode.destroy_node()
    rclpy.shutdown()

class Robot:
    def __init__(self, rosNode):
        # PUBLISHER
        self._publisher = rosNode.create_publisher(Twist, '/multi/cmd_nav', 10)
        
        # # SUBSCRIBER
        self._bumper_state = None
        self._bumper_subscriber = rosNode.create_subscription(BumperEvent, '/events/bumper', self.bumper_callback, 10)
        self._timer = rosNode.create_timer(0.1, self.timer_callback)
        #self.loop(rosNode)
        
    def bumper_callback(self, msg):
        # This function is called when a bumper event is received
        print(f"Bumper Event Received: {msg}")
        
        if msg.bumper == BumperEvent.LEFT:
            self._bumper_state = 0
        elif msg.bumper == BumperEvent.CENTER:
            self._bumper_state = 1
        elif msg.bumper == BumperEvent.RIGHT:
            self._bumper_state = 2

    def timer_callback(self):
        print("callback")
        if self._bumper_state is None:
            self.move(0.3, 0.0)
            print("move")
        else:
            self.stop()
            print("stop")
            self.move(-0.3, 1.0)
            time.sleep(1)
            self._bumper_state = None
            print("resume")
        print("")


    def move(self, x, ang):
        velocity = Twist()
        velocity.linear.x = x
        velocity.angular.z = ang
        self._publisher.publish(velocity)
    
    def stop(self):
        self._stop = Twist()
        self._publisher.publish(self._stop)

# Execute the function.
if __name__ == "__main__":
    start()

