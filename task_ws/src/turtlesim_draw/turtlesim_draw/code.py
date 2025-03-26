import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, Spawn, Kill
import math

class TurtleDraw(Node):
    def __init__(self):
        super().__init__('turtle_draw')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0  # radians, facing right
        self.pen_is_down = True
        self.pose_received = False
    
    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta
        self.pose_received = True
    
    def set_pen(self, r=255, g=255, b=255, width=2, off=False):
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for pen service...')
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self.pen_is_down = not off
        self.pen_client.call_async(req)

    def draw_line(self, end_x, end_y, speed=1.0):
        while not self.pose_received:
            rclpy.spin_once(self)
        
        dx = end_x - self.current_x
        dy = end_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)

        #calculate required rotation
        angle_diff = target_angle - self.current_theta
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        #rotate to face target
        if abs(angle_diff) > 1e-3:
            self._rotate(angle_diff, target_angle, speed)

        #Move forward
        self._move_straight(distance, speed)

    def draw_circle(self, center_x, center_y, radius, speed=1.0):
        """Draw circle around specified center coordinates with given radius"""
        was_pen_down = self.pen_is_down
        
        while not self.pose_received:
            rclpy.spin_once(self)

        # Move to the correct starting position 
        
        start_x = center_x + radius
        start_y = center_y
        self.pen_up()
        self.draw_line(start_x, start_y)  # Move to the start point
        self.pen_down()

        # Rotate the turtle to face the center
        self._rotate(math.pi / 2, self.current_theta + math.pi / 2, speed)

        # Execute movement
        msg = Twist()
        msg.linear.x = speed  # Forward speed
        msg.angular.z = speed / radius  # Angular velocity

        # Compute time required for a full circle
        circle_time = (2 * math.pi * radius) / speed
        start_time = self.get_clock().now().seconds_nanoseconds()[0]

        while (self.get_clock().now().seconds_nanoseconds()[0] - start_time) < circle_time:
            self.publisher_.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Stop movement
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
    
    def _rotate(self,angle_diff, target_angle, speed):
        
        while not self.pose_received:
            rclpy.spin_once(self)
        
        msg = Twist()
        msg.angular.z = speed if angle_diff > 0 else -speed
        
        while abs(target_angle - self.current_theta) > 0.01:
            self.publisher_.publish(msg)
            rclpy.spin_once(self)
        
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
    
    def _move_straight(self, distance, speed):
        while not self.pose_received:
            rclpy.spin_once(self)
        
        start_x, start_y = self.current_x, self.current_y
        msg = Twist()
        msg.linear.x = speed if distance > 0 else -speed
        
        while math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2) < distance:
            self.publisher_.publish(msg)
            rclpy.spin_once(self)
        
        msg.linear.x = 0.0
        self.publisher_.publish(msg)
    
    def pen_up(self):
        self.set_pen(off=True)
    
    def pen_down(self):
        self.set_pen(off=False)

def main(args=None):
    rclpy.init(args=args)
    turtle_draw = TurtleDraw()
    rclpy.spin_once(turtle_draw)
    
    turtle_draw.pen_up()
    turtle_draw.draw_line(5.0, 7.0)
    turtle_draw.pen_down()
    turtle_draw.draw_line(7.0, 5.0)
    turtle_draw.draw_line(5.0, 3.0)
    turtle_draw.draw_line(3.0, 5.0)
    turtle_draw.draw_line(5.0, 7.0)
    turtle_draw.pen_up()
    turtle_draw.draw_line(6.0, 6.0)
    turtle_draw.pen_down()
    turtle_draw.draw_line(8.0, 8.0)
    turtle_draw.draw_circle(8.0 , 8.0, 1.0)
    turtle_draw.pen_up()
    turtle_draw.draw_line(6.0, 4.0)
    turtle_draw.pen_down()
    turtle_draw.draw_line(8.0, 2.0)
    turtle_draw.draw_circle(8.0, 2.0, 1.0)
    turtle_draw.pen_up()
    turtle_draw.draw_line(4.0, 4.0)
    turtle_draw.pen_down()
    turtle_draw.draw_line(2.0, 2.0)
    turtle_draw.draw_circle(2.0, 2.0, 1.0)
    turtle_draw.pen_up()
    turtle_draw.draw_line(4.0, 6.0)
    turtle_draw.pen_down()
    turtle_draw.draw_line(2.0, 8.0)
    turtle_draw.draw_circle(2.0, 8.0, 1.0)
    

    
    turtle_draw.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

