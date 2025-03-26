import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, Spawn, Kill
import math

class TurtleDraw(Node):
    def __init__(self):
        super().__init__('turtle_draw')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')    
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0.0  # radians, facing right
        self.pen_is_down = True

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
        dx = end_x - self.current_x
        dy = end_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        angle_diff = target_angle - self.current_theta
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  
        
        if abs(angle_diff) > 1e-3:
            self._rotate(angle_diff)
        
        self._move_straight(distance, speed)
        
        self.current_x = end_x
        self.current_y = end_y
        self.current_theta = target_angle

    def draw_circle(self, center_x, center_y, radius, speed=1.0):
        was_pen_down = self.pen_is_down
        self.set_pen(off=not was_pen_down)
        
        angular_speed = speed / radius
        duration = 2 * math.pi * radius / speed
        
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = angular_speed
        self.publisher_.publish(msg)
        rclpy.spin_once(self, timeout_sec=duration)
        
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        
    def _rotate(self, angle):
        msg = Twist()
        speed = 1.0
        direction = 1 if angle > 0 else -1
        msg.angular.z = direction * speed
        duration = abs(angle) / speed
        
        self.publisher_.publish(msg)
        rclpy.spin_once(self, timeout_sec=duration)
        
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.current_theta += angle
        self.current_theta = (self.current_theta + math.pi) % (2 * math.pi) - math.pi

    def _move_straight(self, distance, speed):
        msg = Twist()
        msg.linear.x = speed if distance > 0 else -speed
        duration = abs(distance) / speed
        
        self.publisher_.publish(msg)
        rclpy.spin_once(self, timeout_sec=duration)
        
        msg.linear.x = 0.0
        self.publisher_.publish(msg)

    def pen_up(self):
        self.set_pen(off=True)

    def pen_down(self):
        self.set_pen(off=False)


def main(args=None):
    rclpy.init(args=args)
    turtle_draw = TurtleDraw()
    
    turtle_draw.pen_down()
    turtle_draw.draw_line(5, 5)  # Example line drawing
    turtle_draw.pen_up()
    turtle_draw.draw_circle(4, 4, 2)
    
    turtle_draw.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

