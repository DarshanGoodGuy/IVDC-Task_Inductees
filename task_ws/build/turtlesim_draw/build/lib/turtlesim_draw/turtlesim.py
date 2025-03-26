import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from turtlesim.msg import Pose
import math
import time

DEFAULT_PEN_R = 0xb3
DEFAULT_PEN_G = 0xb8
DEFAULT_PEN_B = 0xff
DEFAULT_PEN_WIDTH = 3
LINEAR_SPEED = 1.5
ANGULAR_SPEED = 2.0
POSITION_TOLERANCE = 0.1
ANGLE_TOLERANCE = 0.05

class TurtleDraw(Node):
    def __init__(self):
        super().__init__('drone_drawing')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.g_pose = Pose()
        time.sleep(1)
        self.draw_drone()

    def pose_callback(self, pose):
        self.g_pose = pose

    def normalize_angle(self, angle):
        return ((angle + math.pi) % (2 * math.pi)) - math.pi

    def pen_control(self, pen_down):
        request = SetPen.Request()
        request.r = DEFAULT_PEN_R
        request.g = DEFAULT_PEN_G
        request.b = DEFAULT_PEN_B
        request.width = DEFAULT_PEN_WIDTH
        request.off = 0 if pen_down else 1
        self.pen_client.call_async(request)

    def rotate_to_angle(self, target_angle):
        cmd_vel = Twist()
        while rclpy.ok():
            current_angle = self.g_pose.theta
            angle_diff = self.normalize_angle(target_angle - current_angle)
            
            if abs(angle_diff) < ANGLE_TOLERANCE:
                break
                
            cmd_vel.angular.z = ANGULAR_SPEED * angle_diff
            self.publisher_.publish(cmd_vel)
            time.sleep(0.05)
            
        cmd_vel.angular.z = 0.0
        self.publisher_.publish(cmd_vel)

    def move_to_position(self, target_x, target_y):
        cmd_vel = Twist()
        while rclpy.ok():
            current_x = self.g_pose.x
            current_y = self.g_pose.y
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.hypot(dx, dy)
            
            if distance < POSITION_TOLERANCE:
                break
                
            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - self.g_pose.theta)
            
            cmd_vel.linear.x = LINEAR_SPEED * distance
            cmd_vel.angular.z = ANGULAR_SPEED * angle_diff
            self.publisher_.publish(cmd_vel)
            time.sleep(0.05)
            
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.publisher_.publish(cmd_vel)

    def draw_line(self, x1, y1, x2, y2):
        self.pen_control(False)
        self.move_to_position(x1, y1)
        self.rotate_to_angle(math.atan2(y2-y1, x2-x1))
        self.pen_control(True)
        self.move_to_position(x2, y2)
        self.pen_control(False)

    def draw_square(self):
        vertices = [(3.0, 5.0), (5.0, 7.0), (7.0, 5.0), (5.0, 3.0)]
        for i in range(len(vertices)):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i+1)%len(vertices)]
            self.draw_line(x1, y1, x2, y2)

    def draw_circle(self, center_x, center_y, radius=1.0):
        self.pen_control(False)
        self.move_to_position(center_x + radius, center_y)
        self.rotate_to_angle(math.pi/2)
        self.pen_control(True)
        
        cmd_vel = Twist()
        cmd_vel.linear.x = LINEAR_SPEED
        cmd_vel.angular.z = LINEAR_SPEED / radius
        
        circle_time = 2 * math.pi * radius / LINEAR_SPEED
        start_time = time.time()
        while (time.time() - start_time) < circle_time:
            self.publisher_.publish(cmd_vel)
            time.sleep(0.05)
        
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.publisher_.publish(cmd_vel)
        self.pen_control(False)

    def draw_drone(self):
        self.draw_square()
        self.draw_line(4.0, 6.0, 2.0, 8.0)
        self.draw_line(6.0, 6.0, 8.0, 8.0)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleDraw()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
