"""测试用脚本对手；只控制 test_opponent，不给己方控制器真值。"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class Opponent(Node):
    def __init__(self, mode):
        super().__init__('test_opponent_policy')
        self.mode = mode
        self.pose = None
        self.create_subscription(Odometry, '/opponent/odom', self.observe, 1)
        self.pub = self.create_publisher(Twist, '/opponent/cmd_vel', 1)
        self.create_timer(.05, self.tick)
        self.index = 0
        self.start = None

    def observe(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def tick(self):
        m = Twist()
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.pose and self.mode != 'block' and now<90:
            if self.start is None: self.start = now
            x, y, yaw = self.pose
            if self.mode == 'cross':
                targets = [(-.36,.04),(.36,.04)]
                gx, gy = targets[self.index % len(targets)]
                gyaw = -math.pi/2
            else:
                # 将中线附近新到黄区的物块重新扫回；在不同列重复。
                columns = [0.0, -.25, .25]
                column = columns[(self.index // 3) % len(columns)]
                targets = [(column,.34), (column,-.30), (column,.34)]
                gx, gy = targets[self.index % 3]
                gyaw = -math.pi/2
            if math.hypot(gx-x,gy-y) < .035: self.index += 1
            dx, dy = gx-x, gy-y
            dist = math.hypot(dx,dy)
            speed = min(.20, 1.5*dist)
            vx, vy = speed*dx/max(.001,dist), speed*dy/max(.001,dist)
            m.linear.x = math.cos(yaw)*vx+math.sin(yaw)*vy
            m.linear.y = -math.sin(yaw)*vx+math.cos(yaw)*vy
            m.angular.z = max(-.5,min(.5,1.5*math.atan2(math.sin(gyaw-yaw),math.cos(gyaw-yaw))))
        self.pub.publish(m)


def main():
    import sys
    mode = sys.argv[1]
    rclpy.init(args=[])
    n = Opponent(mode)
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): n.pub.publish(Twist())
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
