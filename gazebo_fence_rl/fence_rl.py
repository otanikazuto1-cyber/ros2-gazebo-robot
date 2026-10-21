#!/usr/bin/env python3
"""Offline Q-learning for a known square arena; ROS 2 runtime with a safety filter."""
import argparse
import math
import os
import random
from pathlib import Path
import numpy as np

HALF = 3.0  # centerline of each wall; inner wall face at 2.9
SAFE = 2.35  # conservative allowable center region
RADIUS = 0.35
GOALS = [(1.8, 1.8), (-1.8, 1.8), (-1.8, -1.8), (1.8, -1.8)]
DT = 0.2
# linear m/s, angular rad/s
ACTIONS = [(0.22, 0), (0.17, 0.85), (0.17, -0.85), (0, 1.1), (0, -1.1)]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clearance(x, y):
    return min(SAFE - abs(x), SAFE - abs(y))


def ray_dist(x, y, angle):
    dx, dy = math.cos(angle), math.sin(angle)
    ds = []
    if abs(dx) > 1e-9:
        ds += [(SAFE - x) / dx, (-SAFE - x) / dx]
    if abs(dy) > 1e-9:
        ds += [(SAFE - y) / dy, (-SAFE - y) / dy]
    return min((d for d in ds if d >= 0), default=0.0)


def state(x, y, yaw, goal):
    a = wrap(math.atan2(goal[1] - y, goal[0] - x) - yaw)
    angle_bin = min(7, int((a + math.pi) / (2 * math.pi) * 8))
    front = ray_dist(x, y, yaw)
    left = ray_dist(x, y, yaw + math.pi / 3)
    right = ray_dist(x, y, yaw - math.pi / 3)
    def bucket(d):
        return 0 if d < 0.55 else 1 if d < 1.1 else 2
    return angle_bin, bucket(front), bucket(left), bucket(right)


def step(x, y, yaw, action):
    v, w = ACTIONS[action]
    new_yaw = wrap(yaw + w * DT)
    heading = yaw + w * DT / 2
    return x + v * DT * math.cos(heading), y + v * DT * math.sin(heading), new_yaw


def train(path, episodes=7000, seed=42):
    random.seed(seed)
    rng = np.random.default_rng(seed)
    q = np.zeros((8, 3, 3, 3, len(ACTIONS)), dtype=np.float32)
    for ep in range(episodes):
        x, y = rng.uniform(-1.9, 1.9, size=2)
        yaw = rng.uniform(-math.pi, math.pi)
        goal = GOALS[int(rng.integers(len(GOALS)))]
        eps = max(0.06, 0.65 * (1 - ep / episodes))
        for _ in range(200):
            s = state(x, y, yaw, goal)
            action = random.randrange(len(ACTIONS)) if random.random() < eps else int(np.argmax(q[s]))
            nx, ny, nyaw = step(x, y, yaw, action)
            old_d = math.hypot(goal[0] - x, goal[1] - y)
            new_d = math.hypot(goal[0] - nx, goal[1] - ny)
            collided = abs(nx) >= SAFE or abs(ny) >= SAFE
            reached = new_d < 0.35
            reward = 8 * (old_d - new_d) - 0.015
            if clearance(nx, ny) < 0.45:
                reward -= 0.08
            if collided:
                reward -= 12
            if reached:
                reward += 12
            next_s = state(nx, ny, nyaw, goal) if not collided else s
            target = reward + (0 if collided or reached else 0.96 * float(np.max(q[next_s])))
            q[s + (action,)] += 0.18 * (target - q[s + (action,)])
            x, y, yaw = nx, ny, nyaw
            if collided or reached:
                break
        if (ep + 1) % 1000 == 0:
            print(f'Learning: {ep+1}/{episodes} episodes', flush=True)
    np.save(path, q)
    print(f'Saved learned policy: {path}')


def run_ros(path):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    q = np.load(path)

    class Controller(Node):
        def __init__(self):
            super().__init__('fence_rl_controller')
            self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
            self.pose = None
            self.index = 0
            self.last_log = 0
            self.create_timer(0.1, self.tick)
            self.get_logger().info('Learned controller running. Safety boundary +/-2.35m.')

        def on_odom(self, msg):
            p = msg.pose.pose.position
            z = msg.pose.pose.orientation.z
            w = msg.pose.pose.orientation.w
            xq = msg.pose.pose.orientation.x
            yq = msg.pose.pose.orientation.y
            yaw = math.atan2(2 * (w*z + xq*yq), 1 - 2*(yq*yq + z*z))
            self.pose = (p.x, p.y, yaw)

        def tick(self):
            if self.pose is None:
                return
            x, y, yaw = self.pose
            goal = GOALS[self.index]
            if math.hypot(goal[0]-x, goal[1]-y) < 0.38:
                self.index = (self.index + 1) % len(GOALS)
                goal = GOALS[self.index]
                self.get_logger().info(f'Goal reached. Next goal: {goal}')
            s = state(x, y, yaw, goal)
            candidates = list(np.argsort(q[s])[::-1])
            chosen = None
            for a in candidates:
                # Predict 1.5 seconds of motion and reject unsafe candidates.
                px, py, pa = x, y, yaw
                ok = True
                for _ in range(15):
                    v, w = ACTIONS[int(a)]
                    pa = wrap(pa + w*0.1)
                    px += v*0.1*math.cos(pa)
                    py += v*0.1*math.sin(pa)
                    if abs(px) > SAFE or abs(py) > SAFE:
                        ok = False
                        break
                if ok:
                    chosen = int(a)
                    break
            # Additional heading fallback avoids persistent circling when learned policy is poor.
            heading_error = wrap(math.atan2(goal[1]-y, goal[0]-x)-yaw)
            if chosen is None or (abs(heading_error) > 1.2 and ACTIONS[chosen][0] > 0):
                chosen = 3 if heading_error >= 0 else 4
            v, w = ACTIONS[chosen]
            # Conservative emergency stop near any wall, permit turning in place.
            if v > 0 and (abs(x) > SAFE - 0.1 or abs(y) > SAFE - 0.1):
                v = 0.0
                w = 1.1 if heading_error >= 0 else -1.1
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(w)
            self.pub.publish(msg)

        def stop(self):
            self.pub.publish(Twist())

    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['train', 'run'])
    parser.add_argument('--policy', default=str(Path(__file__).with_name('fence_policy.npy')))
    parser.add_argument('--episodes', type=int, default=7000)
    args = parser.parse_args()
    if args.mode == 'train':
        train(args.policy, args.episodes)
    else:
        run_ros(args.policy)

if __name__ == '__main__':
    main()
