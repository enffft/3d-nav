import sys
import argparse
import numpy as np

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

from utils import *
from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config

parser = argparse.ArgumentParser()
parser.add_argument('--scene', type=str, default='Spiral',
                    help="Name of the scene. Available: ['Spiral', 'Building', 'Plaza', 'Second']")
args = parser.parse_args()

cfg = Config()

# 根据场景选择末端点
if args.scene == 'Spiral':
    tomo_file = 'spiral0.3_2'
    end_pos = np.array([-26.0, -5.0], dtype=np.float32)
elif args.scene == 'Building':
    tomo_file = 'MultifloorBuilding'
    end_pos = np.array([0.0, 0.0, 5.0], dtype=np.float32)
elif args.scene == "Second":
    tomo_file = 'second_building'
    # end_pos = np.array([5.5, 0.5, 3.0], dtype=np.float32)
    end_pos = np.array([6.0, 6.0, 3.0], dtype=np.float32)
else:
    tomo_file = 'plaza3_10'
    end_pos = np.array([23.0, 10.0], dtype=np.float32)

# 发布器
path_pub = rospy.Publisher("/pct_path", Path, latch=True, queue_size=1)
goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, latch=True, queue_size=1)

planner = TomogramPlanner(cfg)
got_start = False  # 是否已经获取起始点


def odom_callback(msg):
    global got_start

    if got_start:
        return

    # 获取当前位置作为 start_pos
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y + 7.5
    z = msg.pose.pose.position.z
    start_pos = np.array([x, y, 0.0], dtype=np.float32)
    rospy.loginfo(f"Got start position: {start_pos}")

    # 1. 先发一个当前点作为 /move_base_simple/goal → 触发 move_base
    goal_msg = PoseStamped()
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = "odom"
    goal_msg.pose = msg.pose.pose  # 直接用当前位姿
    goal_pub.publish(goal_msg)
    rospy.loginfo("Published dummy goal to /move_base_simple/goal")

    # 2. 调用 planner 计算路径
    planner.loadTomogram(tomo_file)
    traj_3d = planner.plan(start_pos, end_pos)
    if traj_3d is not None:
        path_pub.publish(traj2ros(traj_3d))
        rospy.loginfo("Published planned trajectory to /pct_path")

    got_start = True  # 避免重复调用


if __name__ == '__main__':
    rospy.init_node("pct_planner", anonymous=True)

    # 订阅机器人位置
    # rospy.Subscriber("/LIO/odom_vehicle", Odometry, odom_callback, queue_size=1)
    rospy.Subscriber("/Odometry", Odometry, odom_callback, queue_size=1)
    rospy.spin()
