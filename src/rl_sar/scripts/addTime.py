#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np

class PointCloudTimeAdder:
    def __init__(self):
        rospy.init_node('pointcloud_time_adder')
        
        # 读取参数
        self.input_topic = rospy.get_param('~input_topic', '/velodyne_points')
        self.output_topic = rospy.get_param('~output_topic', '/velodyne_points_with_time')
        self.sensor_frame = rospy.get_param('~sensor_frame', 'velodyne')
        self.scan_rate = rospy.get_param('~scan_rate', 10.0)  # 扫描频率 (Hz)
        self.num_lasers = rospy.get_param('~num_lasers', 16)  # 激光线数
        
        # 订阅原始点云话题
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.callback, queue_size=1)
        
        # 发布添加了time字段的点云
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=10)
        
        rospy.loginfo(f"点云时间戳添加节点已启动 - 输入:{self.input_topic} 输出:{self.output_topic}")

    def callback(self, msg):
        start_time = rospy.Time.now()
        
        # 转换原始点云为可迭代对象
        try:
            points = list(pcl2.read_points(
                msg, 
                field_names=("x", "y", "z", "intensity", "ring"),
                skip_nans=False
            ))
            if not points:
                rospy.logwarn("接收到空点云消息")
                return
                
        except Exception as e:
            rospy.logerr(f"解析点云消息失败: {e}")
            return
        
        # 创建新的字段列表 (正确偏移量)
        new_fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
            PointField('ring', 16, PointField.UINT16, 1),
            PointField('time', 20, PointField.FLOAT32, 1)  # 正确偏移
        ]
        
        # 计算时间戳 (高效方法)
        new_points = []
        scan_period = 1.0 / self.scan_rate
        
        # 按ring分组计数
        ring_counts = [0] * self.num_lasers
        for point in points:
            _, _, _, _, ring = point
            if 0 <= ring < self.num_lasers:
                ring_counts[ring] += 1
        
        # 第二遍: 计算时间戳
        ring_indices = [0] * self.num_lasers
        for point in points:
            x, y, z, intensity, ring = point
            if 0 <= ring < self.num_lasers:
                count = ring_counts[ring]
                if count > 0:
                    # 当前点在ring中的位置比例
                    ratio = ring_indices[ring] / float(count)
                    time_offset = ratio * scan_period
                    ring_indices[ring] += 1
                else:
                    time_offset = 0.0
                new_points.append([x, y, z, intensity, ring, time_offset])
            else:
                # 无效ring值，默认时间0
                new_points.append([x, y, z, intensity, ring, 0.0])
        
        # 创建新的点云消息
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id
        
        new_cloud_msg = pcl2.create_cloud(header, new_fields, new_points)
        new_cloud_msg.is_dense = msg.is_dense
        new_cloud_msg.width = len(new_points)
        new_cloud_msg.height = 1  # 无序点云
        new_cloud_msg.point_step = 24  # 正确步长: 20+4
        new_cloud_msg.row_step = 24 * len(new_points)
        
        # 发布新点云
        self.pub.publish(new_cloud_msg)
        
        # 记录处理时间
        processing_time = (rospy.Time.now() - start_time).to_sec()
        rospy.logdebug(f"处理点云: {len(points)}点, 耗时: {processing_time:.4f}秒")

if __name__ == '__main__':
    try:
        node = PointCloudTimeAdder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal(f"节点异常退出: {e}")
        import traceback
        traceback.print_exc()