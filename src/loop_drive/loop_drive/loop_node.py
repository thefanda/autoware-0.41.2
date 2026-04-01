#!/usr/bin/env python3
import rclpy
import time
import math
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from autoware_adapi_v1_msgs.srv import SetRoutePoints
# 注意：删除了 ChangeOperationMode 的 import，因为不需要切换模式了
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

class InfiniteLoopRouteOnly(Node):
    def __init__(self):
        super().__init__('infinite_loop_route_only')
        
        # ================== 核心配置 ==================
        self.TRIGGER_DISTANCE = 50.0  
        # ============================================

        # 在此处添加任意数量的点
        # 只要保证顺序是 P0 -> P1 -> ... -> Pn -> P0
        self.points = [
            # P0
            Pose(position=Point(x=19661.976, y=71937.679, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.071, w=0.997)), 
            # P1
            Pose(position=Point(x=19822.283, y=71926.992, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.679, w=0.733)), 
            # P2 (请确认这是修正后的不压线坐标)
            Pose(position=Point(x=19634.824, y=71775.656, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.999, w=0.014)), 
            # P3
            Pose(position=Point(x=19546.144, y=71819.539, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.729, w=0.683)),
            
            # 你可以在这里继续添加 P4, P5, P6...
        ]

        # 【关键修改】自动获取点的总数量，不再写死为 4
        self.num_points = len(self.points)
        self.get_logger().info(f"已加载 {self.num_points} 个目标点")
        
        # 只保留路由相关的客户端
        self.client_set = self.create_client(SetRoutePoints, '/api/routing/set_route_points')
        self.client_change = self.create_client(SetRoutePoints, '/api/routing/change_route_points')
        
        self.sub_odom = self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10)
        
        self.current_pose = None
        
        # 【关键修改】初始目标设为最后一个点 (索引为 N-1)
        self.current_goal_idx = self.num_points - 1
        self.last_processed_goal_idx = -1 

        self.get_logger().info(f">>> 纯轨迹规划模式启动 (Trigger: {self.TRIGGER_DISTANCE}m)")
        threading.Thread(target=self.main_loop, daemon=True).start()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def calc_dist(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def get_next_route(self, current_goal_idx):
        # 【关键修改】使用 self.num_points 进行取模
        next_goal_idx = (current_goal_idx + 1) % self.num_points
        
        wps = []
        # 【关键修改】循环所有点：从 1 到 num_points + 1
        # 这意味着无论有多少个点，都会把整整一圈的路径发给规划器
        for i in range(1, self.num_points + 1): 
            idx = (next_goal_idx + i) % self.num_points
            wps.append(self.points[idx])
            
        target_goal = wps[-1]
        target_wps = wps[:-1]
        return target_wps, target_goal, next_goal_idx

    def send_route(self, wps, goal, is_initial=False):
        req = SetRoutePoints.Request()
        req.header.frame_id = "map"
        req.option.allow_goal_modification = True
        req.waypoints = wps
        req.goal = goal
        
        if is_initial:
            future = self.client_set.call_async(req)
        else:
            future = self.client_change.call_async(req)
            
        while not future.done(): time.sleep(0.01)
        return future.result()

    def main_loop(self):
        while not self.client_set.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待 Routing 服务...")
        time.sleep(1.0) 
        
        # ================= 初始路径 (动态生成) =================
        # 初始状态：去往最后一个点 (P_last)，途经前面的所有点 (P0...P_last-1)
        self.current_goal_idx = self.num_points - 1
        
        wps = []
        # 将除了最后一个点之外的所有点加入 waypoints
        for i in range(self.num_points - 1):
            wps.append(self.points[i])
            
        goal = self.points[self.num_points - 1]
        
        if self.send_route(wps, goal, is_initial=True).status.success:
            self.get_logger().info(f" 初始路径已发送 (Goal: P{self.current_goal_idx})")
            self.get_logger().info("请手动驾驶车辆，或者等待车辆移动...")
            self.last_processed_goal_idx = -1 
            time.sleep(1.0)
        
        # ================= 循环监控 =================
        while rclpy.ok():
            if self.current_pose is None:
                time.sleep(0.1); continue

            # 1. 获取关键点索引
            goal_idx = self.current_goal_idx
            # 【关键修改】计算上一个点，处理负数情况
            trigger_idx = (self.current_goal_idx - 1 + self.num_points) % self.num_points 
            
            # 2. 计算距离
            dist_trigger = self.calc_dist(self.current_pose.position, self.points[trigger_idx].position)
            dist_goal    = self.calc_dist(self.current_pose.position, self.points[goal_idx].position)
            
            # 3. 判断触发
            is_near_trigger = (dist_trigger < self.TRIGGER_DISTANCE)
            is_near_goal    = (dist_goal < self.TRIGGER_DISTANCE)
            
            if (is_near_trigger or is_near_goal) and (self.last_processed_goal_idx != self.current_goal_idx):
                
                trigger_source = "P{} (Pre)".format(trigger_idx) if is_near_trigger else "P{} (Final)".format(goal_idx)
                self.get_logger().warn(f" 触发切换! 触发源: {trigger_source}")
                
                next_wps, next_goal, next_goal_idx = self.get_next_route(self.current_goal_idx)
                
                res = self.send_route(next_wps, next_goal, is_initial=False)
                
                if res.status.success:
                    self.get_logger().info(f"成功延展路径 -> 新终点 P{next_goal_idx}")
                    self.last_processed_goal_idx = self.current_goal_idx
                    self.current_goal_idx = next_goal_idx
                    time.sleep(0.5) 
                else:
                    self.get_logger().error("切换失败")

            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = InfiniteLoopRouteOnly()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()