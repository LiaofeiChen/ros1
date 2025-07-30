from ...env.world.world import World
from ...env.agent.agent import Agent
from .scenario import BaseScenario
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospy


AGENTS = {'center': [[-3.3, -1.4, 1], [-3.3, -0.6, 1], [-2.5, -1.4, 1]],
          'color': [[[1.0, 0.0, 0.0], 1.0], [[0.0, 1.0, 0.0], 1.0], [[0.0, 0.0, 1.0], 1.0]]}


class Scenario(BaseScenario):
    def __init__(self, args=None, mode=None):
        self.total_time = None
        self.total_step = None
        self.dt = None
        self.args = args
        self.mode = mode
        # 发布连接线（所有无人机共享同一个发布器）
        self.line_pub = rospy.Publisher(
            "/formation_lines",
            Marker, 
            10
        )
        
        self.world = self.make_world()
        self.reset_world()
        self.timer = rospy.Timer(rospy.Duration(0.01), self.publish_formation_lines_callback)

    def make_world(self):
        # 创建一个World实例
        world = World()
        world.dt = self.args.dt  # 设置时间步长
        world.field_range = [-4, 4, -2.5, 2.5, 0, 2]  # 设置主场范围
        # 智能体的数量
        agent_number = 3

        # 添加智能体
        world.agent_list = []
        for idx in range(agent_number):
            agent = Agent()
            agent.agent_id = 'quad010' + str(idx+1)
            agent.set_contoller()
            agent.r_safe = 0.1
            agent.twist_lim = [1, 1, 1, 0.1, 0.1, 0.1]
            agent.accel_lim = [0.5, 0.5, 0.5, 0.1, 0.1, 0.1]
            world.agent_list.append(agent)

        return world

    def reset_world(self):
        self.total_time = 0.0
        self.total_step = 0

        for agent_i, agent in enumerate(self.world.agent_list):
            agent.state.movable = True
            agent.state.crash_bound = False
            agent.state.crash_agent = False
            agent.state.pose.pose.position.x = AGENTS['center'][agent_i][0]
            agent.state.pose.pose.position.y = AGENTS['center'][agent_i][1]
            agent.state.pose.pose.position.z = AGENTS['center'][agent_i][2]
            agent.action.u = np.array([0.0, 0.0, 0.0])

    def step_world(self):
        self.total_step += 1
        self.total_time += self.world.dt

        u = self.world.update_traditional_formation_control_input()
        # 对每个智能体设置动作
        for i, agent in enumerate(self.world.agent_list):
            agent.action.u = np.clip(u[i], -np.array(agent.accel_lim[:3]), np.array(agent.accel_lim[:3]))
            rospy.loginfo(f"{agent.state.movable}")
        # 更新世界状态
        self.world.update_one_sim_step()

        for i, agent in enumerate(self.world.agent_list):
            agent.controller.publish_pose()
        
        self.publish_formation_lines(self.world.agent_list)
        
        # 检测碰撞状态
        self.world.check_collision()

    def done(self):
        dones = []
        for i, agent in enumerate(self.world.agent_list):
            dones.append(not agent.state.movable)

        if any(dones):
            return True
        else:
            return False
        
    def publish_formation_lines(self, agents):
        """发布编队连接线"""
        if len(agents) < 2:
            return

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "formation"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # 细线宽（0.01~0.05）
        marker.scale.y = 1e-6  
        marker.scale.z = 1e-6 
        marker.color.a = 1.0   # 透明度
        marker.color.r = 0.0   # 红色
        marker.color.g = 1.0   # 绿色
        marker.color.b = 0.0   # 蓝色

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # 添加所有无人机的位置点
        for agent in agents:
            p = Point()
            p.x = agent.state.pose.pose.position.x
            p.y = agent.state.pose.pose.position.y
            p.z = agent.state.pose.pose.position.z
            marker.points.append(p)

        # 闭合图形（连接首尾）
        if len(agents) >= 3:
            first_agent = agents[0]
            p = Point()
            p.x = first_agent.state.pose.pose.position.x
            p.y = first_agent.state.pose.pose.position.y
            p.z = first_agent.state.pose.pose.position.z
            marker.points.append(p)

        self.line_pub.publish(marker)

    def publish_formation_lines_callback(self, event):
        self.publish_formation_lines(self.world.agent_list) 
        rospy.loginfo(f"type(line_pub): {type(self.line_pub)}")  




