import numpy as np
from scipy.integrate import odeint


# 多智能体世界
class World:
    """
    # multi-agent world
    """
    def __init__(self):
        # 一个步骤的实际持续时间
        self.dt = None

        # 智能体列表
        self.agent_list = []

        # 主场范围
        self.field_range = [-4, 4, -2.5, 2.5, -0.1, 2]  # [xmin, xmax, ymin, ymax, zmin, zmax]


    # 主场中心坐标
    @property
    def field_center(self):
        center_x = (self.field_range[0] + self.field_range[1]) / 2
        center_y = (self.field_range[2] + self.field_range[3]) / 2
        center_z = 0  # 主场中心的Z坐标通常为0
        return center_x, center_y, center_z

    # 主场中心的长度、宽度、高度
    @property
    def field_half_size(self):
        length = (self.field_range[1] - self.field_range[0]) / 2
        width = (self.field_range[3] - self.field_range[2]) / 2
        height = (self.field_range[5] - self.field_range[4]) / 2
        return length, width, height
    
    # 对每个智能体检查碰撞状态
    def check_collision(self):
        for idx_a, agent_a in enumerate(self.agent_list):
            if agent_a.state.crash_bound or agent_a.state.crash_agent:
                agent_a.state.movable = False
                continue

            # 检查 agent_a 是否撞上了 agent_b
            for idx_b, agent_b in enumerate(self.agent_list):
                if idx_a == idx_b:
                    continue
                pos_a = agent_a.state.pose.pose.position
                pos_b = agent_b.state.pose.pose.position
                # 计算两智能体之间的距离
                # 计算欧氏距离
                dist = np.sqrt(
                    (pos_a.x - pos_b.x) ** 2 +
                    (pos_a.y - pos_b.y) ** 2 +
                    (pos_a.z - pos_b.z) ** 2
                )
                if dist <= agent_a.r_safe + agent_b.r_safe:
                    agent_a.state.crash_agent = True
                    agent_a.state.movable = False
                    break

            # 检查 agent_a 是否撞上了 boundary
            pos_a = agent_a.state.pose.pose.position
            if pos_a.x-agent_a.r_safe <= self.field_range[0] or \
                    pos_a.x+agent_a.r_safe >= self.field_range[1] or \
                    pos_a.y-agent_a.r_safe <= self.field_range[2] or \
                    pos_a.y+agent_a.r_safe >= self.field_range[3] or \
                    pos_a.z-agent_a.r_safe <= self.field_range[4] or \
                    pos_a.z+agent_a.r_safe >= self.field_range[5]:
                agent_a.state.crash_bound = True
                agent_a.state.movable = False
    
      
    # 限制智能体速度和加速度
    def check_dynamics_constraints(self):
        for agent in self.agent_list:
            vel_lim = agent.twist_lim
            accel_lim = agent.accel_lim
            # 提取速度和加速度
            linear_vel = agent.state.twist.twist.linear
            angular_vel = agent.state.twist.twist.angular
            linear_acc = agent.state.accel.accel.linear
            angular_acc = agent.state.accel.accel.angular

            vx = linear_vel.x
            vy = linear_vel.y
            vz = linear_vel.z
            wx = angular_vel.x
            wy = angular_vel.y
            wz = angular_vel.z
            ax = linear_acc.x
            ay = linear_acc.y
            az = linear_acc.z
            aax = angular_acc.x
            aay = angular_acc.y
            aaz = angular_acc.z
            
            vel = np.array([vx, vy, vz, wx, wy, wz])
            accel_vals = np.array([ax, ay, az, aax, aay, aaz])

            # clip 裁剪到 [-limit, limit]
            vel_clamped = np.clip(vel, -np.array(vel_lim), np.array(vel_lim))
            accel_clamped = np.clip(accel_vals, -np.array(accel_lim), np.array(accel_lim))

            # 写回裁剪后的值（速度）
            linear_vel.x, linear_vel.y, linear_vel.z = vel_clamped[:3]
            angular_vel.x, angular_vel.y, angular_vel.z = vel_clamped[3:]

            # 写回裁剪后的值（加速度）
            linear_acc.x, linear_acc.y, linear_acc.z = accel_clamped[:3]
            angular_acc.x, angular_acc.y, angular_acc.z = accel_clamped[3:]


    def update_one_sim_step(self):
        # 更新位置状态
        t = [0, self.dt]  # 积分时间区间
        for agent in self.agent_list:
            # if the agent is not movable, skip updating its physical state
            if agent.state.movable:
                initial_state = [
                    agent.state.pose.pose.position.x,
                    agent.state.pose.pose.position.y,
                    agent.state.pose.pose.position.z,
                    agent.state.twist.twist.linear.x,
                    agent.state.twist.twist.linear.y,
                    agent.state.twist.twist.linear.z,
                ]

                next_state = odeint(f, initial_state, t, args=(agent.action.u,))

                # 更新位置和速度
                agent.state.pose.pose.position.x = next_state[-1][0]
                agent.state.pose.pose.position.y = next_state[-1][1]
                agent.state.pose.pose.position.z = next_state[-1][2]
                agent.state.twist.twist.linear.x = next_state[-1][3]
                agent.state.twist.twist.linear.y = next_state[-1][4]
                agent.state.twist.twist.linear.z = next_state[-1][5]

                self.check_dynamics_constraints()

    def update_traditional_formation_control_input(self):
        p1 = np.array([
            self.agent_list[0].state.pose.pose.position.x, 
            self.agent_list[0].state.pose.pose.position.y, 
            self.agent_list[0].state.pose.pose.position.z, 
        ])
        p2 = np.array([
            self.agent_list[1].state.pose.pose.position.x, 
            self.agent_list[1].state.pose.pose.position.y, 
            self.agent_list[1].state.pose.pose.position.z, 
        ])
        p3 = np.array([
            self.agent_list[2].state.pose.pose.position.x, 
            self.agent_list[2].state.pose.pose.position.y, 
            self.agent_list[2].state.pose.pose.position.z, 
        ])

        v1 = np.array([
            self.agent_list[0].state.twist.twist.linear.x,
            self.agent_list[0].state.twist.twist.linear.y,
            self.agent_list[0].state.twist.twist.linear.z,
        ])
        v2 = np.array([
            self.agent_list[1].state.twist.twist.linear.x,
            self.agent_list[1].state.twist.twist.linear.y,
            self.agent_list[1].state.twist.twist.linear.z,
        ])
        v3 = np.array([
            self.agent_list[2].state.twist.twist.linear.x,
            self.agent_list[2].state.twist.twist.linear.y,
            self.agent_list[2].state.twist.twist.linear.z,
        ])

        # 两点水平距离d_ij
        d_12 = np.linalg.norm(p1[:2] - p2[:2])
        d_13 = np.linalg.norm(p1[:2] - p3[:2])
        d_21 = np.linalg.norm(p2[:2] - p1[:2])
        d_23 = np.linalg.norm(p2[:2] - p3[:2])
        d_31 = np.linalg.norm(p3[:2] - p1[:2])
        d_32 = np.linalg.norm(p3[:2] - p2[:2])

        # 单位向量z_ij
        z_12 = (p2[:2] - p1[:2]) / d_21
        z_13 = (p3[:2] - p1[:2]) / d_31
        z_21 = (p1[:2] - p2[:2]) / d_12
        z_23 = (p3[:2] - p2[:2]) / d_32
        z_31 = (p1[:2] - p3[:2]) / d_13
        z_32 = (p2[:2] - p3[:2]) / d_23

        dot_product = np.clip(np.dot(z_12, z_13), -1.0, 1.0)  # 防止浮点数误差导致超出 acos 定义域
        alpha_213 = np.arccos(dot_product)

        dot_product = np.clip(np.dot(z_21, z_23), -1.0, 1.0)  # 防止浮点数误差导致超出 acos 定义域
        alpha_123 = np.arccos(dot_product)

        dot_product = np.clip(np.dot(z_31, z_32), -1.0, 1.0)  # 防止浮点数误差导致超出 acos 定义域
        alpha_132 = np.arccos(dot_product)

        k1 = 0.2
        k2 = 0.1
        k3 = 0.1
        desired_alpha_213 = np.pi / 3
        desired_alpha_123 = np.pi / 3
        desired_alpha_132 = np.pi / 3
        desired_d_13 = 1
        desired_vc = np.array([0.5, 0.0])

        u1 = np.zeros(3)
        u2 = np.zeros(3)
        u3 = np.zeros(3)

        u1[:2] = - k1 * (v1[:2] - desired_vc) - k2 * (alpha_213 - desired_alpha_213) * (z_12 + z_13)
        u2[:2] = - k1 * (v2[:2] - desired_vc) - k2 * (alpha_123 - desired_alpha_123) * (z_21 + z_23)
        u3[:2] = - k1 * (v3[:2] - desired_vc) - k2 * (alpha_132 - desired_alpha_132) * (z_31 + z_32) + k3 * (d_13 - desired_d_13) * (z_31 + z_32)

        u = [u1, u2, u3]
        return u


# ode积分
def f(state, t, u):
    # 控制输入
    ux = u[0]
    uy = u[1]
    uz = u[2]
    x, y, z, vx, vy, vz = state
    return [vx, vy, vz, ux, uy, uz]


