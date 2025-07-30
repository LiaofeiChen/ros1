from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from .controller import Controller

class AgentState:
    def __init__(self):
        # 位姿：位置 + 姿态（带时间戳）
        self.pose = PoseStamped()

        # 速度：线速度 + 角速度（带时间戳）
        self.twist = TwistStamped()

        # 加速度：线加速度 + 角加速度
        self.accel = AccelStamped()

        # 是否可以移动
        self.movable = None
        # 默认值为 False，当撞到其他可碰撞对象时设置为 True。
        self.crash_bound = None
        self.crash_agent = None

# 智能体
class Agent:
    def __init__(self):
        self.agent_id: str = ""
        # 智能体的安全半径
        self.r_safe = None
        self.twist_lim = None
        self.accel_lim = None
        # 智能体的状态
        self.state = AgentState()
        self.action = Action()
        self.controller = None
    
    def set_contoller(self):
        self.controller = Controller(self)


# 智能体的动作
class Action():
    def __init__(self):
        # 物理动作
        self.u = None