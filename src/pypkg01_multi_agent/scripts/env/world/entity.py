
# 所有实体的物理/外部基本状态
class EntityState():
    def __init__(self):
        # 物理坐标
        self.pose = None
        # 物理速度
        self.twist = None


# 实体：障碍物
class Obstacle(object):
    def __init__(self):
        # 实体的半径
        self.radius = None
        # 实体的颜色
        self.color = None
        # 实体的状态
        self.state = EntityState()


# 实体：地标
class Landmark(object):
    def __init__(self):
        # 实体的半径
        self.radius = None
        # 实体的颜色
        self.color = None
        # 实体的状态
        self.state = EntityState()


# 实体：目的地
class Destination(object):
    def __init__(self):
        # 实体的半径
        self.radius = None
        # 实体的颜色
        self.color = None
        # 实体的状态
        self.state = EntityState()