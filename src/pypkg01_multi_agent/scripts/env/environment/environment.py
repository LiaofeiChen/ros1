

class MultiAgentEnv():
    def __init__(self, args, scenario):
        self.args = args
        self.scenario = scenario
        self.total_step = 0
        self.total_time = 0.0

    def reset(self):
        self.total_step = 0
        self.total_time = 0.0
        self.scenario.reset_world()
    
    def step(self):
        self.total_step += 1
        self.total_time += self.scenario.world.dt

        # 更新世界状态
        if not self.scenario.done():
            self.scenario.step_world()
