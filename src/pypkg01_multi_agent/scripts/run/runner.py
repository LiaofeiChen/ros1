import rospy

def make_env(args, node):
    from pypkg01_multi_agent.scripts.env.environment.environment import MultiAgentEnv
    from pypkg01_multi_agent.scripts.env.scenarios.sample1 import Scenario

    scenario = Scenario(args, node)
    
    env = MultiAgentEnv(args, scenario)

    return env

class Runner:
    def __init__(self):
        self.args = get_args()
        rospy.init_node("test01")
        self.env = make_env(self.args, self)
        self.sim_timer = rospy.Timer(rospy.Duration(self.args.dt), self.step_cb)

    def step_cb(self, event):
        self.env.step()
        rospy.loginfo(f"仿真时间：{self.env.scenario.total_time}")


    def run(self):
        rospy.spin()

def get_args():
    import argparse
    parser = argparse.ArgumentParser(description="")
    parser.add_argument('--dt', type=float, default="0.1")

    # 在ros中使用argparse, 必须要加
    args, _ = parser.parse_known_args()
    return args
        