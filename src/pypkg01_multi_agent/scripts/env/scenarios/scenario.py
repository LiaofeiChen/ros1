# defines scenario upon which the world is built
class BaseScenario():
    # create elements of the world
    def make_world(self):
        raise NotImplementedError()

    # create initial conditions of the world
    def reset_world(self, world):
        raise NotImplementedError()

    def step_world(self, world):
        raise NotImplementedError()