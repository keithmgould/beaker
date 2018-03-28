# these methods are mostly here to stub out OpenAI Gym methods that are not needed for a real robot.
# But by adding these, there is very little need to change code between a gym and the robot.

class ObservationSpace:
  def __init__(self, shape):
    self.shape = shape

class BaseWorld:
  def seed(self, something):
    return True

  def render(self, something="foo"):
    return True


