import pybullet as pb
import numpy as np

from bulletarm.pybullet.utils import constants
from bulletarm.envs.close_loop_envs.close_loop_env import CloseLoopEnv
from bulletarm.pybullet.utils import transformations
from bulletarm.planners.close_loop_house_building_1_planner import CloseLoopHouseBuilding1Planner
from bulletarm.pybullet.utils.constants import NoValidPositionException

class CloseLoopHouseBuilding2Env(CloseLoopEnv):
  '''Close loop house building 2 task.

  The robot needs to first place two cubic blocks adjacent to each other, then place a roof on top.

  Args:
    config (dict): Intialization arguments for the env
  '''
  def __init__(self, config):
    if 'object_scale_range' not in config:
      config['object_scale_range'] = [0.8, 0.8]
    config['num_objects'] = 3
    super().__init__(config)

  def reset(self):
    while True:
      self.resetPybulletWorkspace()
      self.robot.moveTo([self.workspace[0].mean(), self.workspace[1].mean(), 0.2], transformations.quaternion_from_euler(0, 0, 0))
      try:
        self._generateShapes(constants.ROOF, 1, random_orientation=self.random_orientation)
        self._generateShapes(constants.CUBE, 2, random_orientation=self.random_orientation)
      except NoValidPositionException as e:
        continue
      else:
        break
    return self._getObservation()

  def _getValidOrientation(self, random_orientation):
    if random_orientation:
      orientation = pb.getQuaternionFromEuler([0., 0., np.pi * (np.random.random_sample() - 0.5)])
    else:
      orientation = pb.getQuaternionFromEuler([0., 0., 0.])
    return orientation

  def _checkTermination(self):
    blocks = list(filter(lambda x: self.object_types[x] == constants.CUBE, self.objects))
    roofs = list(filter(lambda x: self.object_types[x] == constants.ROOF, self.objects))

    top_blocks = []
    for block in blocks:
      if self._isObjOnTop(block, blocks):
        top_blocks.append(block)
    if len(top_blocks) != 2:
      return False
    return not self._isHolding() and self._checkOnTop(top_blocks[0], roofs[0]) and \
      self._checkOnTop(top_blocks[1], roofs[0]) and self._checkObjUpright(roofs[0])

def createCloseLoopHouseBuilding2Env(config):
  return CloseLoopHouseBuilding2Env(config)
