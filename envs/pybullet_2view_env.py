import pybullet as pb
import numpy as np
import copy
import scipy
import os
import helping_hands_rl_envs
import numpy.random as npr
import matplotlib.pyplot as plt

from helping_hands_rl_envs.simulators.pybullet.equipments.drawer import Drawer
from helping_hands_rl_envs.envs.pybullet_env import PyBulletEnv, NoValidPositionException
import helping_hands_rl_envs.simulators.pybullet.utils.object_generation as pb_obj_generation
from helping_hands_rl_envs.simulators import constants

class PyBullet2ViewEnv(PyBulletEnv):
  def __init__(self, config):
    super().__init__(config)
    self.wall_x = 1.

    self.view_matrix_2 = pb.computeViewMatrix(
      cameraEyePosition=[-10, self.workspace[1].mean(), self.workspace[2].mean()],
      cameraTargetPosition=[self.wall_x, self.workspace[1].mean(), self.workspace[2].mean()],
      cameraUpVector=[0, 0, 1])
    half_coverage = (self.workspace[2][1] - self.workspace[2][0]) / 2
    far = self.wall_x + 10
    fov = 2 * np.rad2deg(np.arctan(half_coverage / far))
    self.proj_matrix_2 = pb.computeProjectionMatrixFOV(
      fov=fov,
      aspect=1.0,
      nearVal=10,
      farVal=far + 0.01)
    self.wall_id = None

  def initialize(self):
    super().initialize()
    root_dir = os.path.dirname(helping_hands_rl_envs.__file__)
    urdf_filepath = os.path.join(root_dir, 'simulators/urdf/', 'wall.urdf')
    self.wall_id = pb.loadURDF(urdf_filepath,
                                     [self.wall_x,
                                      self.workspace[1].mean(),
                                      0],
                                     pb.getQuaternionFromEuler([0, 0, 0]),
                                     globalScaling=1)

  def _getHeightmap2(self):
    image_arr = pb.getCameraImage(width=self.heightmap_size, height=self.heightmap_size,
                                  viewMatrix=self.view_matrix_2, projectionMatrix=self.proj_matrix_2)
    depthImg = image_arr[3]
    far = self.wall_x + 10 + 0.01
    near = 10
    # far = 100
    # near = -(self.workspace[0][1] - self.workspace[0][0])
    depth = far * near / (far - (far - near) * depthImg)
    return depth.max() - depth

  def _getObservation(self, action=None):
    ''''''
    old_heightmap = self.heightmap
    self.heightmap = self._getHeightmap()

    if action is None or self._isHolding() == False:
      in_hand_img = self.getEmptyInHand()
    else:
      motion_primative, x, y, z, rot = self._decodeAction(action)
      in_hand_img = self.getInHandImage(old_heightmap, x, y, z, rot, self.heightmap)

    forward_heightmap = self._getHeightmap2()
    heightmaps = np.stack((self.heightmap, forward_heightmap), 0)
    heightmaps = np.moveaxis(heightmaps, 0, -1)

    return self._isHolding(), in_hand_img, heightmaps

