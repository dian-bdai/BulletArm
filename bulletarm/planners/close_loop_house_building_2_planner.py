import numpy as np

from bulletarm.pybullet.utils import constants
from bulletarm.planners.close_loop_planner import CloseLoopPlanner
from bulletarm.pybullet.utils import transformations
from bulletarm.planners.house_building_2_planner import HouseBuilding2Planner
class CloseLoopHouseBuilding2Planner(CloseLoopPlanner):
  def __init__(self, env, config):
    super().__init__(env, config)
    self.house_building_2_planner = HouseBuilding2Planner(env, config)
    self.pick_place_stage = 0
    self.task_stage = 0 # 0: nothing 1: two cube aligned

    self.current_target = None
    self.pre_goal = None
    self.goal = None
    self.post_goal = None

    # self.current_target = None
    # self.target_obj = None

  def getNextActionToCurrentTarget(self):
    x, y, z, r = self.getActionByGoalPose(self.current_target[0], self.current_target[1])
    if np.all(np.abs([x, y, z]) < self.dpos) and np.abs(r) < self.drot:
      primitive = self.current_target[2]
      self.current_target = None
    else:
      primitive = constants.PICK_PRIMATIVE if self.isHolding() else constants.PLACE_PRIMATIVE
    return self.env._encodeAction(primitive, x, y, z, r)

  def setNewTarget(self):
    if self.house_building_2_planner.checkFirstLayer():
      self.task_stage = 1
    else:
      self.task_stage = 0

    if self.env.current_episode_steps == 1:
      self.pick_place_stage = 0
      self.current_target = None
      self.pre_goal = None
      self.goal = None
      self.post_goal = None

    if self.pick_place_stage in [0, 1, 2]:
      if self.goal is None:
        p, x, y, z, r = self.house_building_2_planner.getNextAction()
        gripper_rz = transformations.euler_from_quaternion(self.env.robot._getEndEffectorRotation())[2]
        if self.task_stage == 1:
          while r - gripper_rz > np.pi/2:
            r -= np.pi
          while r - gripper_rz < -np.pi/2:
            r += np.pi
        else:
          while r - gripper_rz > np.pi/4:
            r -= np.pi/2
          while r - gripper_rz < -np.pi/4:
            r += np.pi/2
        self.pre_goal = [1, x, y, z + 0.1, r]
        self.goal = [0, x, y, z, r]
        self.post_goal = [0, x, y, z + 0.1, r]
      if self.pick_place_stage == 0:
        self.pick_place_stage = 1
        self.current_target = (self.pre_goal[1:4], self.pre_goal[4], self.pre_goal[0])
      elif self.pick_place_stage == 1:
        self.pick_place_stage = 2
        self.current_target = (self.goal[1:4], self.goal[4], self.goal[0])
      else:
        self.pick_place_stage = 3
        self.current_target = (self.post_goal[1:4], self.post_goal[4], self.post_goal[0])
        self.goal = None
        self.pre_goal = None
        self.post_goal = None

    else:
      if self.goal is None:
        p, x, y, z, r = self.house_building_2_planner.getNextAction()
        gripper_rz = transformations.euler_from_quaternion(self.env.robot._getEndEffectorRotation())[2]
        # if self.task_stage == 1:
        #   while r - gripper_rz > np.pi / 2:
        #     r -= np.pi
        #   while r - gripper_rz < -np.pi / 2:
        #     r += np.pi
        # else:
        #   while r - gripper_rz > np.pi / 4:
        #     r -= np.pi / 2
        #   while r - gripper_rz < -np.pi / 4:
        #     r += np.pi / 2
        while r - gripper_rz > np.pi / 2:
          r -= np.pi
        while r - gripper_rz < -np.pi / 2:
          r += np.pi
        self.pre_goal = [0, x, y, z + 0.1, r]
        self.goal = [1, x, y, z, r]
        self.post_goal = [1, x, y, z + 0.1, r]
      if self.pick_place_stage == 3:
        self.pick_place_stage = 4
        self.current_target = (self.pre_goal[1:4], self.pre_goal[4], self.pre_goal[0])
      elif self.pick_place_stage == 4:
        self.pick_place_stage = 5
        self.current_target = (self.goal[1:4], self.goal[4], self.goal[0])
      else:
        self.pick_place_stage = 0
        self.current_target = (self.post_goal[1:4], self.post_goal[4], self.post_goal[0])
        self.goal = None
        self.pre_goal = None
        self.post_goal = None

  def getNextAction(self):
    if self.env.current_episode_steps == 1:
      self.pick_place_stage = 0
      self.target_obj = None
      self.current_target = None
    if self.current_target is not None:
      return self.getNextActionToCurrentTarget()
    else:
      self.setNewTarget()
      return self.getNextActionToCurrentTarget()

  def getStepsLeft(self):
    return 100
