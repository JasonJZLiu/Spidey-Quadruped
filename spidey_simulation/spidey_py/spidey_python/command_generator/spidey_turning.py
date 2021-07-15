import numpy as np
import copy
from scipy.spatial.transform import Rotation as R


class SpideyTurningCommandGenerator:
    def __init__(self, desired_EE_poses, desired_root_pose):
        self._desired_EE_poses = desired_EE_poses
        self._desired_root_pose = desired_root_pose

        self.delta_angle = 0.3
        
    @property
    def desired_cmd(self):
        return {"desired_feet_poses": copy.deepcopy(self._desired_EE_poses), "desired_base_link_pose": copy.deepcopy(self._desired_root_pose)}
        #return self._desired_cmd
    
    '''
    Internal
    '''
    def _generate_intermediate_leg_cmd(self, leg_num, previous, desired):
        diff_vec_xy = desired[0:2] - previous[0:2]
        intermediate_EE_cmd_xy = previous[0:2] + diff_vec_xy/2
        intermediate_EE_cmd = np.concatenate((intermediate_EE_cmd_xy, np.array([0.06, 0, 0, 0])))
        self._desired_EE_poses[leg_num-1] = intermediate_EE_cmd
        return self.desired_cmd
    
    def _generate_predrop_leg_cmd(self, leg_num, desired):
        predrop_EE_cmd = np.copy(desired)
        predrop_EE_cmd[2] = 0.04
        self._desired_EE_poses[leg_num-1] = predrop_EE_cmd
        return self.desired_cmd

    def _generate_prelift_leg_cmd(self, leg_num, previous):
        prelift_EE_cmd = np.copy(previous)
        prelift_EE_cmd[2] = 0.03#0.0075
        temp = np.copy(self._desired_EE_poses)
        temp[leg_num-1] = prelift_EE_cmd
        return {"desired_feet_poses": temp, "desired_base_link_pose": copy.deepcopy(self._desired_root_pose)}
    
    def _rotate_leg(self, leg_num, previous, delta_angle):
        rotation_matrix = R.from_euler('z', delta_angle).as_matrix()
        return np.concatenate((rotation_matrix.dot(previous[:3]), [0,0,0]))



    '''
    Operation
    '''
    def default_right_ready_state(self):
        self._desired_EE_poses = [np.array([0.0470257064755463, -0.14011758200108318, 0.0008365468198590076, 0, 0, 0]),
                                  np.array([0.1403228518202999, 0.12744491649211073, -0.0006170778066039162, 0, 0, 0]),
                                  np.array([-0.060152276136948436, -0.12626821953289263, 0.0010018703715623106, 0, 0, 0]),
                                  np.array([-0.126564367303947, 0.14027069634788308, -0.0008578122876786165, 0, 0, 0])]
        self._desired_root_pose = np.array([0, 0, -0.035, 0, 0, 0]) #-0.065

        return [self.desired_cmd]


    def turn_CCW_from_right_move_1(self):
        if self.delta_angle >= 0:
            leg_num = 1
        else:
            leg_num = 3
        previous = self._desired_EE_poses[leg_num-1]
        desired = self._rotate_leg(leg_num, previous, self.delta_angle)
        prelift_cmd = self._generate_prelift_leg_cmd(leg_num, previous)
        intermediate_cmd = self._generate_intermediate_leg_cmd(leg_num, previous, desired)
        predrop_cmd = self._generate_predrop_leg_cmd(leg_num, desired)
        self._desired_EE_poses[leg_num-1] = desired
        final_cmd = self.desired_cmd
        return [prelift_cmd, intermediate_cmd, predrop_cmd, final_cmd]

    def turn_CCW_from_right_move_2(self):
        if self.delta_angle >= 0:
            leg_num = 2
        else:
            leg_num = 4
        previous = self._desired_EE_poses[leg_num-1]
        desired = self._rotate_leg(leg_num, previous, self.delta_angle)
        prelift_cmd = self._generate_prelift_leg_cmd(leg_num, previous)
        intermediate_cmd = self._generate_intermediate_leg_cmd(leg_num, previous, desired)
        predrop_cmd = self._generate_predrop_leg_cmd(leg_num, desired)
        self._desired_EE_poses[leg_num-1] = desired
        final_cmd = self.desired_cmd
        return [prelift_cmd, intermediate_cmd, predrop_cmd, final_cmd]

    def turn_CCW_from_right_move_3(self):
        self._desired_root_pose[5] += self.delta_angle
        return [self.desired_cmd]

    def turn_CCW_from_right_move_4(self):
        if self.delta_angle >= 0:
            leg_num = 4
        else:
            leg_num = 2
        previous = self._desired_EE_poses[leg_num-1]
        desired = self._rotate_leg(leg_num, previous, self.delta_angle)
        prelift_cmd = self._generate_prelift_leg_cmd(leg_num, previous)
        intermediate_cmd = self._generate_intermediate_leg_cmd(leg_num, previous, desired)
        predrop_cmd = self._generate_predrop_leg_cmd(leg_num, desired)
        self._desired_EE_poses[leg_num-1] = desired
        final_cmd = self.desired_cmd
        return [prelift_cmd, intermediate_cmd, predrop_cmd, final_cmd]

    def turn_CCW_from_right_move_5(self):
        if self.delta_angle >= 0:
            leg_num = 3
        else:
            leg_num = 1
        previous = self._desired_EE_poses[leg_num-1]
        desired = self._rotate_leg(leg_num, previous, self.delta_angle)
        prelift_cmd = self._generate_prelift_leg_cmd(leg_num, previous)
        intermediate_cmd = self._generate_intermediate_leg_cmd(leg_num, previous, desired)
        predrop_cmd = self._generate_predrop_leg_cmd(leg_num, desired)
        self._desired_EE_poses[leg_num-1] = desired
        final_cmd = self.desired_cmd
        return [prelift_cmd, intermediate_cmd, predrop_cmd, final_cmd]
