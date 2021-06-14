import numpy as np
import copy


class SpideyWalkingCommandGenerator:
    def __init__(self, desired_EE_poses, desired_root_pose):
        self._desired_EE_poses = desired_EE_poses
        self._desired_root_pose = desired_root_pose
        # self._desired_cmd = {"desired_feet_poses": self._desired_EE_poses,
        #                      "desired_base_link_pose": self._desired_root_pose}
        self._delta_x = 0.0759167
        
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
        intermediate_EE_cmd = np.concatenate((intermediate_EE_cmd_xy, np.array([0.02, 0, 0, 0])))
        self._desired_EE_poses[leg_num-1] = intermediate_EE_cmd
        return self.desired_cmd
    
    def _generate_predrop_leg_cmd(self, leg_num, desired):
        predrop_EE_cmd = np.copy(desired)
        predrop_EE_cmd[2] = 0.03
        self._desired_EE_poses[leg_num-1] = predrop_EE_cmd
        return self.desired_cmd
    

    '''
    Operation
    '''



    def default_state(self):
        self._desired_EE_poses = [np.array([0.10715663904943551, -0.12059091774119927, -0.0018311943969772787, 0, 0, 0]),
                                  np.array([0.12059166919289249, 0.1071557395480627, -0.0018311943969772787, 0, 0, 0]),
                                  np.array([-0.12059014557973162, -0.1071553165719085, -0.0018322660567569038, 0, 0, 0]),
                                  np.array([-0.10715501674187393, 0.12059039605987298, -0.0018322660567569038, 0, 0, 0])]
        self._desired_root_pose = np.array([0, 0, -0.065, 0, 0, 0])
        return [self.desired_cmd]

    def move_3_to_IN_init(self):
        previous = self._desired_EE_poses[2]
        desired = np.copy(previous)
        desired[0] += self._delta_x
        self._desired_EE_poses[2] = desired
        final_cmd = self.desired_cmd
        return [final_cmd]


    def move_1_to_IN_init(self):
        previous = self._desired_EE_poses[0]
        desired = np.copy(previous)
        desired[0] += -self._delta_x
        intermediate_cmd = self._generate_intermediate_leg_cmd(1, previous, desired)
        #predrop_cmd = self._generate_predrop_leg_cmd(1, desired)
        self._desired_EE_poses[0] = desired
        final_cmd = self.desired_cmd
        return [intermediate_cmd, final_cmd]
    


    def move_1_to_OUT(self):
        previous = self._desired_EE_poses[0]
        desired = np.copy(previous)
        desired[0] += 2*self._delta_x
        intermediate_cmd = self._generate_intermediate_leg_cmd(1, previous, desired)
        #predrop_cmd = self._generate_predrop_leg_cmd(1, desired)
        self._desired_EE_poses[0] = desired
        final_cmd = self.desired_cmd
        return [intermediate_cmd, final_cmd]
            
    def move_body_forward(self):
        self._desired_root_pose[0] += self._delta_x
        return [self.desired_cmd]

    def move_4_to_IN(self):
        previous = self._desired_EE_poses[3]
        desired = np.copy(previous)
        desired[0] += 2*self._delta_x
        intermediate_cmd = self._generate_intermediate_leg_cmd(4, previous, desired)
        #predrop_cmd = self._generate_predrop_leg_cmd(4, desired)
        self._desired_EE_poses[3] = desired
        final_cmd = self.desired_cmd
        return [intermediate_cmd, final_cmd]
    
    def move_2_to_OUT(self):
        previous = self._desired_EE_poses[1]
        desired = np.copy(previous)
        desired[0] += 2*self._delta_x
        intermediate_cmd = self._generate_intermediate_leg_cmd(2, previous, desired)
        #predrop_cmd = self._generate_predrop_leg_cmd(2, desired)
        self._desired_EE_poses[1] = desired
        final_cmd = self.desired_cmd
        return [intermediate_cmd, final_cmd]

    def move_3_to_IN(self):
        previous = self._desired_EE_poses[2]
        desired = np.copy(previous)
        desired[0] += 2*self._delta_x
        intermediate_cmd = self._generate_intermediate_leg_cmd(3, previous, desired)
       # predrop_cmd = self._generate_predrop_leg_cmd(3, desired)
        self._desired_EE_poses[2] = desired
        final_cmd = self.desired_cmd
        return [intermediate_cmd, final_cmd]
