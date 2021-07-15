import numpy as np
import copy
{'leg1_link3_tip': [0.12672210184071633, -0.1401562475777565, 8.00769041658933e-07, 1.5707999999999998, 1.4692820413320215e-05, 2.35619999994603], 
'leg2_link3_tip': [0.14015713198547564, 0.1267210693833176, 8.00769041658933e-07, 1.570799999999999, 1.4692820413320217e-05, -2.3562000000539696], 
'leg3_link3_tip': [-0.14015654238159364, -0.12672175768933297, 8.00769041658933e-07, 1.5707999999999993, 1.4692820413320215e-05, 0.7853999999460304], 
'leg4_link3_tip': [-0.1267214135368667, 0.14015683718416674, 8.00769041658933e-07, 1.5707999999999998, 1.4692820413320215e-05, -0.7854000000539698], 'base_link': [0.0, 0.0, 0.0, 0.0, -0.0, 0.0]}


class SpideyWalkingCommandGenerator:
    def __init__(self, desired_EE_poses, desired_root_pose):
        self._desired_EE_poses = desired_EE_poses
        self._desired_root_pose = desired_root_pose
        # self._desired_cmd = {"desired_feet_poses": self._desired_EE_poses,
        #                      "desired_base_link_pose": self._desired_root_pose}
        self._delta_x = 0.08#0.0889595
        
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
    

    '''
    Operation
    '''
    def default_state(self):
        self._desired_EE_poses = [np.array([0.12672210184071633, -0.1401562475777565, 8.00769041658933e-07, 0, 0, 0]),
                                  np.array([0.14015713198547564, 0.1267210693833176, 8.00769041658933e-07, 0, 0, 0]),
                                  np.array([-0.14015654238159364, -0.12672175768933297, 8.00769041658933e-07, 0, 0, 0]),
                                  np.array([-0.1267214135368667, 0.14015683718416674, 8.00769041658933e-07, 0, 0, 0])]
        self._desired_root_pose = np.array([0, 0, -0.035, 0, 0, 0]) #-0.065
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

        prelift_cmd = self._generate_prelift_leg_cmd(1, previous)

        intermediate_cmd = self._generate_intermediate_leg_cmd(1, previous, desired)
        predrop_cmd = self._generate_predrop_leg_cmd(1, desired)
        self._desired_EE_poses[0] = desired
        final_cmd = self.desired_cmd
        return [prelift_cmd, intermediate_cmd, predrop_cmd, final_cmd]
    


    def move_1_to_OUT(self):
        previous = self._desired_EE_poses[0]
        desired = np.copy(previous)
        desired[0] += 2*self._delta_x
        prelift_cmd = self._generate_prelift_leg_cmd(1, previous)

        intermediate_cmd = self._generate_intermediate_leg_cmd(1, previous, desired)
        predrop_cmd = self._generate_predrop_leg_cmd(1, desired)
        self._desired_EE_poses[0] = desired
        final_cmd = self.desired_cmd
        return [prelift_cmd, intermediate_cmd, predrop_cmd, final_cmd]
            
    def move_body_forward(self):
        self._desired_root_pose[0] += self._delta_x
        return [self.desired_cmd]

    def move_4_to_IN(self):
        previous = self._desired_EE_poses[3]
        desired = np.copy(previous)
        desired[0] += 2*self._delta_x
        prelift_cmd = self._generate_prelift_leg_cmd(4, previous)
        intermediate_cmd = self._generate_intermediate_leg_cmd(4, previous, desired)
        predrop_cmd = self._generate_predrop_leg_cmd(4, desired)
        self._desired_EE_poses[3] = desired
        final_cmd = self.desired_cmd
        return [prelift_cmd, intermediate_cmd, predrop_cmd, final_cmd]
    
    def move_2_to_OUT(self):
        previous = self._desired_EE_poses[1]
        desired = np.copy(previous)
        desired[0] += 2*self._delta_x
        prelift_cmd = self._generate_prelift_leg_cmd(2, previous)
        intermediate_cmd = self._generate_intermediate_leg_cmd(2, previous, desired)
        predrop_cmd = self._generate_predrop_leg_cmd(2, desired)
        self._desired_EE_poses[1] = desired
        final_cmd = self.desired_cmd
        return [prelift_cmd, intermediate_cmd, predrop_cmd, final_cmd]

    def move_3_to_IN(self):
        previous = self._desired_EE_poses[2]
        desired = np.copy(previous)
        desired[0] += 2*self._delta_x
        prelift_cmd = self._generate_prelift_leg_cmd(3, previous)
        intermediate_cmd = self._generate_intermediate_leg_cmd(3, previous, desired)
        predrop_cmd = self._generate_predrop_leg_cmd(3, desired)
        self._desired_EE_poses[2] = desired
        final_cmd = self.desired_cmd
        return [prelift_cmd, intermediate_cmd, predrop_cmd, final_cmd]
