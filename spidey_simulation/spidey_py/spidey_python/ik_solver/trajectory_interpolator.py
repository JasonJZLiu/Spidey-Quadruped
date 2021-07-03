
import numpy as np
import copy

def interpolate_between_waypoints(current_cmd, next_cmd):
  diff_vector_leg = []
  leg_norm = 0

  for i in range(4):
    diff_vector_leg.append(next_cmd["desired_feet_poses"][i] - current_cmd["desired_feet_poses"][i])
    leg_norm += np.linalg.norm(diff_vector_leg[i])

  diff_vector_body = next_cmd["desired_base_link_pose"] - current_cmd["desired_base_link_pose"]

  body_norm = np.linalg.norm(diff_vector_body)
  iteration = round(max(leg_norm, body_norm)/0.01)#0.01

  increment_vector_leg = []
  for i in range(4):
    increment_vector_leg.append(diff_vector_leg[i]/iteration)
  increment_vector_body = diff_vector_body/iteration


  trajectory = [copy.deepcopy(current_cmd)] 

  for i in range(iteration-1):
    for j in range(4):
      current_cmd["desired_feet_poses"][j] += increment_vector_leg[j]

    current_cmd["desired_base_link_pose"] += increment_vector_body

    trajectory.append(copy.deepcopy(current_cmd))

  # print("Iteration Number: ", iteration)
  # print("Computed Trajectory: ", trajectory)
  # print("final cmd", next_cmd)
  # print("\n")

  return trajectory


def interpolate_trajectory(waypoints):
  if len(waypoints) < 2:
    print("At least two waypoints are required for trajectory interpolation")
  else:
    trajectory = []
    for i in range(len(waypoints)-1):
      trajectory += interpolate_between_waypoints(waypoints[i], waypoints[i+1])
    
    trajectory.append(waypoints[-1])
  
    return trajectory






