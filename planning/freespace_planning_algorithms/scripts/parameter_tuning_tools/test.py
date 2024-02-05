import sys

# sys.path.append("/home/takumiito/pilot-auto.freespace_param_tuning/build/freespace_planning_algorithms")

import freespace_planning_algorithms.freespace_planning_algorithms_python as fp
import numpy as np
from pyquaternion import Quaternion

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from rclpy.serialization import serialize_message

vehicle_shape = fp.VehicleShape(3, 2, 1)
astar_param = fp.AstarParam()
planner_param = fp.PlannerCommonParam()


# base configs
planner_param.time_limit= 30000.0
planner_param.minimum_turning_radius= 9.0
planner_param.maximum_turning_radius= 9.0
planner_param.turning_radius_size= 1
# search configs
planner_param.theta_size= 144
planner_param.angle_goal_range= 6.0
planner_param.curve_weight= 1.2
planner_param.reverse_weight= 2.0
planner_param.lateral_goal_range= 0.5
planner_param.longitudinal_goal_range= 2.0
# costmap configs
planner_param.obstacle_threshold= 100

# -- A* search Configurations --
astar_param.only_behind_solutions= False
astar_param.distance_heuristic_weight= 1.0

astar = fp.AstarSearch(planner_param, vehicle_shape, astar_param)

costmap = OccupancyGrid()
costmap.info.resolution = 0.2
costmap.info.height = 350
costmap.info.width = 350
costmap.data = [0 for i in range(350*350)]
costmap_byte = serialize_message(costmap)

astar.setMap(costmap_byte)

start_pose = Pose()
goal_pose = Pose()

for x in range(-10,10):
    for y in range(-10,10):
        for yaw in range(10):
            # start_pose.position.x = 1.0
            # start_pose.orientation.y = 1.0
            goal_pose.position.x = float(x)
            goal_pose.position.y = float(y)
            
            quaterinon = Quaternion(axis=[0, 0, 1], angle=2*np.pi*(yaw/10))
            goal_pose.orientation.w = quaterinon.w
            goal_pose.orientation.x = quaterinon.x
            goal_pose.orientation.y = quaterinon.y
            goal_pose.orientation.z = quaterinon.z

            start_pose_byte = serialize_message(start_pose)
            goal_pose_byte = serialize_message(goal_pose)

            if astar.makePlan(start_pose_byte, goal_pose_byte):
                print("success")