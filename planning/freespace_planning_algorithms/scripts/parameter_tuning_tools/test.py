import sys

# sys.path.append("/home/takumiito/pilot-auto.freespace_param_tuning/build/freespace_planning_algorithms")

import freespace_planning_algorithms.freespace_planning_algorithms_python as fp

vehicle_shape = fp.VehicleShape(3, 2, 1)
astar_param = fp.AstarParam()
planner_param = fp.PlannerCommonParam()

astar = fp.AstarSearch(planner_param, vehicle_shape, astar_param)

print(astar)