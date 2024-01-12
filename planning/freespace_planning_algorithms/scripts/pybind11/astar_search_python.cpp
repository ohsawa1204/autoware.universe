#include "freespace_planning_algorithms/astar_search.hpp"
#include "freespace_planning_algorithms/abstract_algorithm.hpp"
#include <rclcpp/rclcpp.hpp>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace freespace_planning_algorithms;

PYBIND11_MODULE(freespace_planning_algorithms_python, p)
{
    py::class_<AbstractPlanningAlgorithm>(p, "AbstractPlanningAlgorithm");

    py::class_<AstarSearch, AbstractPlanningAlgorithm>(p, "AstarSearch")
            .def(py::init<PlannerCommonParam &, VehicleShape &, AstarParam &>());

    py::class_<AstarParam>(p, "AstarParam", py::dynamic_attr())
            .def_readwrite("only_behind_solutions", &AstarParam::only_behind_solutions)
            .def_readwrite("use_back", &AstarParam::use_back)
            .def_readwrite("distance_heuristic_weight", &AstarParam::distance_heuristic_weight);
    py::class_<PlannerCommonParam>(p, "PlannerCommonParam", py::dynamic_attr())
            .def_readwrite("time_limit", &PlannerCommonParam::time_limit)
            .def_readwrite("minimum_tuning_radius", &PlannerCommonParam::time_limit)
            .def_readwrite("maximum_tuning_radius", &PlannerCommonParam::time_limit)
            .def_readwrite("turning_radius_size", &PlannerCommonParam::turning_radius_size)
            .def_readwrite("theta_size", &PlannerCommonParam::theta_size)
            .def_readwrite("curve_weight", &PlannerCommonParam::curve_weight)
            .def_readwrite("reverse_weight", &PlannerCommonParam::reverse_weight)
            .def_readwrite("lateral_goal_range", &PlannerCommonParam::lateral_goal_range)
            .def_readwrite("longitudinal_goal_range", &PlannerCommonParam::longitudinal_goal_range)
            .def_readwrite("angle_goal_range", &PlannerCommonParam::angle_goal_range)
            .def_readwrite("obstacle_threshold", &PlannerCommonParam::obstacle_threshold);
    py::class_<VehicleShape>(p, "VehicleShape", py::dynamic_attr())
            .def(py::init<>())
            .def(py::init<double, double, double>())
            .def_readwrite("length", &VehicleShape::length)
            .def_readwrite("width", &VehicleShape::width)
            .def_readwrite("base2back", &VehicleShape::base2back);
}
