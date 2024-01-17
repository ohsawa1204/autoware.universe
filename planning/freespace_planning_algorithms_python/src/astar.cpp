#include <freespace_planning_algorithms/astar_search.hpp>
// #include <freespace_planning_algorithms/abstract_algorithm.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pybind11/pybind11.hpp>

namespace py = pybind11;
using namespace freespace_planning_algorithms;

PYBIND11_MODULE(PYTHON_API_MODULE_NAME, p)
{
    py::class_<AbstractPlanningAlgorithm>(p, "AbstractPlanningAlgorithm")
            .def(py::init<>());

    py::class_<AstarSearch, AbstractPlanningAlgorithm>(p, "AstarSearch")
            .def(py::init<PlannerCommonParam &, VehicleShape &, AstarParam &>());

    py::class_<AstarParam>(p, "AstarParam")
            .def("only_behind_solutions", AstarParam::only_behind_solutions)
            .def("use_back", AstarParam::use_back)
            .def("distance_heuristic_weight", AstarParam::distance_heuristic_weight);
    py::class_<PlannerCommonParam>(p, "PlannerCommonParam")
            .def("time_limit", PlannerCommonParam::time_limit)
            .def("minimum_tuning_radius", PlannerCommonParam::time_limit)
            .def("maximum_tuning_radius", PlannerCommonParam::time_limit)
            .def("turning_radius_size", PlannerCommonParam::turning_radius_size)
            .def("theta_size", PlannerCommonParam::theta_size)
            .def("curve_weight", PlannerCommonParam::curve_weight)
            .def("reverse_weight", PlannerCommonParam::reverse_weight)
            .def("lateral_goal_range", PlannerCommonParam::lateral_goal_range)
            .def("longitudinal_goal_range", PlannerCommonParam::longitudinal_goal_range)
            .def("angle_goal_range", PlannerCommonParam::angle_goal_range)
            .def("obstacle_threshold", PlannerCommonParam::obstacle_threshold);
    py::class_<VehicleShape>(p, "VehicleShape")
            .def(py::init<>())
            .def(py::init<double, double, double>())
            .def("length", VehicleShape::length)
            .def("width", VehicleShape::time_limit)
            .def("base2back", VehicleShape::time_limit);
}

// #include <freespace_planning_algorithms/astar_search.hpp>
// #include <freespace_planning_algorithms/abstract_algorithm.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <boost/python.hpp>
// #include <boost/bind/bind.hpp>


// using namespace freespace_planning_algorithms;

// BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME)
// {
//   using namespace boost::python;
//   class_<AstarSearch, bases<AbstractPlanningAlgorithm>>(
//     "AstarSeach", init<PlannerCommonParam, VehicleShape, AstarParam>())
//     // .def(init<PlannerCommonParam, VehicleShape, rclcpp::Node>())
//     .def("set_map", &AstarSearch::setMap)
//     .def("make_plan", &AstarSearch::makePlan);
// }