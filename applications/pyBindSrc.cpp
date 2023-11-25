//
// Created by hassan on 25/11/23.
//

#include <environment/environment_c100.hpp>
#include <graph/Policy.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
namespace py = pybind11;

typedef Policy<Env::ActionDim> PolicyWithActionDim;
PYBIND11_MODULE(legged_robotics, m) {
py::class_<Env::blind_locomotion>(m, "blind_locomotion")
.def(py::init<bool, int,std::string,std::string>(),py::arg("visualize"),py::arg("instance"),py::arg("urdf_path"),py::arg("actuator_path"))
.def("getHistory", static_cast<Eigen::MatrixXf (Env::blind_locomotion::*)(const size_t &)>(&Env::blind_locomotion::getHistory),py::arg("nums"))
.def("integrate", &Env::blind_locomotion::integrate)
.def("setFootFriction",&Env::blind_locomotion::setFootFriction,py::arg("idx"),py::arg("c_f"))
.def("getState",static_cast<Eigen::MatrixXf (Env::blind_locomotion::*)()>(&Env::blind_locomotion::getState))
.def("updateAction",(&Env::blind_locomotion::updateAction),py::arg("action"))
.def("updateTask",(&Env::blind_locomotion::updateTask),py::arg("input"))
.def("init",(&Env::blind_locomotion::init))
.def("add",(&Env::blind_locomotion::add),py::arg("i"),py::arg("j"));
py::class_<PolicyWithActionDim>(m, "policy")
.def(py::init<>())
.def("load",static_cast<void (PolicyWithActionDim::*)(std::string,std::string,int,int,int)>(&PolicyWithActionDim::load),py::arg("model_path"),py::arg("param_path"),py::arg("state_dim"),py::arg("history_dim"),py::arg("history_len"))
.def("updateStateBuffer",&PolicyWithActionDim::updateStateBuffer,py::arg("state"))
.def("updateStateBuffer2",&PolicyWithActionDim::updateStateBuffer2,py::arg("state2"))
.def("getAction",static_cast<Eigen::MatrixXf (PolicyWithActionDim::*)()>(&PolicyWithActionDim::getAction))
.def("add",(&PolicyWithActionDim::add),py::arg("i"),py::arg("j"));

}