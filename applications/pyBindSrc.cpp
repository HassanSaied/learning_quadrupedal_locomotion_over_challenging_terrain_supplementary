//
// Created by hassan on 25/11/23.
//

#include <environment/environment_c100.hpp>
#include <graph/Policy.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

typedef Policy<Env::ActionDim> PolicyWithActionDim;
PYBIND11_MODULE(legged_robotics, m) {
    py::class_<Env::blind_locomotion>(m, "blind_locomotion")
            .def(py::init<bool, int, std::string, std::string>(), py::arg("visualize"), py::arg("instance"),
                 py::arg("urdf_path"), py::arg("actuator_path"))
            .def("getHistory", static_cast<Eigen::MatrixXf (Env::blind_locomotion::*)(const size_t &)>(&Env::blind_locomotion::getHistory), py::arg("nums"))
            .def("integrate", &Env::blind_locomotion::integrate)
            .def("setFootFriction", &Env::blind_locomotion::setFootFriction, py::arg("idx"), py::arg("c_f"))
            .def("getState",static_cast<Eigen::MatrixXf (Env::blind_locomotion::*)()>(&Env::blind_locomotion::getState))
            .def("getPriviligedState",static_cast<Eigen::MatrixXf (Env::blind_locomotion::*)()>(&Env::blind_locomotion::getPriviligedState))
            .def("updateAction", (&Env::blind_locomotion::updateAction), py::arg("action"))
            .def("updateTask", (&Env::blind_locomotion::updateTask), py::arg("input"))
            .def("init", (&Env::blind_locomotion::init))
            .def("getStateWithoutNoise", (&Env::blind_locomotion::getStateWithoutNoise))
            .def("detect_collisions",(&Env::blind_locomotion::detect_collisions))
            .def("get_torques",((&Env::blind_locomotion::get_torques)))
            .def("get_base_orientation",((&Env::blind_locomotion::get_base_orientation)))
            .def("get_termination_condition",&Env::blind_locomotion::get_termination_condition,py::arg("body_names_to_check"))
            .def("add", (&Env::blind_locomotion::add), py::arg("i"), py::arg("j"))
            .def_readonly("numContact_", &Env::blind_locomotion::numContact_)
            .def_readonly("numFootContact_", &Env::blind_locomotion::numFootContact_)
            .def_readonly("numShankContact_", &Env::blind_locomotion::numShankContact_)
            .def_readonly("numThighContact_", &Env::blind_locomotion::numThighContact_)
            .def_readonly("numBaseContact_", &Env::blind_locomotion::numBaseContact_)
            .def_readonly("numInternalContact_", &Env::blind_locomotion::numInternalContact_)
            .def_readonly("netFootContacts_", &Env::blind_locomotion::netFootContacts_)
            .def_readonly("netFootContacts_b", &Env::blind_locomotion::netFootContacts_b)
            .def_readonly("netFootContactVels_", &Env::blind_locomotion::netFootContactVels_)
            .def_readonly("netContacts_", &Env::blind_locomotion::netContacts_)
            .def_readonly("netContacts_b", &Env::blind_locomotion::netContacts_b)
            .def_readonly("FootContactNums_", &Env::blind_locomotion::FootContactNums_)
            .def_readonly("footPos_Target", &Env::blind_locomotion::footPos_Target)
            .def_readonly("footContactState_", &Env::blind_locomotion::footContactState_)
            .def_readonly("shankContacts_", &Env::blind_locomotion::shankContacts_)
            .def_readonly("thighContacts_", &Env::blind_locomotion::thighContacts_)
            .def_readonly("footNormal_", &Env::blind_locomotion::footNormal_)
            .def_readonly("footNormal_b", &Env::blind_locomotion::footNormal_b)
            .def_readonly("footVel_projected", &Env::blind_locomotion::footVel_projected);
            py::class_<PolicyWithActionDim>(m, "policy")
            .def(py::init<>())
            .def("load", static_cast<void (PolicyWithActionDim::*)(std::string, std::string, int, int,int)>(&PolicyWithActionDim::load),
             py::arg("model_path"), py::arg("param_path"), py::arg("state_dim"), py::arg("history_dim"),
             py::arg("history_len"))
            .def("updateStateBuffer", &PolicyWithActionDim::updateStateBuffer, py::arg("state"))
            .def("updateStateBuffer2", &PolicyWithActionDim::updateStateBuffer2, py::arg("state2"))
            .def("getAction", static_cast<Eigen::MatrixXf (PolicyWithActionDim::*)()>(&PolicyWithActionDim::getAction))
            .def("add", (&PolicyWithActionDim::add), py::arg("i"), py::arg("j"));

}