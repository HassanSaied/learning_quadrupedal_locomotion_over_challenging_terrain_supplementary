//
// Created by joonho on 7/28/20.
//

#include <environment/environment_c100.hpp>
#include <graph/Policy.hpp>
#include <filesystem>

using namespace std;

#include <sstream>

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return std::move(out).str();
}

template<int rows, int columns>
void print_matrices_vector(vector<Eigen::Matrix<float, rows, columns>> &v, const filesystem::path &output_path) {
    auto *output_arr = new float[(v.size() * v[0].rows() * v[0].cols()) + 3];
    int curr_idx = 0;
    output_arr[curr_idx++] = v.size();
    output_arr[curr_idx++] = v[0].rows();
    output_arr[curr_idx++] = v[0].cols();
    for (auto &matrix: v) {
        for (int i = 0; i < matrix.rows(); ++i) {
            for (int j = 0; j < matrix.cols(); ++j) {
                output_arr[curr_idx] = matrix(i, j);
                curr_idx += 1;
            }
        }
    }
    ofstream output_file(output_path, std::ios::binary);
    output_file.write((const char *) output_arr, curr_idx * sizeof(float));
    output_file.close();
    delete[] output_arr;
}

void perform_simulation(Env::blind_locomotion &sim, Policy<Env::ActionDim> &policy, const int history_len,
                        Eigen::Matrix<float, -1, 1> &task_params) {

    Eigen::Matrix<float, -1, -1> history;
    Eigen::Matrix<float, -1, 1> state;
    Eigen::Matrix<float, Env::ActionDim, 1> action;

    sim.updateTask(task_params);
    sim.setFootFriction(0, 0.1);

    sim.init();
    vector<Eigen::Matrix<float, -1, -1>> all_history;
    vector<Eigen::Matrix<float, -1, 1>> all_state;
    vector<Eigen::Matrix<float, Env::ActionDim, 1>> all_action;
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    sim.detect_collisions();
    /// simulate for 30 seconds.
    for (int i = 0; i < 1500; i++) {
        sim.integrate();
        sim.getHistory(history, history_len);
        sim.getState(state);
        sim.getPriviligedState();
        all_history.push_back(history);
        all_state.push_back(state);

        policy.updateStateBuffer(state);
        policy.updateStateBuffer2(history);
        policy.getAction(action);
        all_action.push_back(action);
        sim.updateAction(action);
    }
    string dir_name = "Task[";

    for (float task_param : task_params) {
        dir_name += to_string_with_precision(task_param,2) + ",";

    }
    dir_name[dir_name.length()-1] = ']';
    auto dir_path = filesystem::path(dir_name);
    filesystem::create_directories(dir_path);
    print_matrices_vector(all_history, dir_path/"History.bin");
    print_matrices_vector(all_state, dir_path/"State.bin");
    print_matrices_vector(all_action, dir_path/"Action.bin");
}

int main(int argc, char *argv[]) {


    ///Hard-coded. TODO: clean it up
    int history_len = 100; // todo: move to controller config.
    std::string rsc_path = "/home/hassan/Documents/learning_quadrupedal_locomotion_over_challenging_terrain_supplementary/rsc";

    std::string urdf_path = rsc_path + "/robot/c100/urdf/anymal_minimal.urdf";
    std::string actuator_path = rsc_path + "/actuator/C100/seaModel_2500.txt";
    std::string network_path = rsc_path + "/controller/c100/graph.pb";
    std::string param_path = rsc_path + "/controller/c100/param.txt";

    raisim::World::setActivationKey("/home/hassan/.raisim/activation.raisim");

    Env::blind_locomotion sim(true, 0, urdf_path, actuator_path);

    Policy<Env::ActionDim> policy;

    policy.load(network_path,
                param_path,
                Env::StateDim,
                Env::ObservationDim,
                history_len);

    Eigen::Matrix<float, -1, -1> history;
    Eigen::Matrix<float, -1, 1> state;
    Eigen::Matrix<float, Env::ActionDim, 1> action;

    /// set terrain properties
    Eigen::Matrix<float, -1, 1> task_params(4);
    task_params << 0.0, 0.05, 0.5, 0.5;
    perform_simulation(sim, policy, history_len, task_params);
    return 0;
    task_params << 5, 0.05, 0.5, 0.5;
    perform_simulation(sim, policy, history_len, task_params);
    task_params << 5, 0.03, 0.6, 1.4;
    perform_simulation(sim, policy, history_len, task_params);
    task_params << 3, 0.03, 0.6, 1.4;
    perform_simulation(sim, policy, history_len, task_params);
    task_params << 3, 0.05, 0.5, 0.5;
    perform_simulation(sim, policy, history_len, task_params);
    task_params << 6, 0.3, 0.12, 0.5;
    perform_simulation(sim, policy, history_len, task_params);
    task_params << 7, 0.1, 0.02, 0;
    perform_simulation(sim, policy, history_len, task_params);
    return 0;
}

