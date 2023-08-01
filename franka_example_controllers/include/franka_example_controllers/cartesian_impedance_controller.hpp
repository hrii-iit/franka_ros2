// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/frames.hpp"
// #include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
// #include "pinocchio/algorithm/crba.hpp"

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The joint impedance example controller moves joint 4 and 5 in a very compliant periodic movement.
 */
class CartesianImpedanceController : public controller_interface::ControllerInterface {
    public:
        using Vector7d = Eigen::Matrix<double, 7, 1>;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::return_type update(const rclcpp::Time& time,
                                                const rclcpp::Duration& period) override;
        CallbackReturn on_init() override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    private:
        std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;

        std::string arm_id_;
        const int num_joints = 7;
        Vector7d q_;
        Vector7d initial_q_;
        Vector7d dq_;
        Vector7d dq_filtered_;
        Vector7d k_gains_;
        Vector7d d_gains_;
        rclcpp::Time start_time_;
        void updateJointStates();

        // Pinocchio model and data
        std::string robot_description_;
        std::shared_ptr<pinocchio::Model> pinocchio_model_ptr_;
        std::shared_ptr<pinocchio::Data> pinocchio_data_ptr_;
        // Eigen::VectorXd pinocchio_joint_positions_;
        // Eigen::VectorXd pinocchio_joint_velocities_;
        int pinocchio_dof_;
        Vector7d gravity_;
        bool initModelInterface();
        bool updateModelInterface();
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
        void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);
};

}  // namespace franka_example_controllers