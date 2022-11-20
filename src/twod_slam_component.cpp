// Copyright (c) 2022 OUXT Polaris
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

// Headers in this package
#include "ouagv_2d_slam/twod_slam_component.hpp"
// Components
#include <rclcpp_components/register_node_macro.hpp>

// Headers needed in this component

namespace twod_slam
{
    TwodSlamComponent::TwodSlamComponent(const rclcpp::NodeOptions &options)
        : Node("twod_slam_node", options)
    {
    }

} // namespace twod_slam

RCLCPP_COMPONENTS_REGISTER_NODE(twod_slam::TwodSlamComponent)