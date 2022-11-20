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

#pragma once

#include "ouagv_2d_slam/visibility_control_twod_slam.h"

// Headers in ROS2
#include <rclcpp/rclcpp.hpp>
#include <memory> // shared_ptr in pub_

namespace twod_slam
{
    class TwodSlamComponent : public rclcpp::Node
    {
    public:
        TWOD_SLAM_TWOD_SLAM_COMPONENT_PUBLIC
        explicit TwodSlamComponent(const rclcpp::NodeOptions &options);

    private:
        void aaaa();
    };
}