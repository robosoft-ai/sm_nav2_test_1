// Copyright 2021 RobosoftAI Inc.
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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <sm_nav2_test_1/clients/cl_april_tag_detector/cl_april_tag_detector.hpp>
#include <smacc2/smacc.hpp>

namespace sm_nav2_test_1 {
using namespace std::chrono_literals;

class OrPerception : public smacc2::Orthogonal<OrPerception> {
public:
  void onInitialize() override {
    auto assignerclient =
        this->createClient<cl_apriltag_detector::ClAprilTagDetector>();
    // auto roslaunch = this->createClient<smacc2::client_bases::ClRosLaunch>(
    // "sm_nav2_test_1",
    // "isaac_ros_apriltag_isaac_sim_pipeline.launch.py");
  }
};
} // namespace sm_nav2_test_1
