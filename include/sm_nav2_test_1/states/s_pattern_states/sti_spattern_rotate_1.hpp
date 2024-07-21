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

namespace sm_nav2_test_1 {
namespace s_pattern_states {
// STATE DECLARATION
struct StiSPatternRotate1 : smacc2::SmaccState<StiSPatternRotate1, SS> {
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>,
                 StiSPatternForward1>,
      Transition<EvCbFailure<CbAbsoluteRotate, OrNavigation>,
                 StiSPatternRotate1>

      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    float offset = 0;
    if (SS::direction() == TDirection::RIGHT) {
      // - offset because we are looking to the north and we have to turn
      // clockwise
      configure_orthogonal<OrNavigation, CbAbsoluteRotate>(0 - offset);
    } else {
      // - offset because we are looking to the south and we have to turn
      // counter-clockwise
      configure_orthogonal<OrNavigation, CbAbsoluteRotate>(180 + offset);
    }
  }

  void runtimeConfigure() {}
};
} // namespace s_pattern_states
} // namespace sm_nav2_test_1
