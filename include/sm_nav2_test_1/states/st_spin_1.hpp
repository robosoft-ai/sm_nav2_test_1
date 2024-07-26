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
using namespace cl_keyboard;
using namespace cl_nav2z;

// STATE DECLARATION
struct StSpin1 : smacc2::SmaccState<StSpin1, MsNav2Test1RunMode> {
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StSpin2,
                 SUCCESS>,
      Transition<EvCbFailure<CbAbsoluteRotate, OrNavigation>, StSpin1,
                 ABORT>,

      // Keyboard events  
      Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StSpin2, NEXT>
      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    configure_orthogonal<OrNavigation, CbAbsoluteRotate>();
    configure_orthogonal<OrNavigation, CbResumeSlam>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {
    auto cbAbsRotate =
        this->getClientBehavior<OrNavigation, CbAbsoluteRotate>();

    auto angleSetValue = 360;

    cbAbsRotate->spinningPlanner = SpinningPlanner::PureSpinning;
    cbAbsRotate->absoluteGoalAngleDegree = angleSetValue;
  }
};
} // namespace sm_nav2_test_1
