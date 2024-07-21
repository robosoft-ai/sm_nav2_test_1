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

#include <smacc2/smacc.hpp>
// #include <nav2z_client/client_behavior>

namespace sm_nav2_test_1 {
using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using namespace std::chrono_literals;
using namespace cl_nav2z;
using namespace cl_keyboard;


// STATE DECLARATION
struct StNavigateToWaypoint1 : smacc2::SmaccState<StNavigateToWaypoint1, MsIsaacExplorationRunMode>
{
  using SmaccState::SmaccState;

   // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StNavigateWarehouseWaypointsX, SUCCESS>,
    //Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StNavigateWarehouseWaypointsX, ABORT>

    //Keyboard events
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateWarehouseWaypointsX, NEXT>,
    Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StInitialMoveStop, PREVIOUS>
    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {

    // x: 0.0 #-2.0
    // y: 5.25 # 0.5
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(-5.0,-5.0, 0.0);
    configure_orthogonal<OrNavigation, CbResumeSlam>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }
};
}  // namespace sm_nav2_test_1
