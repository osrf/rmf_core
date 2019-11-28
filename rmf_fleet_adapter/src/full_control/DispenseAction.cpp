/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "FleetAdapterNode.hpp"

namespace rmf_fleet_adapter {
namespace full_control {

namespace {
//==============================================================================
class DispenseAction : public Action
{
public:



};

} // anonymous namespace

//==============================================================================
std::unique_ptr<Action> make_dispense(
    FleetAdapterNode* node,
    const rmf_task_msgs::msg::Behavior& behavior,
    const std::string& dispenser_name)
{

}

} // namespace full_control
} // namespace rmf_fleet_adapter