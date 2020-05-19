/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "Transport.hpp"

void Transport::start()
{
  if (!_stopping)
    return;
  _stopping = false;
  _spin_thread = std::thread{[this]() { _do_spin(); }};
}

void Transport::stop()
{
  _stopping = true;
  if (_spin_thread.joinable())
    _spin_thread.join();
}

void Transport::_do_spin()
{
  while (!_stopping)
    rclcpp::spin_some(shared_from_this());
}