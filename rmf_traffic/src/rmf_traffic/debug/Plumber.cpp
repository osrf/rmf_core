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

#include <rmf_traffic/debug/Plumber.hpp>

#include <iostream>
#include <sstream>

namespace rmf_traffic {
namespace debug {

//==============================================================================
Plumber::Plumber(std::string name)
  : _name(std::move(name))
{
  _print('+');
}

//==============================================================================
Plumber::~Plumber()
{
  _print('-');
}

//==============================================================================
void Plumber::_print(char prefix) const
{
  std::ostringstream oss;
  oss << prefix << this << "[" << _name << "]\n";
  std::cout << oss.str() << std::flush;
}

} // namespace debug
} // namespace rmf_traffic
