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

#ifndef RMF_TRAFFIC__DEBUG__PLUMBER_HPP
#define RMF_TRAFFIC__DEBUG__PLUMBER_HPP

#include <string>

namespace rmf_traffic {
namespace debug {

//==============================================================================
class Plumber
{
public:

  Plumber(std::string name);
  ~Plumber();

private:
  std::string _name;
  void _print(char prefix) const;
};

} // namespace debug
} // namespace rmf_traffic

#define CHECK_LEAK(X) \
  std::shared_ptr<::rmf_traffic::debug::Plumber> _plumber = \
    std::make_shared<::rmf_traffic::debug::Plumber>(X)

#endif // PLUMBER_HPP
