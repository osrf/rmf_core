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

#include <rmf_traffic_ros2/Profile.hpp>
#include <rmf_traffic_ros2/geometry/ConvexShape.hpp>

#include <iostream>

namespace rmf_traffic_ros2 {

//==============================================================================
rmf_traffic::Profile convert(const rmf_traffic_msgs::msg::Profile& from)
{
  const geometry::ConvexShapeContext context = convert(from.shape_context);

  auto to = rmf_traffic::Profile{
      context.at(from.footprint),
      context.at(from.vicinity)
    };

  if (from.footprint.type != from.footprint.NONE)
    std::cout << " ==== MSG HAS A FOOTPRINT --> ";
  else
    std::cout << " ==== MSG NO FOOTPRINT --> ";

  if (to.footprint())
    std::cout << " PROFILE HAS A FOOTPRINT";
  else
    std::cout << " PROFILE NO FOOTPRINT";

  std::cout << std::endl;

  return to;
//  return rmf_traffic::Profile{
//    context.at(from.footprint),
//    context.at(from.vicinity)
//  };
}

//==============================================================================
rmf_traffic_msgs::msg::Profile convert(const rmf_traffic::Profile& from)
{
  geometry::ConvexShapeContext context;

  rmf_traffic_msgs::msg::Profile profile;
  profile.footprint = context.insert(from.footprint());
  profile.vicinity = context.insert(from.vicinity());
  profile.shape_context = convert(context);

  if (from.footprint())
    std::cout << " ==== PROFILE HAS A FOOTPRINT --> ";
  else
    std::cout << " ==== PROFILE NO FOOTPRINT --> ";

  if (profile.footprint.type != profile.footprint.NONE)
    std::cout << " MSG HAS A FOOTPRINT";
  else
    std::cout << " MSG NO FOOTPRINT";

  std::cout << std::endl;

  return profile;
}

} // namespace rmf_traffic_ros2
