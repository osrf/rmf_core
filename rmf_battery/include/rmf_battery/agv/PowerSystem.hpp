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

#ifndef RMF_BATTERY__AGV__POWERSYSTEM_HPP
#define RMF_BATTERY__AGV__POWERSYSTEM_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <optional>

namespace rmf_battery {
namespace agv {


class PowerSystem
{
public:

  /// Returns a PowerSystem object if valid values were supplied for the
  /// various fields else returns std::nullopt. Here valid implies that the
  /// values are greater than zero.
  /// \param[in] nominal_power
  ///   The rated nominal power consumption in Watts for this power system
  static std::optional<PowerSystem> make(double nominal_power);

  /// Get the nominal power of this power system
  double nominal_power() const;

  class Implementation;
private:
  PowerSystem();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

using PowerSystemPtr = std::shared_ptr<PowerSystem>;
using ConstPowerSystemPtr = std::shared_ptr<const PowerSystem>;

} // namespace agv
} // namespace rmf_battery

#endif // RMF_BATTERY__AGV__POWERSYSTEM_HPP
