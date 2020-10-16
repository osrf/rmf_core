
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

#ifndef RMF_BATTERY__AGV__BATTERYSYSTEM_HPP
#define RMF_BATTERY__AGV__BATTERYSYSTEM_HPP

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_utils/optional.hpp>

namespace rmf_battery {
namespace agv {

class BatterySystem
{
public:
  enum class BatteryType : uint16_t
  {
    /// The vehicle is powered by a Lead-Acid battery
    LeadAcid,
    /// The vehicle is powered by a Lithium-Ion battery
    LiIon,
    /// The vehicle is powered by a Nickel-Metal-Hydride battery
    NiMH
  };

  class BatteryProfile
  {
  public:
    /// The parameters required to construct a generic battery model to map
    /// state of charge of the battery to its voltage.
    ///
    /// \param[in] resistance
    ///   The internal resistance of the battery in ohms
    ///
    /// \param[in] max_voltage
    ///   The maximum voltage in Volts of the battery
    ///
    /// \param[in] exp_voltage
    ///   The voltage of the battery in Volts at the end of the exponential 
    ///   zone in its discharge profile
    ///
    /// \param[in] exp_capacity
    ///   The capacity of the battery in Ah at the end of the exponential zone
    ///   in its discharge profile
    ///
    /// \param[in] nominal_capacity
    ///   The capacity of the battery in Ah at the end of the exponential zone
    ///   in its discharge profile
    BatteryProfile(
      double resistance,
      double max_voltage,
      double exp_voltage,
      double exp_capacity,
      double nominal_capacity,
      double discharge_current);

    BatteryProfile& resistance(double resistance);
    double resistance() const;

    BatteryProfile& max_voltage(double max_voltage);
    double max_voltage() const;

    BatteryProfile& exp_voltage(double exp_voltage);
    double exp_voltage() const;

    BatteryProfile& exp_capacity(double exp_capacity);
    double exp_capacity() const;

    BatteryProfile& nominal_capacity(double nominal_capacity);
    double nominal_capacity() const;

    BatteryProfile& discharge_current(double discharge_current);
    double discharge_current() const;

    /// Returns true if the values are valid, i.e. greater than zero.
    bool valid() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Constructor
  ///
  /// \param[in] nominal_voltage
  ///   The nominal voltage of the battery in volts
  ///
  /// \param[in] capacity
  ///   The nominal capacity of the battery in Ah
  ///
  /// \param[in] charging_current
  ///   The rated current in A for charging the battery
  ///
  /// \param[in] type
  ///   The chemisty type of the battery
  ///
  /// \param[in] profile
  ///   The battery profile for this battery. This is only required for calling
  ///   the get_voltage() method
  BatterySystem(
    double nominal_voltage,
    double capacity,
    double charging_current,
    BatteryType type = BatteryType::LeadAcid,
    rmf_utils::optional<BatteryProfile> profile = rmf_utils::nullopt);

  /// Estimate the voltage of the battery given its state of charge. This function
  /// will return a nullopt if the battery profile is not defined
  rmf_utils::optional<double> estimate_voltage(const double soc) const;

  BatterySystem& nominal_voltage(double nominal_voltage);
  double nominal_voltage() const;

  BatterySystem& capacity(double nominal_capacity);
  double capacity() const;

  BatterySystem& charging_current(double charging_current);
  double charging_current() const;

  BatterySystem& type(BatteryType type);
  BatteryType type() const;

  BatterySystem& profile(rmf_utils::optional<BatteryProfile> profile);
  rmf_utils::optional<BatteryProfile> profile() const;

  /// Returns true if the values are valid, i.e. greater than zero.
  bool valid() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

using BatterySystemPtr = std::shared_ptr<BatterySystem>;
using ConstBatterySystemPtr = std::shared_ptr<const BatterySystem>;

} // namespace agv
} // namespace rmf_battery

#endif // RMF_BATTERY__AGV__BATTERYSYSTEM_HPP
