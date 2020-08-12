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

#ifndef RMF_BATTERY__AGV__SYSTEMTRAITS_HPP
#define RMF_BATTERY__AGV__SYSTEMTRAITS_HPP

#include<vector>
#include<rmf_utils/impl_ptr.hpp>
#include<rmf_utils/optional.hpp>

namespace rmf_battery {
namespace agv {

//==============================================================================
class SystemTraits
{
public:
  class PowerSystem;
  using PowerSystems = std::vector<PowerSystem>;

  class PowerSystem
  {
  public:

    PowerSystem(
      double nominal_power,
      double nominal_voltage,
      double efficiency = 1.0);

    PowerSystem& nominal_power(double nom_power);
    double nominal_power() const;

    PowerSystem& nominal_voltage(double nom_voltage);
    double nominal_voltage() const;

    PowerSystem& efficiency(double efficiency);
    double efficiency() const;

    /// Returns true if the values are valid, i.e. greater than zero.
    bool valid() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  class BatterySystem
  {
  public:
    enum class BatteryType : uint16_t
    {
        /// The vehicle is powered by a Lead-Acid battery
        LeadAcid,
        /// The vehicle is powered by a Lithium-Ion battery
        LiIon,
    };

    class BatteryProfile
    {
    public:
      BatteryProfile(
        double resistance,
        double max_voltage,
        double exp_voltage,
        double exp_capacity);

      BatteryProfile& resistance(double resistance);
      double resistance() const;

      BatteryProfile& max_voltage(double max_voltage);
      double max_voltage() const;

      BatteryProfile& exp_voltage(double exp_voltage);
      double exp_voltage() const;

      BatteryProfile& exp_capacity(double exp_capacity);
      double exp_capacity() const;

      /// Returns true if the values are valid, i.e. greater than zero.
      bool valid() const;
      
      class Implementation;
    private:
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    BatterySystem(
      double nominal_voltage,
      double nominal_capacity,
      double charging_current,
      BatteryType type = BatteryType::LeadAcid,
      rmf_utils::optional<BatteryProfile> profile = rmf_utils::nullopt);

    BatterySystem& nominal_voltage(double nominal_voltage);
    double nominal_voltage() const;

    BatterySystem& nominal_capacity(double nominal_capacity);
    double nominal_capacity() const;

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

  class MechanicalSystem
  {
  public:

    MechanicalSystem(
      double mass,
      double friction_coefficient,
      double drag_coefficient,
      double frontal_area);

    MechanicalSystem& mass(double mass);
    double mass() const;

    MechanicalSystem& friction_coefficient(double friction_coeff);
    double friction_coefficient() const;

    MechanicalSystem& drag_coefficient(double drag_coeff);
    double drag_coefficient() const;

    MechanicalSystem& frontal_area(double frontal_area);
    double frontal_area() const;

    /// Returns true if the values are valid, i.e. greater than zero.
    bool valid() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Constructor.
  SystemTraits(
    MechanicalSystem mechanical_system,
    BatterySystem battery_system,
    PowerSystems power_systems);

  SystemTraits& mechanical_system(MechanicalSystem mechanical_system);
  const MechanicalSystem mechanical_system() const;

  SystemTraits& battery_system(BatterySystem battery_system);
  const BatterySystem battery_system() const;

  SystemTraits& power_systems(PowerSystems power_systems);
  const PowerSystems power_systems() const;

  /// Returns true if the values of the traits are valid. For example, this
  /// means that all system values are greater than zero.
  bool valid() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;

};

} // namespace agv
} // namespace rmf_battery

#endif // RMF_BATTERY__AGV__SYSTEMTRAITS_HPP
