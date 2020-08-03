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

    PowerSystem& set_nominal_power(double nom_power);
    double get_nominal_power() const;

    PowerSystem& set_nominal_voltage(double nom_voltage);
    double get_nominal_voltage() const;

    PowerSystem& set_efficiency(double efficiency);
    double get_efficiency() const;

    /// Returns true if the values of these limits are valid, i.e. greater than
    /// zero.
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

    BatterySystem(
      double nominal_voltage,
      double nominal_capacity,
      double nominal_current,
      BatteryType type = BatteryType::LeadAcid;
      rmf_utils::optional<double> resistance = rmf_utils::nullopt,
      rmf_utils::optional<double> max_voltage = rmf_utils::nullopt,
      rmf_utils::optional<double> exp_voltage = rmf_utils::nullopt,
      rmf_utils::optional<double> exp_capacity = rmf_utils::nullopt)


    bool valid() const;

  };

  class MechanicalSystem
  {
  public:

    MechanicalSystem(
      double mass,
      double static_friction,
      double dynamic_friction,
      double drag_coefficient,
      double frontal_area);

    Differential& set_forward(Eigen::Vector2d forward);

    const Eigen::Vector2d& get_forward() const;

    Differential& set_reversible(bool reversible);
    bool is_reversible() const;

    /// Returns true if the length of the forward vector is not too close to
    /// zero. If it is too close to zero, then the direction of the forward
    /// vector cannot be reliably interpreted. Ideally the forward vector should
    /// have unit length.
    bool valid() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Constructor.
  SystemTraits(
    MechanicalSystem mechanical_system,
    BatterySystem battery_system,
    Powersystems power_systems);

  MechanicalSystem& mechanical_system();
  const MechanicalSystem& mechanical_system() const;

  BatterySystem& mechanical_system();
  const BatterySystem& mechanical_system() const;

  Powersystems& mechanical_system();
  const Powersystems& mechanical_system() const;

  /// Returns true if the values of the traits are valid. For example, this
  /// means that all power and voltage values are greater than zero.
  bool valid() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;

};

} // namespace agv
} // namespace rmf_battery

#endif // RMF_BATTERY__AGV__SYSTEMTRAITS_HPP
