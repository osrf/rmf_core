#ifndef RMF_TRAFFIC__SCHEDULE__VERSION_HPP
#define RMF_TRAFFIC__SCHEDULE__VERSION_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <cstdint>

namespace rmf_traffic {
namespace schedule {

/// The schedule version is represented by an unsigned 64-bit integer. This
/// means that the schedule can identify over 9 quintillion database entries
/// at any single moment in time (which is more than a server is likely to have
/// enough RAM to store).
///
/// The Version number is used to identify the current version of a database.
///
/// As database entries become irrelevant (e.g. they refer to events that have
/// already finished taking place) they will be culled from the database. The
/// database will keep track of what its "oldest" known version number is.
/// After a very long period of continuous operation, the version numbers could
/// eventually wrap around and overflow the 64-bit unsigned integer. This is
/// okay because modular arithmetic will be used to ensure that version values
/// which are lower than the "oldest" version number will be evaluated as
/// greater than any version numbers that are greater than the "oldest" version
/// number.
using Version = uint64_t;

/// A class to describe a filter on what version changes to query from a
/// schedule.
class Versions
{
public:

  /// The mode for how to filter versions in a schedule database query.
  enum class Mode : uint16_t
  {
    /// Invalid mode, behavior is undefined.
    Invalid,

    /// Get everything, regardless of version.
    All,

    /// Get every version after the specified one.
    After,
  };

  /// This is a placeholder class in case we ever want to extend the features
  /// of the `All` mode.
  class All
  {
  public:

    class Implementation;
  private:
    All();
    friend class Participants;
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// The interface for the Versions::After mode.
  class After
  {
  public:

    /// Constructor.
    After(Version version);

    /// Get the specified version. The Query will only return Trajectories
    /// which were introduced after this version of the schedule.
    Version get_version() const;

    /// Set the version.
    ///
    /// \param[in] version
    ///   The Query will only return Trajectories which were introduced after
    ///   this version of the schedule.
    After& set_version(Version version);

    class Implementation;
  private:
    /// Default constructor. This is not accessible to users because it leaves
    /// the After instance null.
    After();
    friend class Versions;
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Default constructor, uses All mode.
  Versions();

  /// Constructor to use After mode.
  ///
  /// \param[in] version
  ///   The Query will only return Trajectories which were introduced after
  ///   this version of the schedule.
  Versions(Version version);

  /// Get the current Versions mode of this query.
  Mode get_mode() const;

  /// Set the mode of this Versions interface to query for All Trajectories
  /// regardless of version.
  All& query_all();

  /// Set the mode of this Versions interface to query for only Trajectories
  /// that changed after the given version.
  ///
  /// \param[in] version
  ///   The Query will only return Trajectories which were introduced after
  ///   this version of the schedule.
  After& query_after(Version version);

  /// Get the Versions After interface to use for this Query. If this Versions
  /// is not in the After mode, then this will return a nullptr.
  After* after();

  /// const-qualified after()
  const After* after() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__VERSION_HPP
