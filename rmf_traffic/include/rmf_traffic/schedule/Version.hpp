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

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__VERSION_HPP
