#ifndef RMF_TRAFFIC__BLOCKADE__WRITER_HPP
#define RMF_TRAFFIC__BLOCKADE__WRITER_HPP

#include <Eigen/Geometry>

#include <rmf_traffic/Time.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
class Writer
{
public:

  struct Item
  {
    uint64_t id;
    Eigen::Vector2d start;
    Eigen::Vector2d finish;
    double radius;
    std::array<rmf_traffic::Time, 2> time_range;
  };



};

} // namespace blockade
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__BLOCKADE__WRITER_HPP
