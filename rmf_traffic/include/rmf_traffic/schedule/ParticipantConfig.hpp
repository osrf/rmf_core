#ifndef PARTICIPANT_CONIFG_H
#define PARTICIPANT_CONFIG_H

#include <rmf_traffic/schedule/ParticipantDescription.hpp>

namespace rmf_traffic {
namespace schedule {
  class ParticipantConfig 
  {
  public:
    virtual void was_updated(
      const std::unordered_map<ParticipantId, ParticipantDescription>& participants
      ) = 0;
  };
} // end namespace schedule
} // end namespace rmf_traffic

#endif