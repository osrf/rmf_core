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

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Negotiation.hpp>
#include <rmf_traffic/schedule/Negotiator.hpp>

#include <rmf_utils/catch.hpp>

#include <iostream>

namespace {
//==============================================================================
struct MockNegotiator : public rmf_traffic::schedule::Negotiator
{
  virtual void respond(
      std::shared_ptr<const rmf_traffic::schedule::Negotiation::Table>,
      const Responder& responder,
      const bool* = nullptr) final
  {
    if (_submit)
      responder.submit({});
    else
      responder.reject();
  }

  MockNegotiator& submit()
  {
    _submit = true;
    return *this;
  }

  MockNegotiator& reject()
  {
    _submit = false;
    return *this;
  }

  bool _submit = true;
};

//==============================================================================
struct Table
{
  std::vector<rmf_traffic::schedule::ParticipantId> to_accommodate;
  rmf_traffic::schedule::ParticipantId for_participant;
};

//==============================================================================
void apply_submissions(
    const std::shared_ptr<rmf_traffic::schedule::Negotiation>& negotiation,
    const std::vector<Table>& submitted)
{
  using Responder = rmf_traffic::schedule::SimpleResponder;
  for (const auto& table : submitted)
  {
    MockNegotiator().submit().respond(
          negotiation->table(table.for_participant, table.to_accommodate),
          Responder(negotiation, table.for_participant, table.to_accommodate));
  }
}

//==============================================================================
void apply_rejection(
    const std::shared_ptr<rmf_traffic::schedule::Negotiation>& negotiation,
    const std::vector<Table>& rejected)
{
  using Responder = rmf_traffic::schedule::SimpleResponder;
  for (const auto& table : rejected)
  {
    MockNegotiator().reject().respond(
          negotiation->table(table.for_participant, table.to_accommodate),
          Responder(negotiation, table.for_participant, table.to_accommodate));
  }
}

} // anonymous namespace

//==============================================================================
SCENARIO("Identify a failed negotiation")
{
  // This situation emerged during testing and revealed some bugs that can occur
  // while adding participants.
  rmf_traffic::schedule::Database database;
  auto negotiation =
      std::make_shared<rmf_traffic::schedule::Negotiation>(
        database, std::vector<rmf_traffic::schedule::ParticipantId>{0, 2});

  std::vector<Table> submitted =
  {
    {{}, 2}, {{}, 0}, {{0}, 2}, {{2}, 0}
  };

  apply_submissions(negotiation, submitted);

  CHECK(negotiation->complete());
  CHECK(negotiation->ready());

  negotiation->add_participant(1);

  CHECK_FALSE(negotiation->complete());
  CHECK_FALSE(negotiation->ready());

  submitted =
  {
    {{}, 1}, {{2}, 1}
  };

  apply_submissions(negotiation, submitted);

  CHECK_FALSE(negotiation->table(1, {2})->respond(1));

  std::vector<Table> rejected =
  {
    {{2}, 1}, {{2}, 0}, {{}, 1}, {{2}, 1}, {{0, 2}, 1}, {{0}, 1}
  };

  apply_rejection(negotiation, rejected);

  CHECK(negotiation->complete());
}
