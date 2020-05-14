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

namespace {
//==============================================================================
struct MockNegotiator : public rmf_traffic::schedule::Negotiator
{
  enum Choice {
    Submit,
    Reject,
    Forfeit
  };

  virtual void respond(
    const rmf_traffic::schedule::Negotiation::Table::ViewerPtr&,
    const Responder& responder,
    const bool* = nullptr) final
  {
    if (Submit == _choice)
      responder.submit({});
    else if (Reject == _choice)
      responder.reject({});
    else
      responder.forfeit({});
  }

  virtual void respond(
    const rmf_traffic::schedule::Negotiation::TablePtr table)
  {
    using rmf_traffic::schedule::SimpleResponder;
    respond(table->viewer(), SimpleResponder(table));
  }

  MockNegotiator& submit()
  {
    _choice = Submit;
    return *this;
  }

  MockNegotiator& reject()
  {
    _choice = Reject;
    return *this;
  }

  MockNegotiator& forfeit()
  {
    _choice = Forfeit;
    return *this;
  }

  Choice _choice = Submit;
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
    const auto& table_ptr = negotiation->table(
          table.for_participant, table.to_accommodate);

    MockNegotiator().submit().respond(
          table_ptr->viewer(), Responder(table_ptr));
  }
}

//==============================================================================
void apply_forfeit(
  const std::shared_ptr<rmf_traffic::schedule::Negotiation>& negotiation,
  const std::vector<Table>& forfeited)
{
  using Responder = rmf_traffic::schedule::SimpleResponder;
  for (const auto& table : forfeited)
  {
    const auto& table_ptr = negotiation->table(
          table.for_participant, table.to_accommodate);

    MockNegotiator().forfeit().respond(
          table_ptr->viewer(), Responder(table_ptr));
  }
}

} // anonymous namespace

//==============================================================================
SCENARIO("Identify a failed negotiation")
{
  // This situation emerged during testing and revealed some bugs that can occur
  // while adding participants.
  auto database = std::make_shared<rmf_traffic::schedule::Database>();

  // The Negotiation requires these participants to have descriptions
  auto description =
      rmf_traffic::schedule::ParticipantDescription(
        "", "", rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
        rmf_traffic::Profile(nullptr));
  database->register_participant(description);
  database->register_participant(description);
  database->register_participant(description);

  auto negotiation = rmf_traffic::schedule::Negotiation::make_shared(
    database, std::vector<rmf_traffic::schedule::ParticipantId>{0, 2});

  SECTION("Case 1")
  {
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

    std::vector<Table> forfeited =
    {
      {{2}, 1}, {{2}, 0}, {{}, 1}, {{2}, 1}, {{0, 2}, 1}, {{0}, 1}
    };

    apply_forfeit(negotiation, forfeited);
  }

  SECTION("Case 2")
  {
    negotiation->add_participant(1);

    MockNegotiator().submit().respond(negotiation->table({0}));
    MockNegotiator().submit().respond(negotiation->table({1}));
    MockNegotiator().submit().respond(negotiation->table({2}));

    MockNegotiator().reject().respond(negotiation->table({1}));
    MockNegotiator().reject().respond(negotiation->table({2}));

    MockNegotiator().submit().respond(negotiation->table({0, 1}));
    MockNegotiator().reject().respond(negotiation->table({0, 1}));
    MockNegotiator().reject().respond(negotiation->table({0}));

    MockNegotiator().submit().respond(negotiation->table({0}));

    MockNegotiator().submit().respond(negotiation->table({1}));
    MockNegotiator().submit().respond(negotiation->table({1, 2}));
    MockNegotiator().submit().respond(negotiation->table({1, 0}));

    MockNegotiator().reject().respond(negotiation->table({1, 2}));

    MockNegotiator().forfeit().respond(negotiation->table({2}));
    MockNegotiator().forfeit().respond(negotiation->table({1}));
    MockNegotiator().forfeit().respond(negotiation->table({0}));
  }

  CHECK(negotiation->complete());
}

//==============================================================================
SCENARIO("Submit after a rejection")
{
  class MockResponder : public rmf_traffic::schedule::Negotiator::Responder
  {
  public:

    rmf_traffic::schedule::Negotiation::TablePtr table;
    bool* accepted;
    rmf_utils::optional<rmf_traffic::schedule::Version>* version;

    MockResponder(
      rmf_traffic::schedule::Negotiation::TablePtr table_,
      bool* accepted_,
      rmf_utils::optional<rmf_traffic::schedule::Version>* version_)
    : table(table_),
      accepted(accepted_),
      version(version_)
    {
      // Do nothing
    }

    void submit(
      std::vector<rmf_traffic::Route> itinerary,
      std::function<UpdateVersion()>) const final
    {
      *version = table->version() + 1;
      *accepted = table->submit(std::move(itinerary), **version);
    }

    void reject(const Alternatives& alternatives) const final
    {
      const auto parent = table->parent();
      if (parent)
      {
        parent->reject(
              parent->version(),
              table->sequence().back().participant,
              alternatives);
      }
    }

    void forfeit(const std::vector<ParticipantId>& /*blockers*/) const final
    {
      table->forfeit(table->version());
    }
  };

  auto database = std::make_shared<rmf_traffic::schedule::Database>();

  // The Negotiation requires these participants to have descriptions
  auto description =
      rmf_traffic::schedule::ParticipantDescription(
        "", "", rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
        rmf_traffic::Profile(nullptr));
  database->register_participant(description);
  database->register_participant(description);

  auto negotiation = rmf_traffic::schedule::Negotiation::make_shared(
    database, std::vector<rmf_traffic::schedule::ParticipantId>{0, 1});

  bool accepted = false;
  rmf_utils::optional<rmf_traffic::schedule::Version> version;

  MockResponder responder(negotiation->table(0, {}), &accepted, &version);
  MockNegotiator().submit().respond(responder.table->viewer(), responder);

  CHECK(accepted);
  REQUIRE(version);
  CHECK(*version == 1);
  const auto last_version = *version;

  accepted = false;
  version = rmf_utils::nullopt;
  MockResponder child_responder(negotiation->table(1, {0}), &accepted,
    &version);
  MockNegotiator().forfeit().respond(
        child_responder.table->viewer(), child_responder);

  CHECK_FALSE(version);
  CHECK_FALSE(accepted);

  accepted = false;
  version = rmf_utils::nullopt;
  MockResponder reresponder(responder.table, &accepted, &version);
  MockNegotiator().submit().respond(reresponder.table->viewer(), reresponder);

  CHECK(accepted);
  REQUIRE(version);
  CHECK(last_version < *version);
  CHECK(*version == 2);
}
