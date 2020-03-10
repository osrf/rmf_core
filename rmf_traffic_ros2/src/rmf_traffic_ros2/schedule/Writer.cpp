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

#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantDescription.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rmf_traffic_msgs/msg/itinerary_set.hpp>
#include <rmf_traffic_msgs/msg/itinerary_extend.hpp>
#include <rmf_traffic_msgs/msg/itinerary_delay.hpp>
#include <rmf_traffic_msgs/msg/itinerary_erase.hpp>
#include <rmf_traffic_msgs/msg/itinerary_clear.hpp>

#include <rmf_traffic_msgs/msg/schedule_inconsistency.hpp>

#include <rmf_traffic_msgs/srv/register_participant.hpp>
#include <rmf_traffic_msgs/srv/unregister_participant.hpp>

namespace rmf_traffic_ros2 {
namespace schedule {

namespace {
//==============================================================================
class RectifierFactory
    : public rmf_traffic::schedule::RectificationRequesterFactory
{
public:


  struct RectifierStub;

  class Requester : public rmf_traffic::schedule::RectificationRequester
  {
  public:

    rmf_traffic::schedule::Rectifier rectifier;
    std::shared_ptr<RectifierStub> stub;

    Requester(rmf_traffic::schedule::Rectifier rectifier_);

  };

  struct RectifierStub
  {
    Requester& requester;
  };

  using StubMap = std::unordered_map<
      rmf_traffic::schedule::ParticipantId,
      std::weak_ptr<RectifierStub>
  >;

  StubMap stub_map;

  using InconsistencyMsg = rmf_traffic_msgs::msg::ScheduleInconsistency;
  rclcpp::Subscription<InconsistencyMsg>::SharedPtr inconsistency_sub;

  RectifierFactory(rclcpp::Node& node)
  {
    inconsistency_sub = node.create_subscription<InconsistencyMsg>(
          ScheduleInconsistencyTopicName,
          rclcpp::SystemDefaultsQoS().reliable(),
          [&](const InconsistencyMsg::UniquePtr msg)
    {
      check_inconsistencies(*msg);
    });
  }

  std::unique_ptr<rmf_traffic::schedule::RectificationRequester> make(
      rmf_traffic::schedule::Rectifier rectifier,
      rmf_traffic::schedule::ParticipantId participant_id) final
  {
    auto requester = std::make_unique<Requester>(std::move(rectifier));

    // It's okay to just override any entry that might have been in here before,
    // because the Database should never double-assign a ParticipantId
    stub_map[participant_id] = requester->stub;

    return std::move(requester);
  }

  void check_inconsistencies(const InconsistencyMsg& msg)
  {
    if (msg.ranges.empty())
    {
      // This shouldn't generally happen, since empty ranges should not get
      // published, but we'll check here anyway.
      return;
    }

    const auto it = stub_map.find(msg.participant);
    if (it == stub_map.end())
      return;

    const auto& stub = it->second.lock();
    if (!stub)
    {
      // This participant has expired, so we should remove it from the map
      stub_map.erase(it);
      return;
    }

    using Range = rmf_traffic::schedule::Rectifier::Range;
    std::vector<Range> ranges;
    ranges.reserve(msg.ranges.size());
    for (const auto& r : msg.ranges)
      ranges.emplace_back(Range{r.lower, r.upper});

    stub->requester.rectifier.retransmit(ranges, msg.last_known_version);
  }
};

//==============================================================================
RectifierFactory::Requester::Requester(
    rmf_traffic::schedule::Rectifier rectifier_)
  : rectifier(std::move(rectifier_)),
    stub(std::make_shared<RectifierStub>(RectifierStub{*this}))
{
  // Do nothing
}

} // anonymous namespace

//==============================================================================
class Writer::Implementation
    : public rmf_traffic::schedule::Writer
{
public:

  RectifierFactory rectifier_factory;

  using Set = rmf_traffic_msgs::msg::ItinerarySet;
  using Extend = rmf_traffic_msgs::msg::ItineraryExtend;
  using Delay = rmf_traffic_msgs::msg::ItineraryDelay;
  using Erase = rmf_traffic_msgs::msg::ItineraryErase;
  using Clear = rmf_traffic_msgs::msg::ItineraryClear;

  rclcpp::Publisher<Set>::SharedPtr set_pub;
  rclcpp::Publisher<Extend>::SharedPtr extend_pub;
  rclcpp::Publisher<Delay>::SharedPtr delay_pub;
  rclcpp::Publisher<Erase>::SharedPtr erase_pub;
  rclcpp::Publisher<Clear>::SharedPtr clear_pub;

  using Register = rmf_traffic_msgs::srv::RegisterParticipant;
  using Unregister = rmf_traffic_msgs::srv::UnregisterParticipant;

  rclcpp::Client<Register>::SharedPtr register_client;
  rclcpp::Client<Unregister>::SharedPtr unregister_client;

  Implementation(rclcpp::Node& node)
    : rectifier_factory(node)
  {
    set_pub = node.create_publisher<Set>(
          ItinerarySetTopicName,
          rclcpp::SystemDefaultsQoS().best_effort());

    extend_pub = node.create_publisher<Extend>(
          ItineraryExtendTopicName,
          rclcpp::SystemDefaultsQoS().best_effort());

    delay_pub = node.create_publisher<Delay>(
          ItineraryDelayTopicName,
          rclcpp::SystemDefaultsQoS().best_effort());

    erase_pub = node.create_publisher<Erase>(
          ItineraryEraseTopicName,
          rclcpp::SystemDefaultsQoS().best_effort());

    clear_pub = node.create_publisher<Clear>(
          ItineraryClearTopicName,
          rclcpp::SystemDefaultsQoS().best_effort());

    register_client =
        node.create_client<Register>(RegisterParticipantSrvName);

    unregister_client =
        node.create_client<Unregister>(UnregisterParticipantSrvName);
  }

  void set(
      const rmf_traffic::schedule::ParticipantId participant,
      const Input& itinerary,
      const rmf_traffic::schedule::ItineraryVersion version) final
  {
    Set msg;
    msg.participant = participant;
    msg.itinerary = convert(itinerary);
    msg.itinerary_version = version;

    set_pub->publish(std::move(msg));
  }

  void extend(
      const rmf_traffic::schedule::ParticipantId participant,
      const Input& routes,
      const rmf_traffic::schedule::ItineraryVersion version) final
  {
    Extend msg;
    msg.participant = participant;
    msg.routes = convert(routes);
    msg.itinerary_version = version;

    extend_pub->publish(std::move(msg));
  }

  void delay(
      const rmf_traffic::schedule::ParticipantId participant,
      const rmf_traffic::Time from,
      const rmf_traffic::Duration duration,
      const rmf_traffic::schedule::ItineraryVersion version) final
  {
    Delay msg;
    msg.participant = participant;
    msg.from_time = from.time_since_epoch().count();
    msg.delay = duration.count();
    msg.itinerary_version = version;

    delay_pub->publish(std::move(msg));
  }

  void erase(
      const rmf_traffic::schedule::ParticipantId participant,
      const std::vector<rmf_traffic::RouteId>& routes,
      const rmf_traffic::schedule::ItineraryVersion version) final
  {
    Erase msg;
    msg.participant = participant;
    msg.routes = routes;
    msg.itinerary_version = version;

    erase_pub->publish(std::move(msg));
  }

  void erase(
      const rmf_traffic::schedule::ParticipantId participant,
      const rmf_traffic::schedule::ItineraryVersion version) final
  {
    Clear msg;
    msg.participant = participant;
    msg.itinerary_version = version;

    clear_pub->publish(std::move(msg));
  }

  rmf_traffic::schedule::ParticipantId register_participant(
      rmf_traffic::schedule::ParticipantDescription participant_info) final
  {
    using namespace std::chrono_literals;

    auto request = std::make_shared<Register::Request>();
    request->description = convert(participant_info);

    auto future = register_client->async_send_request(request);
    while(future.wait_for(100ms) != std::future_status::ready)
    {
      if (!rclcpp::ok())
      {
        throw std::runtime_error(
              "[rmf_traffic_ros2::schedule::Writer] Tearing down while waiting "
              "for a schedule participant to finish registering");
      }
    }

    const auto response = future.get();
    if (!response->error.empty())
    {
      throw std::runtime_error(
            "[rmf_traffic_ros2::schedule::Writer] Error while attempting to "
            "register a participant: " + response->error);
    }

    return response->participant_id;
  }

  void unregister_participant(
      const rmf_traffic::schedule::ParticipantId participant) final
  {
    auto request = std::make_shared<Unregister::Request>();
    request->participant_id = participant;

    unregister_client->async_send_request(
          request,
          [=](const rclcpp::Client<Unregister>::SharedFuture response_future)
    {
      const auto response = response_future.get();
      if (!response->error.empty())
      {
        throw std::runtime_error(
              "[rmf_traffic_ros2::schedule::Writer] Error while attempting to "
              "unregister a participant: " + response->error);
      }
    });
  }

  std::future<rmf_traffic::schedule::Participant> make_participant(
      rmf_traffic::schedule::ParticipantDescription description)
  {
    // TODO(MXG): This implementation assumes that the async promise will be
    // finished before the Writer instance is destructed. If that is not true,
    // then we could get undefined behavior from this implementation. However,
    // the Writer should only get destructed during the teardown of the whole
    // Node, which implies that the program is exiting.
    //
    // This shouldn't be a major concern, but it may be worth revisiting whether
    // a cleaner approach is possible.

    std::promise<rmf_traffic::schedule::Participant> promise;
    auto future = promise.get_future();
    std::thread worker(
          [this](
          rmf_traffic::schedule::ParticipantDescription description,
          std::promise<rmf_traffic::schedule::Participant> promise)
    {
      promise.set_value(rmf_traffic::schedule::make_participant(
            std::move(description), *this, &rectifier_factory));
    }, std::move(description), std::move(promise));

    worker.detach();

    return future;
  }

  void async_make_participant(
      rmf_traffic::schedule::ParticipantDescription description,
      std::function<void(rmf_traffic::schedule::Participant)> ready_callback)
  {
    std::thread worker(
          [description = std::move(description),
           this,
           ready_callback = std::move(ready_callback)]()
    {
      // TODO(MXG): We could probably make an implementation of the
      // RectifierFactory that allows us to pass the ready_callback along to
      // the service call so that it gets triggered when the service response is
      // received. That way we don't need to create an additional thread here
      // and worry about the threat of race conditions.
      auto participant = rmf_traffic::schedule::make_participant(
            std::move(description), *this, &rectifier_factory);

      if (ready_callback)
        ready_callback(std::move(participant));
    });

    worker.detach();
  }

};

//==============================================================================
std::shared_ptr<Writer> Writer::make(rclcpp::Node& node)
{
  return std::shared_ptr<Writer>(new Writer(node));
}

//==============================================================================
bool Writer::ready() const
{
  return _pimpl->register_client->service_is_ready()
      && _pimpl->unregister_client->service_is_ready();
}

//==============================================================================
void Writer::wait_for_service() const
{
  _pimpl->register_client->wait_for_service();
  _pimpl->unregister_client->wait_for_service();
}

//==============================================================================
bool Writer::wait_for_service(rmf_traffic::Time stop) const
{
  bool ready = true;

  ready &= _pimpl->register_client->wait_for_service(
        stop - std::chrono::steady_clock::now());

  ready &= _pimpl->unregister_client->wait_for_service(
        stop - std::chrono::steady_clock::now());

  return ready;
}

//==============================================================================
std::future<rmf_traffic::schedule::Participant> Writer::make_participant(
    rmf_traffic::schedule::ParticipantDescription description)
{
  return _pimpl->make_participant(std::move(description));
}

//==============================================================================
void Writer::async_make_participant(
    rmf_traffic::schedule::ParticipantDescription description,
    std::function<void(rmf_traffic::schedule::Participant)> ready_callback)
{
  _pimpl->async_make_participant(
        std::move(description), std::move(ready_callback));
}

//==============================================================================
Writer::Writer(rclcpp::Node& node)
  : _pimpl(rmf_utils::make_unique_impl<Implementation>(node))
{
  // Do nothing
}

} // namespace schedule
} // namespace rmf_traffic_ros2
