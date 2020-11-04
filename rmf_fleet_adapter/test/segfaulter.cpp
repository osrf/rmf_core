#include "phases/MockAdapterFixture.hpp"
#include "phases/DoorOpen.hpp"
#include "rmf_fleet_adapter/StandardNames.hpp"
#include <iostream>

#include <rcl/rcl.h>

namespace rmf_fleet_adapter {
namespace phases {
namespace test {

using rmf_door_msgs::msg::DoorRequest;
using rmf_door_msgs::msg::DoorMode;
using rmf_door_msgs::msg::DoorState;
using rmf_door_msgs::msg::SupervisorHeartbeat;
using rmf_door_msgs::msg::DoorSessions;
using rmf_door_msgs::msg::Session;

class SegFaulter : public MockAdapterFixture {
public:
  SegFaulter() {}

  void Run() {
    sub = adapter->node()->create_subscription<DoorRequest>(
      "adapter_door_requests",
      10,
      [&](DoorRequest::UniquePtr door_request)
      {
        std::unique_lock<std::mutex> lk(m);
        received_requests.emplace_back(std::move(door_request));
        received_requests_cv.notify_all();
      });

    info = add_robot();

    door_name = "test_door";
    request_id = "test_id";
    pending_phase = std::make_shared<DoorOpen::PendingPhase>(
      info.context,
      door_name,
      request_id,
      rmf_traffic::Time()
    );
    active_phase = pending_phase->begin();

    when_it_is_cancelled_before_its_started();
    when_it_is_started();
  }

private:
  std::shared_ptr<DoorOpen::PendingPhase> pending_phase;
  std::shared_ptr<Task::ActivePhase> active_phase;
  struct MockAdapterFixture::RobotInfo info;
  rclcpp::Subscription<rmf_door_msgs::msg::DoorRequest>::SharedPtr sub;
  rclcpp::Publisher<DoorState>::SharedPtr door_state_pub;
  rclcpp::Publisher<SupervisorHeartbeat>::SharedPtr heartbeat_pub;
  std::string door_name;
  std::string request_id;

  std::mutex m;
  std::condition_variable received_requests_cv;
  std::list<DoorRequest::UniquePtr> received_requests;

  void when_it_is_cancelled_before_its_started()
  {
    active_phase->cancel();

    {
      bool received_open = false;
      rxcpp::composite_subscription rx_sub;
      auto subscription = adapter->node()->create_subscription<DoorRequest>(
        "adapter_door_requests",
        10,
        [&](DoorRequest::UniquePtr door_request)
        {
          if (door_request->requested_mode.value == DoorMode::MODE_OPEN)
            received_open = true;
          else if (door_request->requested_mode.value == DoorMode::MODE_CLOSED)
            rx_sub.unsubscribe();
        });
      auto obs = active_phase->observe();
      obs.as_blocking().subscribe(rx_sub);
      assert(!received_open);
    }
  }

  void when_it_is_started()
  {
    std::condition_variable status_updates_cv;
    std::list<Task::StatusMsg> status_updates;
    auto sub = active_phase->observe().subscribe(
      [&](const auto& status)
      {
        std::unique_lock<std::mutex> lk(m);
        status_updates.emplace_back(status);
        status_updates_cv.notify_all();
      });

    {
      std::unique_lock<std::mutex> lk(m);
      if (received_requests.empty())
        received_requests_cv.wait(lk, [&]() { return !received_requests.empty(); });
    }

    {
      std::unique_lock<std::mutex> lk(m);
      received_requests_cv.wait(lk, [&]() { return received_requests.size() >= 3; });
    }

    door_state_pub = adapter->node()->create_publisher<DoorState>(DoorStateTopicName, 10);
    heartbeat_pub = adapter->node()->create_publisher<SupervisorHeartbeat>(
      DoorSupervisorHeartbeatTopicName, 10);


    and_when_door_state_is_open_and_supervisor_has_session(status_updates_cv, status_updates);
    and_when_door_state_is_open_and_supervisor_do_not_have_session(
      status_updates_cv,
      status_updates);
    and_when_door_state_is_closed_and_supervisor_has_session(status_updates_cv, status_updates);
    and_when_it_is_cancelled();

    sub.unsubscribe();
  }

  void publish_door_state(uint32_t mode) {
    DoorState door_state;
    door_state.door_name = door_name;
    door_state.door_time = adapter->node()->now();
    door_state.current_mode.value = mode;
    door_state_pub->publish(door_state);
  };

  void publish_heartbeat_with_session() {
    Session session;
    session.requester_id = request_id;
    DoorSessions door_sessions;
    door_sessions.door_name = door_name;
    door_sessions.sessions.emplace_back(std::move(session));
    SupervisorHeartbeat heartbeat;
    heartbeat.all_sessions.emplace_back(std::move(door_sessions));
    heartbeat_pub->publish(heartbeat);
  };

  void publish_empty_heartbeat() {
    heartbeat_pub->publish(SupervisorHeartbeat());
  };

  void and_when_door_state_is_open_and_supervisor_has_session(
    std::condition_variable& status_updates_cv,
    std::list<Task::StatusMsg>& status_updates)
  {
    auto sub2 = rxcpp::observable<>::interval(std::chrono::milliseconds(100))
      .subscribe_on(rxcpp::observe_on_new_thread())
      .subscribe([&](const auto&)
      {
        publish_door_state(DoorMode::MODE_OPEN);
        publish_heartbeat_with_session();
      });

    {
      std::unique_lock<std::mutex> lk(m);
      bool completed = status_updates_cv.wait_for(lk, std::chrono::milliseconds(1000), [&]()
      {
        return status_updates.back().state == Task::StatusMsg::STATE_COMPLETED;
      });
    }

    sub2.unsubscribe();
  }

  void and_when_door_state_is_open_and_supervisor_do_not_have_session(
    std::condition_variable& status_updates_cv,
    std::list<Task::StatusMsg>& status_updates)
  {
    auto sub2 = rxcpp::observable<>::interval(std::chrono::milliseconds(100))
      .subscribe_on(rxcpp::observe_on_new_thread())
      .subscribe([&](const auto&)
      {
        publish_door_state(DoorMode::MODE_OPEN);
        publish_empty_heartbeat();
      });

    {
      std::unique_lock<std::mutex> lk(m);
      bool completed = status_updates_cv.wait_for(lk, std::chrono::milliseconds(1000), [&]()
      {
        return status_updates.back().state == Task::StatusMsg::STATE_COMPLETED;
      });
    }

    sub2.unsubscribe();
  }

  void and_when_door_state_is_closed_and_supervisor_has_session(
    std::condition_variable& status_updates_cv,
    std::list<Task::StatusMsg>& status_updates)
  {
    auto sub2 = rxcpp::observable<>::interval(std::chrono::milliseconds(100))
      .subscribe_on(rxcpp::observe_on_new_thread())
      .subscribe([&](const auto&)
      {
        publish_door_state(DoorMode::MODE_CLOSED);
        publish_empty_heartbeat();
      });

    {
      std::unique_lock<std::mutex> lk(m);
      bool completed = status_updates_cv.wait_for(lk, std::chrono::milliseconds(1000), [&]()
      {
        return status_updates.back().state == Task::StatusMsg::STATE_COMPLETED;
      });
    }

    sub2.unsubscribe();
  }

  void and_when_it_is_cancelled()
  {
    {
      std::unique_lock<std::mutex> lk(m);
      received_requests_cv.wait(lk, [&]()
      {
        return !received_requests.empty();
      });
      active_phase->cancel();
    }

    {
      std::unique_lock<std::mutex> lk(m);
      received_requests_cv.wait(lk, [&]()
      {
        return received_requests.back()->requested_mode.value == DoorMode::MODE_CLOSED;
      });
    }
  }
};

}  // namespace test
}  // namespace phases
}  // namespace rmf_fleet_adapter

int main(int argc, char ** argv) {
  for (int ii = 0; ii < 1000; ++ii) {
    rmf_fleet_adapter::phases::test::SegFaulter segfaulter;
    std::cout << ii << std::endl;
    segfaulter.Run();
  }
  return 0;
}
