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

#ifndef RMF_RXCPP__TRANSPORT_HPP
#define RMF_RXCPP__TRANSPORT_HPP

#include <rmf_rxcpp/detail/TransportDetail.hpp>
#include <rmf_rxcpp/RxJobs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rxcpp/rx.hpp>
#include <utility>
#include <optional>

namespace rmf_rxcpp {

class RxCppExecutor: 
  public rclcpp::Executor
{
public:
  RxCppExecutor(
    rxcpp::schedulers::worker worker,
    const rclcpp::ExecutorOptions& options = rclcpp::ExecutorOptions())
    : 
    rclcpp::Executor{options}, 
    _worker{std::move(worker)},
    _stopping{false}
  {

  }
  virtual ~RxCppExecutor() {};

  void stop()
  {
    _stopping = true;
  }
  void spin() override
  {
    _schedule_spin();
  }
private:
  rxcpp::schedulers::worker _worker;
  bool _stopping;

  void _do_spin()
  {
    spin_some();
    wait_for_work();
    
    if(rclcpp::ok(this->context_) && !_stopping)
      _schedule_spin();
  }

  void _schedule_spin()
  {
    _worker.schedule(
          [w = this](const auto&)
    {
      if (w->weak_nodes_.size() == 0)
      {
        w->_do_spin();
        return;
      }
      //Assume at most 1 node. OK for our use case. 
      else if (const auto lock = w->weak_nodes_.back().lock())
        w->_do_spin();
    });
  }

};

// TODO(MXG): We define all the member functions of this class inline so that we
// don't need to export/install rmf_rxcpp as its own shared library (linking to
// it as a static library results in linking errors related to symbols not being
// relocatable). This is a quick and dirty solution to the linking problem, so
// we may want to consider replacing it with something cleaner.
class Transport : public rclcpp::Node
{
public:

  explicit Transport(
      rxcpp::schedulers::worker worker,
      const std::string& node_name,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : rclcpp::Node{node_name, options},
      _executor(worker, _make_exec_args(options))
  {
    // Do nothing
  }

  /**
   * Not threadsafe
   */
  void start()
  {
    if (!_stopping)
      return;

    if (!_node_added)
      _executor.add_node(shared_from_this());

    _stopping = false;
    _executor.spin();
  }

  void stop()
  {
    _stopping = true;
    _executor.stop();
  }

  std::condition_variable& spin_cv()
  {
    return _spin_cv;
  }

  bool still_spinning() const
  {
    return !_stopping && rclcpp::ok(get_node_options().context());
  }

  /**
   * Creates a sharable observable that is bridged to a rclcpp subscription and is observed on an
   * event loop. When there are multiple subscribers, it multiplexes the message onto each
   * subscriber.
   * @tparam Message
   * @param topic_name
   * @param qos
   * @return
   */
  template<typename Message>
  auto create_observable(const std::string& topic_name, const rclcpp::QoS& qos)
  {
    auto wrapper = std::make_shared<detail::SubscriptionWrapper<Message>>(
      shared_from_this(), topic_name, qos);
    return rxcpp::observable<>::create<typename Message::SharedPtr>([wrapper](const auto& s)
    {
      (*wrapper)(s);
    }).publish().ref_count().observe_on(rxcpp::observe_on_event_loop());
  }

private:
  bool _stopping = true;
  RxCppExecutor _executor;
  bool _node_added = false;
  std::condition_variable _spin_cv;

  static rclcpp::ExecutorOptions _make_exec_args(
      const rclcpp::NodeOptions& options)
  {
    rclcpp::ExecutorOptions exec_args;
    exec_args.context = options.context();
    return exec_args;
  }
};

} // namespace rmf_rxcpp

#endif //RMF_RXCPP__TRANSPORT_HPP
