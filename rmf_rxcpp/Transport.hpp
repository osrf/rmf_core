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

#include "RxJobs.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rxcpp/rx.hpp>
#include <utility>

class Transport : public rclcpp::Node
{
private:

  template<typename Message>
  class EventJob
  {
  public:

    EventJob(rclcpp::Node::SharedPtr  node, std::string topic_name, const rclcpp::QoS& qos)
    : _node{std::move(node)}, _topic_name{std::move(topic_name)}, _qos{qos}
    {
      // no op
    }

    template<typename Subscriber>
    void operator()(const Subscriber& s)
    {
      _subscription = _node->create_subscription<Message>(_topic_name, _qos, [s](typename Message::SharedPtr msg)
      {
        s.on_next(msg);
      });
    }
  private:

    rclcpp::Node::SharedPtr _node;
    std::string _topic_name;
    rclcpp::QoS _qos;
    typename rclcpp::Subscription<Message>::SharedPtr _subscription;
  };
public:

  explicit Transport(const std::string& node_name) : rclcpp::Node{node_name}
  {
    // no op
  }

  template<typename Message>
  auto create_event_job(const std::string& topic_name, const rclcpp::QoS& qos)
  {
    return std::make_shared<EventJob<Message>>(shared_from_this(), topic_name, qos);
  }

  void spin_background()
  {
    _spin_thread = std::thread{[this]() { _do_spin(); }};
  }

private:

  std::thread _spin_thread;

  void _do_spin()
  {
    rclcpp::spin(shared_from_this());
  }
};


#endif //RMF_RXCPP__TRANSPORT_HPP
