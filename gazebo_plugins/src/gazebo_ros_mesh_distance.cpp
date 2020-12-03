// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gazebo/transport/TransportIface.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_plugins/gazebo_ros_mesh_distance.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace gazebo_plugins
{
using std::placeholders::_1;

/// Class to hold private data members (PIMPL pattern)
class GazeboRosModelDistancePrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  std::string own_ns_;

  rclcpp::TimerBase::SharedPtr posPubTimer_;
  rclcpp::TimerBase::SharedPtr distPubTimer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisherPos_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisherDist_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> sub_list_;
  std::vector<std::string> pos_topics_;
  std::map<std::string, double> distances_;

  void posPubTimer_callback();
  void distPubTimer_callback();
  bool compareTopics(std::vector<std::string>& v1, std::vector<std::string>& v2);
  void topic_callback(const std_msgs::msg::String::SharedPtr msg);
  double calculateDistance(const double x, const double y, const double z) const;
};

GazeboRosModelDistance::GazeboRosModelDistance()
: impl_(std::make_unique<GazeboRosModelDistancePrivate>())
{
}

GazeboRosModelDistance::~GazeboRosModelDistance()
{
}

void GazeboRosModelDistance::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
  impl_->model_ = model;
  impl_->world_ = model->GetWorld();

  if (!sdf->HasElement("namespace")) {
    impl_->own_ns_ = "default";
  } else {
    impl_->own_ns_ = sdf->GetElement("namespace")->Get<std::string>();
  }

  // The model pointer gives you direct access to the physics object,
  // for example:
  RCLCPP_INFO(impl_->ros_node_->get_logger(), model->GetName().c_str());

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.

  impl_->posPubTimer_ = impl_->ros_node_->create_wall_timer(
      330ms, std::bind(&GazeboRosModelDistancePrivate::posPubTimer_callback, impl_.get()));
  impl_->distPubTimer_ = impl_->ros_node_->create_wall_timer(
      1000ms, std::bind(&GazeboRosModelDistancePrivate::distPubTimer_callback, impl_.get()));

  impl_->publisherPos_ = impl_->ros_node_->create_publisher<std_msgs::msg::String>("SimPosition", 3);
  impl_->publisherDist_ = impl_->ros_node_->create_publisher<std_msgs::msg::String>("SimDistances", 3);

}

void GazeboRosModelDistancePrivate::posPubTimer_callback()
{
  ignition::math::Pose3d pose = model_->WorldPose();
  auto message = std_msgs::msg::String();

  // Publish own position
  message.data = own_ns_;
  message.data += " ";
  message.data += std::to_string(pose.X()) + " ";
  message.data += std::to_string(pose.Y()) + " ";
  message.data += std::to_string(pose.Z());
  publisherPos_->publish(message);

  // Publish distances to other drones
  message.data = "";
  for(auto dist : distances_)
  {
    message.data += dist.first;
    message.data += "=";
    message.data += std::to_string(dist.second);
    message.data += " ";
  }
  if (message.data.length())
    publisherDist_->publish(message);
}

void GazeboRosModelDistancePrivate::distPubTimer_callback()
{
  // update other drones positions
  std::vector<std::string> available_pos_topics;
  std::map<std::string, std::vector<std::string>> all_topics = ros_node_->get_topic_names_and_types();
  for (std::map<std::string, std::vector<std::string>>::iterator it = all_topics.begin(); it != all_topics.end(); it++)
  {
    auto tp = it->first;
    if (tp.find("SimPosition") != std::string::npos && tp.find(own_ns_) == std::string::npos) {
      available_pos_topics.push_back(tp);
      //RCLCPP_INFO(ros_node_->get_logger(), "PosTopic: %s", tp.c_str());
    }
  }
  if (! compareTopics(pos_topics_, available_pos_topics))
  {
    // Topic list changed => need to update subscriptions
    distances_.clear();
    for(auto sub : sub_list_)
    {
      sub.reset();
    }
    sub_list_.clear();
    pos_topics_ = available_pos_topics;

    for(auto tp : pos_topics_)
    {
      auto sub = ros_node_->create_subscription<std_msgs::msg::String>(
        tp, 10, std::bind(&GazeboRosModelDistancePrivate::topic_callback, this, _1));
      sub_list_.push_back(sub);
    }
  }
}

bool GazeboRosModelDistancePrivate::compareTopics(std::vector<std::string>& v1, std::vector<std::string>& v2)
{
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    return v1 == v2;
}

void GazeboRosModelDistancePrivate::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::stringstream ss;
  std::string name;
  double x, y, z;

  ss << msg->data;
  ss >> name >> x >> y >> z;
  distances_[name] = calculateDistance(x,y,z);

}

double GazeboRosModelDistancePrivate::calculateDistance(const double x, const double y, const double z) const
{
  const ignition::math::Vector3d pos = model_->WorldPose().Pos();
  const ignition::math::Vector3d pos2(x, y, z);
  return pos.Distance(pos2);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosModelDistance)
}  // namespace gazebo_plugins
