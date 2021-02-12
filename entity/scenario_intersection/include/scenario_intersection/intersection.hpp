// Copyright 2020 Tier IV, Inc.
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

#ifndef SCENARIO_INTERSECTION__INTERSECTION_HPP_
#define SCENARIO_INTERSECTION__INTERSECTION_HPP_

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "boost/lexical_cast.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/logger.hpp"

#include "scenario_api/scenario_api_core.hpp"
#include "scenario_intersection/arrow.hpp"
#include "scenario_intersection/color.hpp"
#include "scenario_intersection/utility.hpp"
#include "scenario_utility/scenario_utility.hpp"

namespace scenario_intersection
{

class Intersection
{
  class Controller
  {
    class Transition
    {
      const int target_;

      const Color color_;
      std::vector<Arrow> arrows_;

public:
      Transition()
      : target_{-1},
        color_{Color::Blank}
      {}

      Transition(
        const YAML::Node & target,
        const YAML::Node & color,
        const YAML::Node & arrows)
      : target_{target.as<int>()},
        color_{color ? convert<Color>(color.as<std::string>()) : Color::Blank}
      {
        if (arrows && !arrows.IsNull()) {
          if (arrows.IsScalar()) {  // NOTE: deperecated behavior
            const auto value {convert<Arrow>(arrows.as<std::string>())};

            if (value != Arrow::Blank) {
              arrows_.emplace_back(value);
            }
          } else {
            for (const auto & each : arrows) {
              const auto value {convert<Arrow>(each.as<std::string>())};

              if (value != Arrow::Blank) {
                arrows_.emplace_back(value);
              }
            }
          }
        }
      }

      bool changeColor(ScenarioAPI & simulator) const
      {
        if (target_ < 0 || color_ == Color::Blank) {
          return simulator.resetTrafficLightColor(target_, false);
        } else {  // NOTE: Maybe specified illiegal traffic-light-id.
          return simulator.setTrafficLightColor(
            target_, boost::lexical_cast<std::string>(
              color_), false);
        }
      }

      bool changeArrow(ScenarioAPI & simulator) const
      {
        simulator.resetTrafficLightArrow(target_, false);

        if (0 <= target_) {
          return
            std::all_of(
            std::begin(arrows_), std::end(arrows_),
            [&](const auto & each)
            {
              return
              simulator.setTrafficLightArrow(
                target_, boost::lexical_cast<std::string>(each), false);
            });
        } else {
          return false;
        }
      }

      bool operator()(ScenarioAPI & simulator) const
      {
        return changeColor(simulator) && changeArrow(simulator);
      }
    };

    std::vector<Transition> transitions_;

public:
    Controller()
    {
      transitions_.emplace_back();
    }

    Controller(const YAML::Node & node, const rclcpp::Logger & logger)
    {
      if (const auto traffic_lights {node["TrafficLight"]}) {
        for (const auto & each : traffic_lights) {
          if (const auto arrow {each["Arrow"]}) {
            // NOTE: tag 'Arrow' is deperecated
            RCLCPP_WARN_STREAM(
              logger, "Tag 'Arrow: <String>' is deprecated. Use 'Arrows: [<String>*]'");
            transitions_.emplace_back(each["Id"], each["Color"], arrow);
          } else {
            transitions_.emplace_back(each["Id"], each["Color"], each["Arrows"]);
          }
        }
      } else {
        RCLCPP_ERROR_STREAM(logger, "Each element of node 'Control' requires hash 'TrafficLight'.");
      }
    }

    bool operator()(ScenarioAPI & simulator) const
    {
      return
        std::all_of(
        transitions_.begin(), transitions_.end(),
        [&](const auto & transition)
        {
          return transition(simulator);
        });
    }
  };

  const YAML::Node script_;

  const std::shared_ptr<ScenarioAPI> simulator_;

  std::vector<std::size_t> ids_;

  std::unordered_map<std::string, Controller> change_to_;

  std::string current_state_;

public:
  Intersection(const YAML::Node &, const std::shared_ptr<ScenarioAPI> &);

  bool change_to(const std::string & the_state);

  const auto & current_state() const noexcept
  {
    return current_state_;
  }

  bool is(const std::string & state) const
  {
    return current_state() == state;
  }

  const std::vector<std::size_t> & ids() const;

  simulation_is update();
};

}  // namespace scenario_intersection
#endif  // SCENARIO_INTERSECTION__INTERSECTION_HPP_
