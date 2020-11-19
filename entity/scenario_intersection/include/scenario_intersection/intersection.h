#ifndef SCENARIO_INTERSECTION_INTERSECTION_H_INCLUDED
#define SCENARIO_INTERSECTION_INTERSECTION_H_INCLUDED

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <boost/lexical_cast.hpp>

#include <yaml-cpp/yaml.h>

#include <scenario_api/scenario_api_core.h>
#include <scenario_intersection/arrow.h>
#include <scenario_intersection/color.h>
#include <scenario_intersection/utility.h>
#include <scenario_utility/scenario_utility.h>

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
        : target_ {-1}
        , color_ {Color::Blank}
      {}

      Transition(
        const YAML::Node& target,
        const YAML::Node& color,
        const YAML::Node& arrows)
        : target_ {target.as<int>()}
        , color_ {color ? convert<Color>(color.as<std::string>()) : Color::Blank}
      {
        if (arrows and not arrows.IsNull())
        {
          if (arrows.IsScalar()) // NOTE: deperecated behavior
          {
            const auto value { convert<Arrow>(arrows.as<std::string>()) };

            if (value != Arrow::Blank)
            {
              arrows_.emplace_back(value);
            }
          }
          else for (const auto& each : arrows)
          {
            const auto value { convert<Arrow>(each.as<std::string>()) };

            if (value != Arrow::Blank)
            {
              arrows_.emplace_back(value);
            }
          }
        }
      }

      bool changeColor(ScenarioAPI& simulator) const
      {
        if (target_ < 0 or color_ == Color::Blank)
        {
          return simulator.resetTrafficLightColor(target_, false);
        }
        else // NOTE: Maybe specified illiegal traffic-light-id.
        {
          return simulator.setTrafficLightColor(target_, boost::lexical_cast<std::string>(color_), false);
        }
      }

      bool changeArrow(ScenarioAPI& simulator) const
      {
        simulator.resetTrafficLightArrow(target_, false);

        if (0 <= target_)
        {
          return
            std::all_of(
              std::begin(arrows_), std::end(arrows_),
              [&](const auto& each)
              {
                return
                  simulator.setTrafficLightArrow(
                    target_, boost::lexical_cast<std::string>(each), false);
              });
        }
        else
        {
          return false;
        }
      }

      bool operator()(ScenarioAPI& simulator) const
      {
        return changeColor(simulator) and changeArrow(simulator);
      }
    };

    std::vector<Transition> transitions_;

  public:
    Controller()
    {
      transitions_.emplace_back();
    }

    Controller(const YAML::Node& node)
    {
      if (const auto traffic_lights {node["TrafficLight"]})
      {
        for (const auto& each : traffic_lights)
        {
          if (const auto arrow { each["Arrow"] })
          {
            // NOTE: tag 'Arrow' is deperecated
            ROS_WARN_STREAM("Tag 'Arrow: <String>' is deperecated. Use 'Arrows: [<String>*]'");
            transitions_.emplace_back(each["Id"], each["Color"], arrow);
          }
          else
          {
            transitions_.emplace_back(each["Id"], each["Color"], each["Arrows"]);
          }
        }
      }
      else
      {
        ROS_ERROR_STREAM("Each element of node 'Control' requires hash 'TrafficLight'.");
      }
    }

    bool operator()(ScenarioAPI& simulator) const
    {
      return
        std::all_of(
          transitions_.begin(), transitions_.end(),
          [&](const auto& transition)
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
  Intersection(const YAML::Node&, const std::shared_ptr<ScenarioAPI>&);

  bool change_to(const std::string& the_state);

  const auto& current_state() const noexcept
  {
    return current_state_;
  }

  bool is(const std::string& state) const
  {
    return current_state() == state;
  }

  const std::vector<std::size_t>& ids() const;

  simulation_is update(const ros::Time&);
};

} // namespace scenario_intersection

#endif // SCENARIO_INTERSECTION_INTERSECTION_H_INCLUDED

