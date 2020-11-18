#include <boost/cstdlib.hpp>
#include <scenario_runner/scenario_terminater.h>

namespace scenario_runner
{

ScenarioTerminator::ScenarioTerminator(const char * host, int port)
  : client_ { host, port }
  , status { boost::exit_failure }
{}

void ScenarioTerminator::sendTerminateRequest(int exit_status)
{
  XmlRpc::XmlRpcValue value;
  value.setSize(1);
  value[0] = status = exit_status;

  XmlRpc::XmlRpcValue result;

  client_.execute("terminate", value, result);
}

void ScenarioTerminator::update_mileage(double new_mileage)
{
  XmlRpc::XmlRpcValue value;
  value.setSize(1);

  value[0] = new_mileage;

  XmlRpc::XmlRpcValue result;

  client_.execute("update_mileage", value, result);
}

void ScenarioTerminator::update_duration(double new_duration)
{
  XmlRpc::XmlRpcValue value;
  value.setSize(1);

  value[0] = new_duration;

  XmlRpc::XmlRpcValue result;

  client_.execute("update_duration", value, result);
}

}  // namespace scenario_runner
