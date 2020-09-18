#include <scenario_runner/scenario_terminater.h>

namespace scenario_runner
{
ScenarioTerminator::ScenarioTerminator(const char * host, int port) : client_(host, port) {}

void ScenarioTerminator::sendTerminateRequest(int exit_status)
{
  XmlRpc::XmlRpcValue value;
  value.setSize(1);
  value[0] = exit_status;

  XmlRpc::XmlRpcValue result;

  client_.execute("terminate", value, result);
}
}  // namespace scenario_runner
