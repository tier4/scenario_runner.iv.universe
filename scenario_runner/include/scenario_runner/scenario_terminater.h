#ifndef SCENARIO_RUNNER_SCENARIO_TERMINATOR_H_INCLUDEED
#define SCENARIO_RUNNER_SCENARIO_TERMINATOR_H_INCLUDEED

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcClient.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace scenario_runner
{
class ScenarioTerminator
{
public:
  ScenarioTerminator(const char * host, int port);

  int status;

  void sendTerminateRequest(int);

  void update_mileage(double = 0);
  void update_duration(double = 0);

private:
  XmlRpc::XmlRpcClient client_;
};
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER_SCENARIO_TERMINATOR_H_INCLUDEED
