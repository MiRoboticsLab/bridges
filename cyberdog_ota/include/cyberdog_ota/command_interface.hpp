#ifndef CYBERDOG_OTA_COMMAND_INTERFACE_HPP_
#define CYBERDOG_OTA_COMMAND_INTERFACE_HPP_


namespace cyberdog
{

enum class CommandType
{
  kQueryVerion,
  kForceUpgrade,
  kExecuteShell,
  kExecuteAppBin
};

enum class ExecuteState
{
  kUnknown,
  kSuccess,
  kFailure,
  kRunning
};

class CommandInterface
{
public:
  CommandInterface() {}
  virtual ~CommandInterface() {}

  CommandInterface(const CommandInterface&) = delete;
  CommandInterface& operator=(const CommandInterface&) = delete;

  virtual bool Execute(const CommandType& type) = 0;
  virtual ExecuteState GetExecuteState() = 0;
};


}  // namespace cyberdog

#endif  // CYBERDOG_OTA_COMMAND_INTERFACE_HPP_