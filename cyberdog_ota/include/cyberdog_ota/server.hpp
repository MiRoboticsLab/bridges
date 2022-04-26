#ifndef CYBERDOG_OTA_PROTOCOL_SERVER_HPP_
#define CYBERDOG_OTA_PROTOCOL_SERVER_HPP_

#include "cyberdog_ota/server_interface.hpp"
#include "cyberdog_ota/protocol_server_message.hpp"
#include "cyberdog_ota/port.hpp"

#include <thread>
#include <mutex>
#include <memory>
#include <string>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

namespace cyberdog
{

class Manager;

class Server : public ServerInterface
{
public:   
  Server(Manager* manager);        
  virtual ~Server();

  Server(const Server&) = delete;
  Server& operator=(const Server&) = delete;

  virtual bool Initialize() override;

  virtual bool SendMessage(char* buf, size_t size) override;

  virtual bool ReceiveMessage(char* buf, size_t size) override;

  // Handle mobile phone command and schedule 
  void HandleMessages();

private:
  // Handle mobile phone command
  bool HandleProtocolServerMessage(ServerMessage& msg);

  int Start(const std::string& ip, const uint32_t port);
  int Stop();
  
  int Connected();
  int Disconnection();
  int Listen(const std::string& ip, const uint32_t port, const int backlog = 100);
  void Run();

  int SetNonblock(int fd);
  int SetBlock(int fd);
  int SetCloseOnExec(int fd);

  Manager* manager_;
  std::shared_ptr<std::thread> message_handler_;
  int listen_socket_ {-1};
  int socket_ {-1};
  std::mutex mutex_;
  std::mutex write_mutex_;
};



}  // namespace cyberdog

#endif  // CYBERDOG_OTA_PROTOCOL_SERVER_HPP_

