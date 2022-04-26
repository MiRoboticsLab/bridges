#include "cyberdog_ota/server.hpp"
#include "cyberdog_ota/utils.hpp"
#include "cyberdog_ota/manager.hpp"

#include <iostream>
#include <functional>

#include <memory.h>

namespace cyberdog
{

const int kServerPort = 10206;

Server::Server(Manager* manager)
  : manager_ (manager)
{
  bool success = Initialize();
  if (success) {
    std::cout << "Server initialize success." << std::endl;
  }
}

Server::~Server()
{
   Stop();
}

void Server::Run()
{
  fd_set readset;
  int ret = 0;

  while (true)
  {
    shutdown(socket_, SHUT_RDWR);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        close(socket_);
        socket_ = -1;
    }

    FD_ZERO(&readset);
    FD_SET(listen_socket_, &readset);

    ///tv.tv_sec = 0;
    ///tv.tv_usec = 1000 * 200;
    ret = select(listen_socket_ + 1, &readset, NULL, NULL, /*&tv*/NULL);
    if (ret <= 0) {
        continue;
    }

    socket_ = accept(listen_socket_, NULL, NULL);
    if (socket_ < 0) {
        // LOGE("accept fail");
        continue;
    }
    else {
        ///LOGN("accept success");
        SetBlock(socket_);
        SetCloseOnExec(socket_);
        Connected();
        HandleMessages();
    }

    std::cout << "Thread: Server::Run() ... " << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

bool Server::Initialize()
{
  signal(SIGPIPE, SIG_IGN);
  signal(SIGHUP, SIG_IGN);

  if (Start("", kServerPort) < 0) {
    std::cout << "server start fail" << std::endl;
    return false;
  }

  message_handler_ = std::make_shared<std::thread>(&Server::Run, this);
  return true;
}

bool Server::SendMessage(char* buf, size_t size)
{
  int ret = 0;
  int write_ok = 0;

  std::lock_guard<std::mutex> lock(mutex_);
  while (write_ok != size)
  {
    ret = (int)write(socket_, buf + write_ok, size - write_ok);
    if (ret < 0) {
        if ((errno == EAGAIN) || (errno == EWOULDBLOCK) || (errno == EINTR)) {
        continue;
      }
      else if (errno == EPIPE) {
        ///LOGN("client %d closed", m_socket);
        shutdown(socket_, SHUT_RDWR);
        Disconnection();
        return false;
      }
      else {
        // LOGE("write error");
        shutdown(socket_, SHUT_RDWR);
        Disconnection();
        return false;
      }
    } else {
      write_ok += ret;
    }
  }
  return true;
}

bool Server::ReceiveMessage(char* buf, size_t size)
{
  int ret = 0;
  int read_ok = 0;

  while (read_ok != size)
  {
    ret = (int)recv(socket_, buf + read_ok, size - read_ok, 0);
    if (ret < 0) {
        if ((errno == EAGAIN) || (errno == EWOULDBLOCK) || (errno == EINTR)) {
          std::cout << "read error" << std::endl;
          continue;
        }
        else {
          // LOGE("read error");
          std::cout << "read error" << std::endl;
          shutdown(socket_, SHUT_RDWR);
          Disconnection();
          return false;
        }
    }
    else if (0 == ret) {
      // LOGN("client %d closed", m_socket);
      std::cout << "client closed" << std::endl;
      shutdown(socket_, SHUT_RDWR);
      Disconnection();
      return false;
    }
    else {
      read_ok += ret;
    }
  }
  return true;
}

void Server::HandleMessages()
{
  while (true)
  {
    ServerMessage message;
    bool ret = HandleProtocolServerMessage(message);
    if (!ret) {
      return;
    }

    auto request = std::get<0>(message);
    std::cout << "ID = " << request.command << std::endl;

    switch (request.command)
    {
      case CommandID::kStartDownload:
        manager_->RunDownloadFilesCommand();
        std::cout << "Start download ... " << std::endl;
        break;

      case CommandID::kVersionQuery:
        std::cout << "version query ... " << std::endl;
        break;

      case CommandID::kStatusQuery:
        std::cout << "status query ... " << std::endl;
        break;

      case CommandID::kStartUpdate:
        std::cout << "start update ... " << std::endl;
        break;

      case CommandID::kAbortDownload:
        std::cout << "abort download ... " << std::endl;
        break;

      case CommandID::kUpdateTime:
        std::cout << "update time ... " << std::endl;
        break;

      case CommandID::kUpdateProcessQuery:
        std::cout << "update process query ... " << std::endl;
        break;

      case CommandID::kDownloadProcessQuery:
        std::cout << "download process query ... " << std::endl;
        break;
      
      default:
        std::cout << "unknown recv command id : " << static_cast<int>(request.command) << std::endl;
        break;
    }
  }
}

bool Server::HandleProtocolServerMessage(ServerMessage& msg)
{
  // auto request = std::get<0>(msg);

  unsigned int value;
  int len = sizeof(unsigned int);

  if (!ReceiveMessage((char*)(&value), sizeof(unsigned int))) {
      std::cout << "recv data error" << std::endl;
      return false;
  }
 
  Strrevn((char*)(&value), sizeof(unsigned int));
  std::get<0>(msg).command = static_cast<CommandID>(value);
  return true;
}

int Server::Start(const std::string& ip, const uint32_t port)
{
  Stop();

  listen_socket_ = Listen(ip, port, 100);
  if (listen_socket_ < 0) {
      return -1;
  }

  SetNonblock(listen_socket_);
  SetCloseOnExec(listen_socket_);
  return 0;
}

int Server::Stop()
{
  shutdown(listen_socket_, SHUT_RDWR);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown(socket_, SHUT_RDWR);
  }

  close(listen_socket_);
  listen_socket_ = -1;

  close(socket_);
  socket_ = -1;
  return 0;
}

int Server::Connected()
{
  return 0;
}

int Server::Disconnection()
{
   return 0;
}

int Server::Listen(const std::string& ip, const uint32_t port, const int backlog)
{
  int listen_socket = socket(AF_INET, SOCK_STREAM, 0);
  if (-1 == listen_socket)
  {
      // LOGE("socket error");
      return -1;
  }

  struct sockaddr_in server_address;
  memset(&server_address, 0, sizeof(server_address));
  server_address.sin_family = AF_INET;
  server_address.sin_port = htons(port);
  if (ip.empty()) {
      server_address.sin_addr.s_addr = htonl(INADDR_ANY);
  }
  else {
      inet_pton(AF_INET, ip.c_str(), &server_address.sin_addr);
  }

  int reuse = 1;
  if (setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
  {
      // LOGE("setsockopt error");
      close(listen_socket);
      return -2;
  }

  if (bind(listen_socket, (struct sockaddr*)(&server_address), sizeof(struct sockaddr)) < 0)
  {
      // LOGE("bind error");
      close(listen_socket);
      return -3;
  }

  if (listen(listen_socket, backlog) < 0)
  {
      // LOGE("listen error");
      close(listen_socket);
      return -4;
  }

  return listen_socket;
}

int Server::SetNonblock(int fd)
{
  int flags = fcntl(fd, F_GETFL);
  if (-1 == flags) {
      return -1;
  }

  flags = fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  if (-1 == flags) {
      return -2;
  }
  return 0;
}

int Server::SetBlock(int fd)
{
  unsigned long ul = 0;
  if (ioctl(fd, FIONBIO, &ul) < 0) {
      return -1;
  }

  return 0;
}

int Server::SetCloseOnExec(int fd)
{
  int flags = fcntl(fd, F_GETFD);
  if (-1 == flags) {
      return -1;
  }

  flags = fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
  if (-1 == flags) {
      return -2;
  }
  return 0;
}

}  // namespace cyberdog