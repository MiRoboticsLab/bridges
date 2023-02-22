// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef POSIX_SOCKETS_HPP_
#define POSIX_SOCKETS_HPP_

#include <unistd.h>
#include <linux/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <arpa/inet.h>
#if defined(__VMS)
#include <ioctl.h>
#endif
#include <fcntl.h>

#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace bridge
{
class PosixSocket final
{
public:
  PosixSocket() = delete;
  static int open_nb_socket(const char * addr, const char * port)
  {
    struct addrinfo hints;
    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;             /* IPv4 or IPv6 */
    hints.ai_socktype = SOCK_STREAM;             /* Must be TCP */
    int sockfd = -1;
    int rv;
    struct addrinfo * p, * servinfo;
    /* get address information */
    rv = getaddrinfo(addr, port, &hints, &servinfo);
    if (rv != 0) {
      ERROR("Failed to open socket (getaddrinfo): %s\n", gai_strerror(rv));
      return -1;
    }
    /* open the first possible socket */
    for (p = servinfo; p != NULL; p = p->ai_next) {
      sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
      if (sockfd == -1) {continue;}
      /* connect to server */
      rv = connect(sockfd, p->ai_addr, p->ai_addrlen);
      if (rv == -1) {
        close(sockfd);
        sockfd = -1;
        continue;
      }
      break;
    }
    /* free servinfo */
    freeaddrinfo(servinfo);
    /* make non-blocking */
#if !defined(WIN32)
    if (sockfd != -1) {
      fcntl(sockfd, F_SETFL, fcntl(sockfd, F_GETFL) | O_NONBLOCK);
    }
#else
    if (sockfd != INVALID_SOCKET) {
      int iMode = 1;
      ioctlsocket(sockfd, FIONBIO, &iMode);
    }
#endif
#if defined(__VMS)
    /*
                    OpenVMS only partially implements fcntl. It works on file descriptors
                    but silently fails on socket descriptors. So we need to fall back on
                    to the older ioctl system to set non-blocking IO
    */
    int on = 1;
    if (sockfd != -1) {ioctl(sockfd, FIONBIO, &on);}
#endif
    /* return the new socket fd */
    if (!p && sockfd == -1) {
      ERROR("Failed to open the first possible socket!");
    }
    return sockfd;
  }
};
}  // namespace bridge
}  // namespace cyberdog

#endif  // POSIX_SOCKETS_HPP_
