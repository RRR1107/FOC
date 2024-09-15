/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-09-14     17932       the first version
 */
#ifndef APPLICATIONS_SOURCE_ETHERNET_ETHERNET_H_
#define APPLICATIONS_SOURCE_ETHERNET_ETHERNET_H_

#include "socket.h"

#define NET_SERVER      ///<网络

#define SOCKET_Conltrol_NO 1
#define SOCKET_Conltrol_Port 8001



extern void    initW5300(void);
extern Uint16    TcpNetServerInit(SOCKET SocketNo, Uint16 port);
extern void    Isr_CheckNetLink(void);
extern void    CheckNetLink(void);
extern void    CheckNetLink2(void);

void NetUploadData(void);

#endif /* APPLICATIONS_SOURCE_ETHERNET_ETHERNET_H_ */
