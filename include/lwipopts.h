#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

/* OS Support: FreeRTOS SYS mode */
#define NO_SYS                      0

/* Enable standard BSD Sockets API */
#define LWIP_SOCKET                 1
#define LWIP_NETCONN                1

/* Core Network Protocols */
#define LWIP_IPV4                   1
#define LWIP_TCP                    1
#define LWIP_UDP                    1
#define LWIP_DNS                    1
#define LWIP_ICMP                   1
#define LWIP_IGMP                   1
#define LWIP_DHCP                   1

/* mDNS Configuration */
#define LWIP_MDNS_RESPONDER         1
#define LWIP_NUM_NETIF_CLIENT_DATA  (LWIP_MDNS_RESPONDER)

/* Memory Allocations & Pools */
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    (20 * 1024)
#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_ARP_QUEUE          10
#define PBUF_POOL_SIZE              24
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1

/* Internal lwIP Timeout Pool Size */
#define MEMP_NUM_SYS_TIMEOUT        16

/* CRITICAL FIX: Double background thread stacks to prevent crash */
#define TCPIP_THREAD_STACKSIZE      2048  /* 8 KB */
#define DEFAULT_THREAD_STACKSIZE    2048  /* 8 KB */
#define TCPIP_THREAD_PRIO           2
#define DEFAULT_THREAD_PRIO         2
#define DEFAULT_RAW_RECVMBOX_SIZE   8
#define TCPIP_MBOX_SIZE             8
#define DEFAULT_UDP_RECVMBOX_SIZE   8
#define DEFAULT_TCP_RECVMBOX_SIZE   8
#define DEFAULT_ACCEPTMBOX_SIZE     8

/* Prevent struct timeval conflicts */
#define LWIP_TIMEVAL_PRIVATE        0

/* CYW4343 Wi-Fi Driver Requirement */
#define LWIP_CHECKSUM_CTRL_PER_NETIF 1

#endif /* _LWIPOPTS_H */