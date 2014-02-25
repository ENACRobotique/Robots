/*
 * UDP4bn.c
 *
 *  Created on: 25 févr. 2014
 *      Author: ludo6431
 */

#include "UDP4bn.h"

#if MYADDRD
#include <stdio.h> // perror()
#include <sys/time.h> // select()
#include <sys/types.h> // recvfrom(), select(), sendto()
#include <unistd.h> // select()
#include <sys/socket.h> // socket(), bind(), recvfrom(), sendto()
#include <netinet/in.h> // struct sockaddr_in
#include <arpa/inet.h> // inet_pton()

int sockfd = -1; // file descriptor

int UDP_init(){
    int ret;
    struct sockaddr_in serv_addr = { 0 };

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd < 0){
        perror("socket()");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(42000 + MYADDRD);

    ret = bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
    if(ret < 0){
        perror("bind()");
        return -1;
    }

    printf("peer initialized on port %hu...\n", ntohs(serv_addr.sin_port));

    return 0;
}

int UDP_receive(sMsg *msg){
    int ret;
    fd_set rset;
    struct timeval tv;
    socklen_t len;
    struct sockaddr_in cli_addr = { 0 };

    // check if there is any data
    FD_ZERO(&rset);
    FD_SET(sockfd, &rset);
    tv.tv_sec = 0;
    tv.tv_usec = 500;
    ret = select(sockfd + 1, &rset, NULL, NULL, &tv);
    if(ret < 0){
        perror("select()");
        return -1;
    }
    else if(!ret){
        return 0; // no data available
    }

    // get data if any
    len = sizeof(cli_addr);
    ret = recvfrom(sockfd, (char *)msg, sizeof(*msg), 0, (struct sockaddr *)&cli_addr, &len);
    if(ret < 0){
        perror("recvfrom()");
        return -1;
    }

    printf("received %i bytes from %hhu.%hhu.%hhu.%hhu:%hu\n", ret, ntohl(cli_addr.sin_addr.s_addr)>>24, ntohl(cli_addr.sin_addr.s_addr)>>16, ntohl(cli_addr.sin_addr.s_addr)>>8, ntohl(cli_addr.sin_addr.s_addr), ntohs(cli_addr.sin_port));

    return ret;
}

int UDP_send(const sMsg *msg, bn_Address nextHop){
    int ret;
    struct sockaddr_in serv_addr = { 0 };

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(42000 + nextHop);

    ret = inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);
    if(ret < 0){
        perror("inet_pton()");
        return -1;
    }

    ret = sendto(sockfd, (char *)msg, sizeof(msg->header) + msg->header.size, 0, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
    if(ret < 0){
        perror("sendto()");
        return -1;
    }

    printf("sent %i bytes to %hhu.%hhu.%hhu.%hhu:%hu\n", ret, ntohl(serv_addr.sin_addr.s_addr)>>24, ntohl(serv_addr.sin_addr.s_addr)>>16, ntohl(serv_addr.sin_addr.s_addr)>>8, ntohl(serv_addr.sin_addr.s_addr), ntohs(serv_addr.sin_port));

    return ret;
}

#endif
