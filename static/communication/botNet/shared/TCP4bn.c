/*
 * TCP4bn.c
 *
 *  Created on: 25 févr. 2014
 *      Author: ludo6431
 */

#include "TCP4bn.h"

#if MYADDRT
#include <stdio.h> // perror()
#include <sys/socket.h> // socket(), bind(), listen(), accept()
#include <netinet/in.h> // struct sockaddr_in
#include <arpa/inet.h> // inet_pton()
#include <unistd.h> // read()

int connfd = -1; // file descriptor

int TCP_init(){
    struct sockaddr_in serv_addr = {0};
    int ret;

#if MYADDRT & 1 // odd => TCP server
    int listenfd = 0;
    unsigned int addrlen;
    struct sockaddr_in peer_addr = {0};

    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    if(listenfd < 0){
        perror("socket()");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons((42000 + MYADDRT)&~1);

    ret = bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    if(ret < 0){
        perror("bind()");
        return -1;
    }

    ret = listen(listenfd, 1);
    if(ret < 0){
        perror("listen()");
        return -1;
    }

    printf("waiting for peer to connect on port %hu...\n", ntohs(serv_addr.sin_port));

    addrlen = sizeof(peer_addr);
    connfd = accept(listenfd, (struct sockaddr*)&peer_addr, &addrlen);
    if(connfd < 0){
        perror("accept()");
        return -1;
    }

    printf("accepted connection from peer %hhu.%hhu.%hhu.%hhu:%hu\n", ntohl(peer_addr.sin_addr.s_addr)>>24, ntohl(peer_addr.sin_addr.s_addr)>>16, ntohl(peer_addr.sin_addr.s_addr)>>8, ntohl(peer_addr.sin_addr.s_addr), ntohs(peer_addr.sin_port));
#else // even => TCP client
    connfd = socket(AF_INET, SOCK_STREAM, 0);
    if(connfd < 0){
        perror("socket()");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons((42000 + MYADDRT)&~1);


    ret = inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);
    if(ret < 0){
        perror("inet_pton()");
        return -1;
    }

    printf("trying to connect to server on %hhu.%hhu.%hhu.%hhu:%hu\n", ntohl(serv_addr.sin_addr.s_addr)>>24, ntohl(serv_addr.sin_addr.s_addr)>>16, ntohl(serv_addr.sin_addr.s_addr)>>8, ntohl(serv_addr.sin_addr.s_addr), ntohs(serv_addr.sin_port));

    ret = connect(connfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
    if(ret < 0){
        perror("connect()");
        return -1;
    }
#endif

    return 0;
}

int TCP_receive(sMsg *msg){
    int ret, count;

    for(count = 0; count < sizeof(msg->header); count += ret){
        ret = read(connfd, (char *)msg + count, sizeof(msg->header) - count);
        if(ret < 0){
            perror("read()");
            return -1;
        }
    }

    for(; count < sizeof(msg->header) + msg->header.size; count += ret){
        ret = read(connfd, (char *)msg + count, sizeof(msg->header) + msg->header.size - count);
        if(ret < 0){
            perror("read()");
            return -1;
        }
    }

    return 1;
}

int TCP_send(const sMsg *msg){
    int ret;

    ret = write(connfd, (char*)msg, sizeof(msg->header) + msg->header.size);
    if(ret < 0){
        perror("write()");
        return -1;
    }
    else if(ret != sizeof(msg->header) + msg->header.size){
        return 0;
    }

    return 1;
}

#endif
