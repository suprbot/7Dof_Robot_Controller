#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "socket_server.h"
#include "global_variables.h"

void *tcp_server_thread(void *arg)
{
    CommandStackAppendPoint=1;
    /****************socket******************/
    int listenfd = 0, connfd = 0;
    struct sockaddr_in serv_addr;

    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(6000);

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

    listen(listenfd, 10);

    connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);
    /****************socket******************/

    while(1)
    {
        //write to UI
        write(connfd, &rc2tp, sizeof(rc2tp));

        //read from UI
        read(connfd, &tp2rc, sizeof(tp2rc));

        usleep(1000);
     }

     close(connfd);
}
