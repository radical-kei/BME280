/**
[sample_drv_bme280]

Copyright (c) [2021] [radical-kei]

This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h> //socket(), bind(), accept(), listen()
#include <arpa/inet.h> // struct sockaddr_in, struct sockaddr, inet_ntoa()
#include <unistd.h>
#include <string.h>
#include <pthread.h>

#include <drv_bme280.h>

#define DEV_I2C "/dev/i2c-1"
#define QUEUELIMIT 5
#define BUFSIZE 1024
#define SERVER_PORT 50000

struct clientdata {
  int sock;
  struct sockaddr_in saddr;
};

int get_temp(struct drv_bme280_data* comp_data)
{
    struct drv_bme280_dev dev;

    dev.dev_path = DEV_I2C;
    dev.settings.osrs_p = DRV_BME280_OSRS_VALUE_16;
    dev.settings.osrs_t = DRV_BME280_OSRS_VALUE_2;
    dev.settings.osrs_h = DRV_BME280_OSRS_VALUE_1;
    dev.settings.iir_filter = DRV_BME280_IIR_VALUE_16;

    if (DRV_BME280_OK != drv_bme280_init(&dev))
    {
        fprintf(stderr, "Failed drv_bme280_init()\n");
        return -1;
    }

    if (DRV_BME280_OK != drv_bme280_get_temp_forcemode(&dev, comp_data))
    {
        drv_bme280_exit(&dev);
        fprintf(stderr, "Failed drv_bme280_get_temp_forcemode()\n");
        return -1;
    }

    printf("%0.2lf deg C, %0.2lf hPa, %0.2lf%%\n", comp_data->temperature, comp_data->pressure * 0.01, comp_data->humidity);
    drv_bme280_exit(&dev);

    return 0;
}

void* get_temp_thread(void* pArg)
{
    int recvMsgSize, sendMsgSize;
    struct clientdata *cdata = pArg;
    struct drv_bme280_data comp_data;
    char buf[BUFSIZE];

    while(1)
    {
        memset(buf, 0, BUFSIZE);
        if ((recvMsgSize = recv(cdata->sock, buf, BUFSIZE, 0)) < 0)
        {
            fprintf(stderr, "Failed recv().\n");
            break;
        }
        else if(recvMsgSize == 0)
        {
            fprintf(stderr, "connection closed by foreign host.\n");
            break;
        }

        if(get_temp(&comp_data) == 0)
        {
            memset(buf, 0, BUFSIZE);
            sprintf(buf, "%0.2lf deg C, %0.2lf hPa, %0.2lf%%\n", comp_data.temperature, comp_data.pressure * 0.01, comp_data.humidity);
        }

        if((sendMsgSize = send(cdata->sock, buf, strlen(buf), 0)) < 0)
        {
            fprintf(stderr, "Failed send().\n");
            break;
        }
        else if(sendMsgSize == 0)
        {
            fprintf(stderr, "connection closed by foreign host.\n");
            break;
        }
    }

    close(cdata->sock);
    free(cdata);
    return NULL;
}

int get_temp_server(void){
    int servSock;
    struct clientdata* clit;
    struct sockaddr_in servSockAddr; //server internet socket address
    struct sockaddr_in clitSockAddr; //client internet socket address
    unsigned int clitLen; // client internet socket address length

    pthread_t th;

    if ((servSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0 ){
        fprintf(stderr, "Failed socket().\n");
        return -1;
    }

    memset(&servSockAddr, 0, sizeof(servSockAddr));
    servSockAddr.sin_family      = AF_INET;
    servSockAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servSockAddr.sin_port        = htons(SERVER_PORT);

    if (bind(servSock, (struct sockaddr *) &servSockAddr, sizeof(servSockAddr) ) < 0 ) {
        fprintf(stderr, "Failed bind().\n");
        close(servSock);
        return -1;
    }

    if (listen(servSock, QUEUELIMIT) < 0) {
        fprintf(stderr, "Failed listen().\n");
        close(servSock);
        return -1;
    }

    while(1) {
        clit = malloc(sizeof(struct clientdata));
        clitLen = sizeof(clit->saddr);
        if ((clit->sock = accept(servSock, (struct sockaddr *) &clit->saddr, &clitLen)) < 0) {
            fprintf(stderr, "Failed accept().\n");
            close(servSock);
            break;
        }
        printf("Connected from %s.\n", inet_ntoa(clit->saddr.sin_addr));

        if (pthread_create(&th, NULL, get_temp_thread, clit) != 0) {
            fprintf(stderr, "Failed pthread_create().\n");
            break;
        }

        if (pthread_detach(th) != 0) {
            fprintf(stderr, "Failed pthread_detach().\n");
            break;
        }
    }
    close(servSock);
    return 0;
}

int main(void)
{
    get_temp_server();
    return 0;
}
