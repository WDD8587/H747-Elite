#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string>

static int gSock = -1;

int Mqtt_Connect(const char *broker, int port, const char *client_id)
{
    gSock = socket(AF_INET, SOCK_STREAM, 0);
    if (gSock < 0) { perror("MQTT socket"); return -1; }

    struct hostent *he = gethostbyname(broker);
    if (!he) { fprintf(stderr, "MQTT DNS fail\n"); close(gSock); return -1; }

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)port);
    memcpy(&addr.sin_addr, he->h_addr_list[0], (size_t)he->h_length);

    if (connect(gSock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("MQTT connect"); close(gSock); return -1;
    }

    uint8_t connect_pkt[] = {
        0x10, 0x16,                           /* CONNECT, remaining=22 */
        0x00,0x04,'M','Q','T','T',            /* protocol name */
        0x04,                                  /* level 4 = MQTT 3.1.1 */
        0x02,                                  /* clean session */
        0x00,0x3C,                             /* keep-alive 60s */
        0x00,(uint8_t)strlen(client_id)
    };
    uint8_t buf[64]; memcpy(buf, connect_pkt, sizeof(connect_pkt));
    int pos = sizeof(connect_pkt);
    memcpy(buf + pos, client_id, strlen(client_id));
    pos += (int)strlen(client_id);
    send(gSock, (const char *)buf, (size_t)pos, 0);

    uint8_t ack[4]; recv(gSock, (char *)ack, 4, 0);
    printf("[MQTT] Connected to %s:%d\n", broker, port);
    return 0;
}

void Mqtt_Publish(const char *topic, const char *payload)
{
    if (gSock < 0) return;
    uint8_t buf[512]; int pos = 0;
    buf[pos++] = 0x30;
    uint16_t tlen = (uint16_t)strlen(topic);
    uint32_t plen = (uint32_t)strlen(payload);
    uint32_t rem = 2 + tlen + plen;
    do { buf[pos++] = (uint8_t)(rem & 0x7F); rem >>= 7; } while (rem);
    buf[pos++] = (uint8_t)(tlen >> 8); buf[pos++] = (uint8_t)(tlen & 0xFF);
    memcpy(buf + pos, topic, tlen); pos += tlen;
    memcpy(buf + pos, payload, plen); pos += (int)plen;
    send(gSock, (const char *)buf, (size_t)pos, 0);
}
