#include "SRV_UDP_Interface.h"
#include "SRV_UDP_Cfg.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

static const char *TAG = "SRV_UDP";
static int udp_socket = -1;
static struct sockaddr_in dest_addr;

ErrorState_TypeDef SRV_UDP_Init(void)
{
    if (udp_socket >= 0) return FUNCTION_OK;
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_socket < 0) { ESP_LOGE(TAG, "UDP socket creation failed"); return FUNCTION_ERROR; }
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_addr.s_addr = inet_addr(SRV_UDP_HOST_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(SRV_UDP_PORT);
    struct sockaddr_in local_addr = {0};
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(SRV_UDP_PORT);
    if (bind(udp_socket, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "UDP bind failed on port %d", SRV_UDP_PORT);
        close(udp_socket); udp_socket = -1; return FUNCTION_ERROR;
    }
    int flags = fcntl(udp_socket, F_GETFL, 0);
    if (flags < 0 || fcntl(udp_socket, F_SETFL, flags | O_NONBLOCK) < 0) {
        ESP_LOGE(TAG, "Could not make UDP socket non-blocking");
        close(udp_socket); udp_socket = -1; return FUNCTION_ERROR;
    }
    ESP_LOGI(TAG, "UDP telemetry/command socket active on port %d", SRV_UDP_PORT);
    return FUNCTION_OK;
}

ErrorState_TypeDef SRV_UDP_Send(const char *payload)
{
    if (udp_socket < 0 || payload == NULL) return FUNCTION_COMMUNICATION_ERROR;
    int err = sendto(udp_socket, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    return (err < 0) ? FUNCTION_ERROR : FUNCTION_OK;
}

ErrorState_TypeDef SRV_UDP_Receive(char *buffer, uint16_t max_len)
{
    if (udp_socket < 0 || buffer == NULL || max_len < 2) return FUNCTION_COMMUNICATION_ERROR;
    int len = recv(udp_socket, buffer, max_len - 1, 0);
    if (len > 0) { buffer[len] = '\0'; return FUNCTION_OK; }
    return FUNCTION_BUSY;
}

