#define WIN32_LEAN_AND_MEAN

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>

#include "blinkd.h"

#pragma comment(lib, "ws2_32.lib")

#define usleep(x) Sleep((x)/1000)

int main(void) {
    printf("[blinkd] Initializing blink detection engine...\n");

    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        printf("[blinkd] ERROR: WSAStartup failed\n");
        return 1;
    }

    BlinkHandle* B = blink_create(1.0f);
    if (!B) {
        printf("[blinkd] ERROR: Failed to create BlinkHandle\n");
        WSACleanup();
        return 1;
    }

    printf("[blinkd] Opening UDP socket to 127.0.0.1:7777...\n");
    int sock = blink_udp_open("127.0.0.1", 7777);
    if (sock < 0) {
        printf("[blinkd] ERROR: Failed to open UDP socket (code %d)\n", sock);
        blink_destroy(B);
        WSACleanup();
        return 1;
    }

    printf("[blinkd] Initialization complete.\n");
    printf("[blinkd] Streaming blink events every 5ms (200 Hz)...\n");
    printf("[blinkd] Press Ctrl+C to stop.\n");

    uint32_t dur = 0, flags = 0;
    unsigned long count = 0;

    while (1) {
        float L = 1.0f, R = 1.0f;
        uint32_t t = (uint32_t)(clock() * 1000 / CLOCKS_PER_SEC);

        blink_update(B, t, L, R, &dur, &flags);
        if (flags & BLINK_EVT_BLINK) {
            printf("[blinkd] Blink detected! Duration=%ums Flags=0x%X\n", dur, flags);
            blink_udp_send(sock, "blink", t, dur, flags);
        }

        if ((++count % 1000) == 0)
            printf("[blinkd] Heartbeat: running smoothly... (%lu cycles)\n", count);

        usleep(5000); // 200 Hz
    }

    blink_udp_close(sock);
    blink_destroy(B);
    WSACleanup();

    printf("[blinkd] Shutdown complete.\n");
    return 0;
}
