#ifdef _WIN32
  #define WIN32_LEAN_AND_MEAN
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <windows.h>
  #pragma comment(lib, "ws2_32.lib")
  #define usleep(x) Sleep((x)/1000)
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  #define INVALID_SOCKET -1
  #define SOCKET_ERROR   -1
  #define closesocket close
#endif

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>

#include "blinkd.h"

int main(void) {
    printf("[blinkd] Initializing blink detection engine...\n");

#ifndef _WIN32
  // POSIX sockets don't need initialization
#else
WSADATA wsa;
if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
    printf("[blinkd] ERROR: WSAStartup failed\n");
    return 1;
}
#endif

    BlinkdHandle* B = blinkd_create(1.0f);
    blinkd_set_preset(B, BLINK_PRESET_HIGH);
    if (!B) {
        printf("[blinkd] ERROR: Failed to create blinkd instance.\n");
#ifdef _WIN32
  WSACleanup();
#endif
        return 1;
    }

    printf("[blinkd] Opening UDP socket to 127.0.0.1:7777...\n");
    int sock = blinkd_udp_open("127.0.0.1", 7777);
    if (sock < 0) {
        printf("[blinkd] ERROR: Failed to open UDP socket (code %d)\n", sock);
        blinkd_destroy(B);
#ifdef _WIN32
  WSACleanup();
#endif
        return 1;
    }

    printf("[blinkd] Initialization complete.\n");
    printf("[blinkd] Waiting for openness feed from stdin (t openL openR)...\n");
    printf("[blinkd] Press Ctrl+C to stop.\n");

    uint32_t dur = 0, flags = 0;
    char line[256];
    unsigned long count = 0;

    // Main loop
    while (fgets(line, sizeof(line), stdin)) {
        uint32_t t;
        float L, R;

        if (sscanf(line, "%u %f %f", &t, &L, &R) == 3) {
            uint32_t dur = 0, flags = 0;
            blinkd_update(B, t, L, R, &dur, &flags);
#ifdef BLINKD_DEBUG
blinkd_debug_dump(B);
#endif

            if (flags & BLINK_EVT_BLINK) {
                printf("[blinkd] Blink detected! Duration=%ums Flags=0x%X\n", dur, flags);
                blinkd_udp_send(sock, "blink", t, dur, flags);
            }

        if (++count % 1000 == 0)
            fprintf(stderr, "[blinkd] Heartbeat: running smoothly... (%lu cycles)\n", count);
        }
    }

    printf("[blinkd] End of input stream. Cleaning up...\n");

    blinkd_udp_close(sock);
    blinkd_destroy(B);
#ifdef _WIN32
  WSACleanup();
#endif

    printf("[blinkd] Shutdown complete.\n");
    return 0;
}
