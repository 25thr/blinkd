#include "blinkd.h"
#include <stdio.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#define usleep(x) Sleep((x)/1000)
#endif

int main(){
  BlinkHandle* B = blink_create(1.0f);
  int sock = blink_udp_open("127.0.0.1", 7777);
  uint32_t dur, flags;
  while(1){
    // feed eye openness values from camera
    float L = 1.0f, R = 1.0f;
    uint32_t t = (uint32_t) (clock() * 1000 / CLOCKS_PER_SEC);
    blink_update(B, t, L, R, &dur, &flags);
    if(flags & BLINK_EVT_BLINK)
      blink_udp_send(sock, "blink", t, dur, flags);
    usleep(5000); // 200 Hz
  }
  blink_udp_close(sock);
  blink_destroy(B);
}
