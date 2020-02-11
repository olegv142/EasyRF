//
// Periodically send 2 bytes packet and try to receive it back
//

#include "ERF69.h"
#include <string.h>

#define CS_PIN  14
#define RST_PIN 15

static RF69 g_rf(CS_PIN, RST_PIN);
static uint8_t key[RF69::key_len] = {1,2,3,4,5,6,7,8};

static uint8_t ping[] = {2, 0, 0};

unsigned bad_pkt_cnt = 0;
unsigned rf_err_cnt = 0;

struct RF69_config cfg = {
  433000, 600, rx_boost: true
};

void rf_init() {
  g_rf.init(&cfg);
  g_rf.set_network_id(0x12345679ULL);
  g_rf.set_key(key);
}

void setup() {
  Serial.begin(9600);
  g_rf.begin();
  rf_init();
}

void loop() {
  RF69_mode_t m = g_rf.get_mode();
  if (m != rf_rx)
    Serial.println(m);
  else
    Serial.print('.');
  bool has_pkt = g_rf.packet_rxed();
  if (!g_rf.cancel()) {
    Serial.println("failed to cancel receiving");
    goto err;
  }
  if (has_pkt) {
    uint8_t pong[3];
    Serial.println();
    if (g_rf.rd_packet(pong, sizeof(pong))) {
      Serial.print(pong[0]);
      Serial.print(' ');
      Serial.print(pong[1]);
      Serial.print(' ');
      Serial.print(pong[2]);
      if (pong[0] != ping[0] || pong[1] != pong[2]) {
        Serial.print(" bad packet");
        ++bad_pkt_cnt;
      }
      Serial.println();
    } else {
      Serial.println("corrupt packet");
      ++bad_pkt_cnt;
    }
    if (bad_pkt_cnt || rf_err_cnt) {
      Serial.print(bad_pkt_cnt);
      Serial.print(" bad pkts, ");
      Serial.print(rf_err_cnt);
      Serial.println(" hw errors");
    }
  }
  ping[2] = ++ping[1];
  if (!g_rf.send_packet(ping)) {
    Serial.println("failed to send packet");
    goto err;
  }
  if (!g_rf.start_rx()) {
    Serial.println("failed to start receiving");
    goto err;
  }
  delay(500);
  return;

err:
  ++rf_err_cnt;
  rf_init();
}
