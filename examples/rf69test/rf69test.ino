//
// Periodically send 2 bytes packet and try to receive it back
//

#include "ERF69.h"
#include <string.h>

#define CS_PIN  14
#define RST_PIN 15

static RF69 g_rf(CS_PIN, RST_PIN);

static uint8_t key[16] = {1,2,3,4,5,6,7,8};

static uint8_t ping[] = {2, 0, 0};

unsigned bad_pkt_cnt = 0;

void setup() {
  Serial.begin(9600);
  g_rf.begin();
  struct RF69_config cfg = {
    433000, 600, rx_boost: true
  };
  g_rf.init(&cfg);
  g_rf.set_network_id(0x12345679ULL);
  g_rf.set_key(key);
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
    return;
  }
  if (has_pkt) {
    uint8_t pong[3];
    g_rf.rd_packet(pong, sizeof(pong));
    Serial.println();
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
    if (bad_pkt_cnt) {
      Serial.print(bad_pkt_cnt);
      Serial.println(" bad pkts");
    }
  }
  ping[2] = ++ping[1];
  if (!g_rf.send_packet(ping)) {
    Serial.println("failed to send packet");
    return;
  }
  if (!g_rf.start_rx()) {
    Serial.println("failed to start receiving");
    return;
  }
  delay(500);
}
