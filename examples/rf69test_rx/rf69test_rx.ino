//
// Continuously receiving 2 bytes packets from rf69test
//

#include "ERF69.h"
#include <string.h>

#define CS_PIN  14
#define RST_PIN 15

static RF69 g_rf(CS_PIN, RST_PIN);
uint8_t key[16] = {1,2,3,4,5,6,7,8};

void setup() {
  Serial.begin(9600);
  g_rf.begin();
  struct RF69_config cfg = {
    433000, 600, rx_boost: true
  };
  g_rf.init(&cfg);
  g_rf.set_network_id(0x12345679ULL);
  g_rf.set_key(key);
  if (!g_rf.start_rx())
    Serial.println('failed to start receiving');
}

void loop() {
  RF69_mode_t m = g_rf.get_mode();
  if (m != rf_rx)
    Serial.println(m);
  else
    Serial.print('.');
  if (g_rf.packet_rxed()) {
    uint8_t pkt[3];
    g_rf.rd_packet(pkt, sizeof(pkt));
    Serial.println();
    Serial.print(pkt[0]);
    Serial.print(' ');
    Serial.println(pkt[1]);
  }
  delay(200);
}
