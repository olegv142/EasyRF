//
// Receiving 2 bytes packets from rf69test and sending them back
//

#include "ERF69.h"
#include <string.h>

#define CS_PIN  14
#define RST_PIN 15

static RF69 g_rf(CS_PIN, RST_PIN);

uint8_t test[] = {2, 3, 5};

uint8_t key[16] = {1,2,3,4,5,6,7,8};

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
  if (!g_rf.start_rx())
    Serial.println("failed to start receiving");
}

void loop() {
  RF69_mode_t m = g_rf.get_mode();
  if (m != rf_rx)
    Serial.println(m);
  else
    Serial.print('.');
  if (g_rf.packet_rxed()) {
    if (!g_rf.cancel()) {
      Serial.println("failed to cancel receiving");
      return;
    }
    uint8_t pkt[3];
    g_rf.rd_packet(pkt, sizeof(pkt));
    Serial.println();
    Serial.print(pkt[0]);
    Serial.print(' ');
    Serial.print(pkt[1]);
    Serial.print(' ');
    Serial.print(pkt[2]);
    if (pkt[0] != 2 || pkt[1] != pkt[2]) {
      Serial.print(" bad packet");
      ++bad_pkt_cnt;
    }
    Serial.println();
    if (bad_pkt_cnt) {
      Serial.print(bad_pkt_cnt);
      Serial.println(" bad pkts");
    }
    pkt[0] = 2;
    if (!g_rf.send_packet(pkt)) {
      Serial.println("failed to send packet");
      return;
    }
    if (!g_rf.start_rx()) {
      Serial.println("failed to start receiving");
      return;
    }
  }
  delay(200);
}
