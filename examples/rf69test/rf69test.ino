//
// Periodically send 2 bytes packet and try to receive it back
//

#include "ERF69.h"
#include <string.h>

#define CS_PIN  14
#define RST_PIN 15
#define LED_PIN 2

// Uncomment the following lines if you have high power module
//#define HCW_MODULE
//#define HCW_BOOST_MAX

static RF69 g_rf(CS_PIN, RST_PIN);
static uint8_t key[RF69::key_len] = {1,2,3,4,5,6,7,8};

static uint8_t ping[] = {2, 0, 0};

static unsigned bad_pkt_cnt = 0;
static unsigned rf_err_cnt = 0;

void rf_init() {
  g_rf.init(rf_mode_1kb);
  g_rf.set_freq(433000);
  g_rf.set_network_id(0x12345679ULL);
  g_rf.set_key(key);
#ifdef HCW_MODULE
#ifdef HCW_BOOST_MAX
  g_rf.set_tx_power(15, rf_pw_boost_max);
#else
  g_rf.set_tx_power(15, rf_pw_boost_normal);
#endif
#endif
}

void setup() {
  Serial.begin(9600);
  g_rf.begin();
  rf_init();
  digitalWrite(LED_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
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
      } else {
          digitalWrite(LED_PIN, HIGH);
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
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  return;

err:
  ++rf_err_cnt;
  rf_init();
}