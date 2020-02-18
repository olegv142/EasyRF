#pragma once

//
// RF69 transceiver minimalistic driver
//

#include <Arduino.h>
#include <SPI.h>

#define RF69_SPI_SETTINGS 8000000, MSBFIRST, SPI_MODE0
#define RF69_MODE_SWITCH_TOUT 100
#define RF69_PKT_SEND_TOUT    4000

// Operation modes
typedef enum {
	rf_sleep,  // lowest power mode
	rf_idle,   // idle mode
	rf_fs,     // intermediate mode
	rf_tx,     // transmitting
	rf_rx      // receiving
} RF69_mode_t;

// Important events
typedef enum {
	rf_PayloadReady = 1 << 2,
	rf_PacketSent   = 1 << 3,
} RF69_event_t;

// Power modes
typedef enum {
	// The normal mode is for modules without high transmission power capabilities.
	// Don't use it for high power modules since they have transmission path 
	// connected to separate pin.
	rf_pw_normal = 0,
	// The following modes are for the high power modules only
	// They are using power amplifiers connected to separate pin of the chip (PA_BOOST)
	rf_pw_boost_normal,
	rf_pw_boost_high,
	rf_pw_boost_max,
} RF69_pw_mode_t;

class RF69 {
public:
	// Encryption key length
	static const unsigned key_len = 16;
	// The maximum length of the message payload (not including length prefix)
	static const unsigned max_payload_len = 60;
	// The minimum and maximum baud rate
	static const unsigned min_baud_rate = 500;    // 12.5kHz rx bandwidth given 5kHz freq_margin
	static const uint32_t max_baud_rate = 200000; // 500kHz rx bandwidth (the maximum allowed)

	// Create transceiver object given the chip select and reset pins
	RF69(uint8_t cs_pin, uint8_t rst_pin)
		: m_cs_pin(cs_pin), m_rst_pin(rst_pin)
		, m_spi_settings(RF69_SPI_SETTINGS)
		{}

	// Initialize IO ports used for communications
	void   begin();
	// Check if transceiver is connected and powered on. May be called before init (but after begin).
	bool   probe();

	// Initialize transceiver. It makes hard reset first to have it clean. This method must be called
	// before any actions taken. It also may be called to recover from fatal errors. The optimal set of
	// transceiver parameters will be deduced from the desired baud rate passed as parameter. The second
	// parameter specifies the desired tolerance with respect to central frequency drift. The drift 
	// originates from the quartz oscillator temperature dependence. It is typically in the range 10-20 ppm.
	// So the effect is more noticeable in higher frequency bands. The good value is 5000 for 433MHz band and
	// 10000 for high frequency bands.
	// Note that the less baud rate the more range you can get in the field and wise versa. Unfortunately
	// the minimum allowed baud rate is around 500 baud.
	void   init(uint32_t br, uint16_t freq_margin);

	// Set carrier frequency
	void   set_freq(uint32_t freq_khz);

	// Set transmit power and power mode.
	// The tx_pw must be in the range -16..15. Negative settings may not work on high power modules.
	// You may set tx_pw_mode to non default value only if you have high power module.
	// For low power module using this method is optional. You may work with the module defaults that give you the maximum power.
	// For high power modules you have to call this method and pass one of the boost modes as second parameter. Otherwise
	// the transmit power will be quite low.
	void   set_tx_power(int8_t tx_pw, RF69_pw_mode_t tx_pw_mode = rf_pw_normal);

	// Both communication devices must be initialized with the same network_id.
	// It provides the simple means for filtering garbage packets catch from the
	// noise and coming from foreign transmitters.
	void   set_network_id(uint32_t id);
	// Set encryption key (16 bytes long). Called with zero key pointer will clear current key.
	void   set_key(uint8_t const* key);

	//
	// The following methods will initiate transitions to different operating modes
	// and wait till transition completion. They may fail in case the transition does not
	// complete within predefined timeout. This typically means that transceiver becomes	
	// unresponsive and should be reinitialized.
	//
	bool   sleep()    { return switch_mode(rf_sleep); }
	bool   start_tx() { return switch_mode(rf_tx); }
	bool   start_rx() { return switch_mode(rf_rx); }
	bool   cancel()   { return switch_mode(rf_idle); }

	// Query current operating mode
	RF69_mode_t get_mode() { return (RF69_mode_t)((rd_reg(1) >> 2) & 7); }
	// Return the last set mode
	RF69_mode_t last_mode() { return (RF69_mode_t)m_flags.last_mode; }

	// Write packet to transceiver. Should be called in idle state.
	// The packet uses length prefix equals to the length of the message payload that follows.
	// The maximum payload size is max_payload_len = 60 bytes (61 bytes taking length prefix into account).
	// You have to call start_tx() to trigger actual packet transmission.
	void   wr_packet(uint8_t const* data);
	// Read packet to the given buffer. If packet corrupted or does not fit to the buffer or addr does not match
	// the first byte of packet payload the method returns false. Zero address match any other address.
	// The method may be called either in idle or receiving state. In the latter case the receiving will be
	// restarted automatically upon packet read.
	bool   rd_packet(uint8_t* buff, uint8_t buff_len, uint8_t addr = 0);

	// Check if the packet was sent successfully in transmit mode.
	bool   packet_sent() { return chk_events(rf_PacketSent); }
	// Check if new packet was received in receive mode.
	bool   packet_rxed() { return chk_events(rf_PayloadReady); }
	// Wait particular event.
	bool   wait_event(RF69_event_t e, uint16_t tout);
	// Write packet and send it waiting for completion.
	bool   send_packet(uint8_t const* data) {
			wr_packet(data);
			return start_tx() && wait_event(rf_PacketSent, RF69_PKT_SEND_TOUT);
		}
	// Returns the transceiver firmware version
	uint8_t get_version() { return rd_reg(0x10); }

protected:
	// Hard reset transceiver
	void	reset();
	// Data exchange transaction begin / end
	void    tx_begin();
	void    tx_end();
	// Exchange data with transceiver registers
	uint8_t	tx_reg(uint16_t w);
	uint8_t rd_reg(uint8_t addr) {
			return tx_reg(addr << 8);
		}
	void    wr_reg(uint8_t addr, uint8_t val) {
			tx_reg(((0x80 | addr) << 8) | val);
		}
	void    wr_burst(uint8_t addr, uint8_t const* data, uint8_t len);
	// Configure baud rate, returns actual rate set
	void    set_baud_rate(uint32_t br);
	// Set frequency deviation
	void    set_fdev(uint32_t fdev);
	// Set receiver bandwidth
	void    set_rx_bw(uint32_t bw);
	// Start operation mode switch
	void    set_mode(RF69_mode_t m);
	// Wait operation mode switch completes
	bool    wait_mode(RF69_mode_t m, uint8_t tout = RF69_MODE_SWITCH_TOUT);
	// Trigger mode switch and wait completion
	bool    switch_mode(RF69_mode_t m) {
			set_mode(m); return wait_mode(m);
		}
	// Get events bitmask
	uint8_t get_events() { return rd_reg(0x28); }
	// Check if particular events active
	bool    chk_events(RF69_event_t e) {
			return (get_events() & e) == e;
		}
	// Clear FIFO before writing new packet
	void    clr_fifo() {
			wr_reg(0x28, 1 << 4);
		}
	// Restart receive in case FIFO was not read completely (so auto restart did not occurred)
	void    restart_rx() {
			wr_reg(0x3d, rd_reg(0x3d) | (1 << 2));
		}

private:
	uint8_t		m_cs_pin;
	uint8_t		m_rst_pin;
	SPISettings	m_spi_settings;
	struct {
		uint8_t	last_mode : 3;
		uint8_t	max_boost : 1;
	} m_flags;
};
