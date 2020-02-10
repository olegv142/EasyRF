//
// RF69 transceiver minimalistic driver
//

#include "ERF69.h"

#define RF69_OSC_KHZ 32000

void RF69::begin()
{
	SPI.begin();
	digitalWrite(m_cs_pin, HIGH);
	pinMode(m_cs_pin, OUTPUT);
	digitalWrite(m_rst_pin, LOW);
	pinMode(m_rst_pin, OUTPUT);
}

void RF69::reset()
{
	digitalWrite(m_rst_pin, HIGH);
	delay(1);
	digitalWrite(m_rst_pin, LOW);
	delay(10);
}

bool RF69::probe()
{
	return rd_reg(0x14) == 0x40 && rd_reg(0x15) == 0xB0 && rd_reg(0x16) == 0x7B && rd_reg(0x17) == 0x9B;
}

void RF69::tx_begin()
{
	digitalWrite(m_cs_pin, LOW);
	SPI.beginTransaction(m_spi_settings);
}

void RF69::tx_end()
{
	SPI.endTransaction();
	digitalWrite(m_cs_pin, HIGH);
}

uint8_t	RF69::tx_reg(uint16_t w)
{
	tx_begin();
	uint8_t r = SPI.transfer16(w);
	tx_end();
	return r;
}

void RF69::rd_burst(uint8_t addr, uint8_t* buff, uint8_t buff_len, bool with_len_prefix)
{
	uint8_t data_len;
	tx_begin();
	*buff = SPI.transfer16(addr << 8);
	if (with_len_prefix)
		data_len = *buff;
	else
		data_len = buff_len - 1;
	for (++buff, --buff_len; data_len; --data_len) {
		uint8_t v = SPI.transfer(0);
		if (buff_len) {
			*buff++ = v;
			--buff_len;
		}
	}
	tx_end();
}

void RF69::wr_burst(uint8_t addr, uint8_t const* data, uint8_t len)
{
	tx_begin();
	SPI.transfer16(((0x80 | addr) << 8) | *data);
	for (++data, --len; len; ++data, --len)
		SPI.transfer(*data);
	tx_end();
}

void RF69::init(struct RF69_config const* cfg)
{
	reset();

	// configure carrier freq divider
	uint32_t fdiv = (cfg->freq_khz << 11) / (RF69_OSC_KHZ >> 8);
	wr_reg(7, fdiv >> 16);
	wr_reg(8, fdiv >> 8);
	wr_reg(9, fdiv);

	// configure baud rate
	uint32_t brdiv = RF69_OSC_KHZ * 1000ULL / cfg->baud_rate;
	wr_reg(3, brdiv >> 8);
	wr_reg(4, brdiv);

	wr_reg(0x13, 0); // disable over-current protection
	wr_reg(0x37, (1<<7) | (1<<6) | (1<<4)); // var length packets, whitening, CRC, no address filtering
	wr_reg(0x3c, 1<<7); // start tx on fifo not empty

	if (cfg->rx_boost)
		wr_reg(0x58, 0x2d);

	uint8_t pw_cfg = 0x1f;
	switch (cfg->tx_pw_mode) {
	case rf_pw_boost_max:
	case rf_pw_boost_high:
		pw_cfg |= 1 << 5; // PA2
	case rf_pw_boost_normal:
		pw_cfg |= 1 << 6; // PA1
		break;
	default:
		pw_cfg |= 1 << 7; // PA0
		break;		
	}
	wr_reg(0x11, pw_cfg);
	m_flags.last_mode = rf_idle;
	m_flags.max_boost = (cfg->tx_pw_mode == rf_pw_boost_max);
}

void RF69::set_network_id(uint32_t id)
{
	// Configure sync word
	wr_reg(0x2e, (1 << 7) | (sizeof(id)-1) << 3);
	wr_burst(0x2f, (uint8_t const*)&id, sizeof(id));
}

void RF69::set_key(uint8_t const* key)
{
	if (key) {
		wr_reg(0x3d, 3);
		wr_burst(0x3e, key, RF69_KEY_LENGTH);
	} else
		wr_reg(0x3d, 2);
}

void RF69::set_mode(RF69_mode_t m)
{
	if (m_flags.max_boost) {
		if (m == rf_tx) {
			wr_reg(0x5a, 0x5d);
			wr_reg(0x5c, 0x7c);
		} else if (m_flags.last_mode == rf_tx) {
			wr_reg(0x5a, 0x55);
			wr_reg(0x5c, 0x70);
		}
	}
	m_flags.last_mode = m;
	wr_reg(1, (m << 2));
}

bool RF69::wait_mode(RF69_mode_t m, uint8_t tout = RF69_MODE_SWITCH_TOUT)
{
	uint32_t start = millis();
	while (get_mode() != m) {
		if (millis() - start > tout)
			return false;
	}
	return true;
}

bool RF69::wait_event(RF69_event_t e, uint16_t tout)
{
	uint32_t start = millis();
	while (!(get_events() & e)) {
		if (millis() - start > tout)
			return false;
	}
	return true;
}
