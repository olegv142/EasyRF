# EasyRF
## Minimalistic RF69 radio library for Arduino with small memory footprint

![RFM69CW module](https://github.com/olegv142/EasyRF/blob/master/doc/rf69cw.jpg)

The code is written solely based on the datasheet using default settings as much as possible to minimize code size. It differs from the alternative implementations (RadioHead for ex.) in one important aspect - it does not use interrupts to transfer received data to the memory buffer. The motivation is twofold:
1. The RF chip has its own packet buffer (FIFO). Copying its content makes sense only in case you have significantly larger buffer in RAM. The buffer capable of storing single packet is just meaningless waste of resources.
2. Transferring packet in interrupt service routine should be definitely avoided. Transferring data via SPI introduces wait for the transfer completion (unless it involves DMA). During the wait you can miss important events, for ex receiving bytes via other interfaces such as UART.

Instead of interrupt line the physical connection of the RF69 module to microcontroller includes the reset line. Using hard reset is necessary to be able to recover from any kind of anomalies that may occur during long term operation. Other important additions in comparison to existing libraries are the following:
- Simple configuration. Just choose baud rate and all necessary parameters will be automatically deduced.
- Additional payload protection by encrypted checksum at the end of the payload. It is necessary since attacker can present us with garbage packet even in case the packet is encrypted and he does not know encryption key. The garbage can be decrypted the same way as normal packet so we need the way to filter it out. The second checksum is the simplest way to do it. Since it is encrypted it can't be constructed without knowing the encryption key.

The code compiled for AVR uses less than 4kB of flash. The instance of transceiver driver uses 5 bytes in RAM.

## Compatibility
The code is compiled for AVR, STM32.
Tested with ATmega328P, RFM69CW, RFM69HCW

## Range test results
Using 500 baud bit rate and maximum power settings.

RFM69CW -> RFM69CW with 4cm spring antenna (as on the figure above), open air: 400m

RFM69HCW with 2x5cm spring dipole -> RFM69CW with 4cm spring antenna, open air partially covered with forest: 1km

## Author

Oleg Volkov (olegv142@gmail.com)
