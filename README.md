# EasyRF
## Minimalistic RF69 radio library for Arduino with small memory footprint

The code is written solely based on the datasheet using default settings as much as possible to minimize code. It differs from the alternative implementations (RadioHead for ex.) in one important aspect - it does not use interrupts to transfer received data to the memory buffer. The motivation is twofold:
1. The RF chip has its own packet buffer (FIFO). Copying its content makes sense only in case you have significantly larger buffer in RAM. Small cheap microcontrollers does not necessary have enough RAM for such buffer.
2. Transferring packet in interrupt service routine looks silly. ISR are for transferring words to / from MCU registers. Transferring data via SPI introduces wait for the transfer completion (unless it involves DMA). Waiting anything in ISR is bad practice and should be avoided. Otherwise you can miss important events, for ex receiving bytes via other interfaces such as UART.
Instead of interrupt line the physical connection of the RF69 module to microcontroller includes reset line. Using hard reset is necessary to be able to recover from any kind of anomalies that may occur during long term operation.

## Author

Oleg Volkov (olegv142@gmail.com)
