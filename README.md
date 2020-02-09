# EasyRF
## Minimalistic RF69 radio library with small memory footprint

The code is partially based on the RadioHead library with one important modification - is does not use interrupts to transfer received data to the memory buffer. The motivation is twofold:
1. The RF chip has its own packet buffer (FIFO). Copying its content makes sense only in case you have significantly larger buffer in RAM. Small cheap microcontrollers does not necessary have enough RAM for such buffer.
2. Transferring packet in interrupt service routine looks silly. ISR are for transferring words to / from MCU registers. Transferring data via SPI introduces wait for the transfer completion (unless it involves DMA). Waiting anything in ISR is bad practice and should be avoided. Otherwise you can miss important events, for ex receiving bytes via other interfaces such as UART.
