Improved motion event logger so that it handles various corner cases including power loss. Flash logging appends new data  after the last record after power loss, either intentional (battery change) or intermittent (power failure or deep brownout, etc). Added additional data parameters to the flash storage page. Added CRC check, partial write check. Added various tests for successful write of flash data. Corrected some minor formatting errors.

There is the main applications sketch as well as two helper sketches: one to completely erase the flash before starting a logging application (takes !70 seconds) and one to read all of the flash pages that have been written and output to the serial monitor in CSV format.

The sketches now have persistent append logging, low-power recovery, retained state validation, CRC-protected records, richer event metadata, and backward-compatible readback.

Recommend that the flash erase sketch is run before first use of the data logging application to reset flash and put flash into a known initial state. Thereafter, data may be logged until the flash is full.

Typical STANDBY current is ~4.5 uA and each wake event uses ~1 uA to log the data.
