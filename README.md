# microkvs-stress-test
Stress test platform for microkvs in the presence of unexpected resets and power cycles

The basic concept is simple: two STM32L431s, with the first (test controller) able to reset or power cycle the second
(DUT).

The DUT
