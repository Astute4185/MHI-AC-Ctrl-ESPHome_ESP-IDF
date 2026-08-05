# Transport Compile Matrix

This matrix separates source/toolchain compatibility from hardware validation.

A successful compile proves that the selected MHI transport code and shared component build against the ESPHome and ESP-IDF target. It does not prove that GPIO timing, interrupt latency, SPI/RMT behaviour, electrical levels, or MHI command timing work on physical hardware.

## Representative gate

The default gate remains intentionally small:

```bash
./scripts/compile-tests.sh compile representative
```

It covers the current material transport boundaries:

- ESP32-C3 FastGPIO selection;
- ESP32 `rmt_cs_spi`;
- ESP32-S3 `rmt_cs_spi`;
- ESP32-S3 split `rmt_spi_rx` with `fast_gpio_tx`.

## Extended portability gate

Run the complete optional matrix with:

```bash
./scripts/compile-tests.sh validate extended
./scripts/compile-tests.sh compile extended
```

The portability fixtures use `external_clock_rx` with `tx_driver: none`. Because `external_clock_rx` includes the internal FastGPIO recovery path, each portability build compiles:

- the shared component, protocol, command, state, publication, and diagnostic code;
- the external-clock RX backend;
- the FastGPIO RX recovery backend;
- the null TX backend;
- split transport and transport-manager wiring;
- normal ESPHome Wi-Fi and API support.

Targets:

| ESPHome variant | Toolchain note | Hardware claim |
|---|---|---|
| ESP32 | ESP-IDF recommended by pinned ESPHome | Compile coverage only for this fixture |
| ESP32-S2 | ESP-IDF recommended by pinned ESPHome | Not hardware-validated |
| ESP32-S3 | ESP-IDF recommended by pinned ESPHome | Other S3 transports have hardware evidence; this RX-only fixture is compile coverage |
| ESP32-C2 | ESP-IDF recommended by pinned ESPHome | Not hardware-validated |
| ESP32-C3 | ESP-IDF recommended by pinned ESPHome | Compile coverage; runtime validation still required |
| ESP32-C5 | ESP-IDF recommended by pinned ESPHome | Not hardware-validated |
| ESP32-C6 | ESP-IDF recommended by pinned ESPHome | Not hardware-validated |
| ESP32-C61 | ESP-IDF recommended by pinned ESPHome | Not hardware-validated |
| ESP32-S31 | Requires ESP-IDF 6.1 or newer; selected through ESPHome `recommended` | Not hardware-validated |

Thread and Zigbee are deliberately not enabled. C5, C6 and S31 are present because they are Wi-Fi-capable ESP variants, not because their 802.15.4 features are in scope.

## Driver scope

The extended matrix does not claim that every hardware-assisted backend works on every chip.

- `external_clock_rx` and the generic FastGPIO RX fallback are compile-enabled across the Wi-Fi ESP32 variants.
- `rmt_spi_rx` remains ESP32-S3-specific.
- `rmt_cs_spi` remains limited to ESP32 and ESP32-S3 until a target-specific implementation is added.
- FastGPIO TX remains enabled only where the current implementation is supported; portability fixtures are RX-only.

Hardware support should only be documented after physical testing with the board, MHI unit, frame size, commands, soak duration, and final counters recorded.
