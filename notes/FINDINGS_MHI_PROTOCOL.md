# MHI Protocol Findings

## Purpose and evidence boundaries

This document consolidates the protocol facts used by the ESP-IDF rewrite and
separates them from project-specific hardware findings.

Two evidence sources are used:

1. **Protocol baseline** — the supplied MHI SPI protocol reference describing
   the three-wire bus, 20-byte frame, signatures, checksum, core control fields,
   and operation-data requests.
2. **Project findings** — observations from the ESP32/ESP32-S3 implementation,
   33-byte units, command-confirmation testing, fan-profile testing, long
   transport soaks, and the complete louver/3D Auto matrix.

Model-specific observations must not be treated as universal across all MHI
indoor units. The project preserves unknown bits and validates returned MOSI
state rather than assuming every undocumented field has the same meaning on
all models.

## Physical bus

The air conditioner is the bus master and supplies the clock.

| Signal | Direction relative to AC | Purpose |
|---|---|---|
| `SCK` | Output | AC-provided serial clock |
| `MOSI` | Output | AC status and feedback sent to the controller |
| `MISO` | Input | Controller command data returned to the AC |

There is no physical slave-select/chip-select signal.

The documented clock mode is:

```text
CPOL = 1: clock idles high
CPHA = 1: data is sampled on the rising edge and changes on the falling edge
```

This is SPI mode 3, but the missing CS line means normal transaction boundaries
cannot be taken directly from a pin. The project derives boundaries from the
SCK idle gap and then validates complete frames by length, signature, and
checksum.

Representative timing for an SRK ZS-S unit is:

| Timing | Representative value |
|---|---:|
| Frame duration | `10 ms` |
| Inter-frame pause | `40 ms` |
| Byte duration | `250 us` |
| Inter-byte pause | `250 us` |
| Bit period | `31.25 us` |
| Nominal cadence | `20 frames/s` |

Other models can use different timing. Driver thresholds therefore need to
remain configurable and must be validated on hardware.

## Direction and terminology

The AC is authoritative for state:

```text
MISO command from controller
  -> AC processes request
  -> MOSI status/feedback from AC
  -> semantic confirmation
```

Queueing or transmitting a MISO frame does not prove that the AC accepted the
command. The implementation starts confirmation after actual bus transmission
and confirms against returned MOSI fields.

## Base 20-byte frame

The documented 20-byte frame contains three signature bytes, fifteen data bytes,
and a two-byte additive checksum.

| Raw index | Field |
|---:|---|
| 0 | `SB0` |
| 1 | `SB1` |
| 2 | `SB2` |
| 3 | `DB0` |
| 4 | `DB1` |
| 5 | `DB2` |
| 6 | `DB3` |
| 7 | `DB4` |
| 8 | `DB5` |
| 9 | `DB6` |
| 10 | `DB7` |
| 11 | `DB8` |
| 12 | `DB9` |
| 13 | `DB10` |
| 14 | `DB11` |
| 15 | `DB12` |
| 16 | `DB13` |
| 17 | `DB14` |
| 18 | Checksum high |
| 19 | Checksum low |

Known signatures:

```text
MOSI: 0x6C 0x80 0x04
      or 0x6D 0x80 0x04 on some units

MISO: 0xA9 0x00 0x07
```

The base checksum is:

```text
checksum = sum(SB0..SB2) + sum(DB0..DB14)
```

The high and low bytes are stored at raw positions 18 and 19.

## Extended 33-byte frame

Some units use a 33-byte frame. The first 20 bytes retain the base-frame
positions, followed by extended data and a final extended checksum byte.

The project uses these logical positions:

| Logical field | Raw index | Project finding |
|---|---:|---|
| `DB15` | 20 | Extended data |
| `DB16` | 21 | Horizontal vane fixed/retained position |
| `DB17` | 22 | Horizontal swing and 3D Auto |
| `DB18..DB26` | 23..31 | Other extended data |
| extended checksum | 32 | Final checksum byte |

The detailed horizontal and 3D mapping is a project hardware finding rather
than part of the supplied 20-byte baseline. See
[`FINDINGS_LOUVERS_3D_AUTO.md`](FINDINGS_LOUVERS_3D_AUTO.md).

## Core control fields

### Power

| Direction | Field | Meaning |
|---|---|---|
| MOSI status | `DB0[0]` | Current power state |
| MISO command | `DB0[1]` | Power set bit |

### Mode

MOSI `DB0[4:2]`:

| Bits | Mode |
|---|---|
| `000` | Auto |
| `001` | Dry |
| `010` | Cool |
| `011` | Fan |
| `100` | Heat |

MISO uses the same value with command/set bit `DB0[5]`.

### Temperature setpoint

MOSI `DB2[6:0]`:

```text
setpoint C = DB2 / 2
```

Resolution is `0.5 C`. MISO command/set bit is `DB2[7]`.

### Room temperature

MOSI `DB3`:

```text
room temperature C = (DB3 - 61) / 4
```

Resolution is `0.25 C`.

### Error code

MOSI `DB4` is the reported error code. `0` means no reported error.

## Fan findings

The legacy protocol description expresses the four fixed levels through
`DB1[1:0]` plus `DB6[6]`, with a separate MISO set form for the highest fixed
speed. The ESPHome component uses a normalized internal fan code and two
presentation profiles.

Project-observed normalized values are:

| Internal/status code | Meaning under `four_speed` |
|---:|---|
| `0` | Quiet |
| `1` | Low |
| `2` | Medium |
| `6` | High |
| `7` | Auto |

The normalized code is an implementation-level representation and should not be
confused with the legacy documentation's user-facing level numbers `1..4`.
Hardware testing confirmed that code `0` is distinct from Low on the tested
units.

See [`FINDINGS_FAN_PROFILES.md`](FINDINGS_FAN_PROFILES.md) for encoding,
profile behaviour, and validation.

## Vertical vane findings

The supplied protocol baseline identifies:

| Field | Meaning |
|---|---|
| MOSI `DB0[6]` | Vertical swing status |
| MOSI `DB1[5:4]` | Fixed vertical position |
| MISO `DB0[7]` | Vertical swing command/set bit |
| MISO `DB1[7]` | Fixed vertical position command/set bit |

Project hardware testing confirmed the four fixed positions and established an
important confirmation rule: while vertical swing is active, the returned
`DB1` position bits retain the previous fixed position. Swing confirmation must
therefore check `DB0[6]` and ignore retained `DB1[5:4]`.

## Horizontal vane and 3D Auto findings

On tested 33-byte units:

```text
DB16      horizontal fixed position, or retained position during swing
DB17[0]   horizontal swing
DB17[2]   3D Auto, mask 0x04
```

Horizontal and 3D Auto share one encoded `DB16`/`DB17` domain. A command that
changes either field must preserve the other field. Newer horizontal/3D intent
supersedes stale pending confirmation and can be retransmitted as a combined
`0x60` command mask.

The complete 80-case mapping and confirmation rules are documented in
[`FINDINGS_LOUVERS_3D_AUTO.md`](FINDINGS_LOUVERS_3D_AUTO.md).

## Operation data

Operation-data requests use MISO request fields and are returned in MOSI frames.
The supplied protocol reference documents examples such as outdoor temperature,
last-error data, mode, set temperature, return-air temperature, runtime,
compressor frequency, current, discharge temperature, and superheat.

Key implementation finding:

- opdata must be stored by semantic key rather than in one generic latest-frame
  slot;
- repeated status can overwrite older status safely;
- different opdata fields must retain separate latest values;
- unsupported or absent fields should remain unavailable rather than being
  fabricated as zero.

## Command confirmation findings

The project confirms commands semantically rather than byte-for-byte:

- power, mode, temperature, and fan compare their relevant status fields;
- vertical swing ignores retained fixed-position bits;
- horizontal swing ignores retained `DB16` position;
- 3D Auto confirms from `DB17[2]` independently;
- horizontal confirmation also verifies the preserved 3D companion state;
- duplicate or already-confirmed requests are suppressed;
- retry timeout is shorter for horizontal/3D than for ordinary fields;
- a recovered timeout is not a hard failure when retry exhaustion remains zero
  and the final physical/published state matches.

## Transport implications

The bus constraints explain the retained transport options:

- software FastGPIO for conservative compatibility;
- external-clock RX experiments to reduce synchronous RX pressure;
- `rmt_spi_rx` as an ESP32-S3 DMA-backed receive-only reference;
- `rmt_cs_spi` as the preferred FIFO-backed integrated ESP32/ESP32-S3 path.

See [`FINDINGS_SPI_TRANSPORTS.md`](FINDINGS_SPI_TRANSPORTS.md).

## Limitations and unknowns

- Timing and undocumented fields can vary by model.
- IR control can clear or change protocol set indicators.
- Vertical status may reflect the latest SPI-controller change rather than an IR
  command.
- Some DB17 invariant bits are preserved but not independently decoded.
- Extended fields outside the confirmed louver domain remain only partially
  understood.
- Opdata availability differs between indoor/outdoor unit combinations.

New model findings should include raw frames, target hardware, frame size,
driver configuration, returned MOSI state, and final diagnostic counters.

## Related documents

- [`../ARCHITECTURE.md`](../ARCHITECTURE.md)
- [`../DRIVER_SELECTION.md`](../DRIVER_SELECTION.md)
- [`../DIAGNOSTICS.md`](../DIAGNOSTICS.md)
- [`FINDINGS_SPI_TRANSPORTS.md`](FINDINGS_SPI_TRANSPORTS.md)
- [`FINDINGS_FAN_PROFILES.md`](FINDINGS_FAN_PROFILES.md)
- [`FINDINGS_LOUVERS_3D_AUTO.md`](FINDINGS_LOUVERS_3D_AUTO.md)
- [`history/FINDINGS_FASTGPIO_EXTERNAL_CLOCK.md`](history/FINDINGS_FASTGPIO_EXTERNAL_CLOCK.md)
