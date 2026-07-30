# MHI louver raw-bit diagnostic spike

## Purpose

This spike does not change command encoding or confirmation behaviour. It adds trace-only instrumentation so the current vertical-vane, horizontal-vane, and 3D Auto assumptions can be tested against real command and feedback frames.

The current candidate fields are:

```text
Vertical vane:
  DB0 bits 7:6      mask 0xC0
  DB1 bits 7,5:4    mask 0xB0

Horizontal vane:
  DB16 bits 2:0     mask 0x07
  DB17 bit 0        mask 0x01, currently treated as horizontal swing

3D Auto:
  DB17 bit 2        mask 0x04

Current extended-louver command indicators:
  DB17 bits 3 and 1 mask 0x0A
```

These are hypotheses derived from the current decoder and TX builder. The trace logs the complete DB0, DB1, DB16, and DB17 bytes, their binary representation, and XOR changes so other changing bits remain visible.

## Changed files

```text
components/MhiAcCtrl/mhi_louver_diag.h
components/MhiAcCtrl/mhi_status_decoder.cpp
components/MhiAcCtrl/mhi_tx_builder.cpp
```

Additional test-support files:

```text
examples/mhi_louver_diag_marker.yaml
home_assistant/mhi_louver_diag_full_matrix.yaml
```

## Logging behaviour

The logger tag is:

```text
mhi.louver_diag
```

A TX entry is emitted for every command frame containing at least one of:

```text
vertical vane
horizontal vane
3D Auto
```

An RX entry is emitted for the first eligible 33-byte feedback frame and whenever any of these raw inputs changes:

```text
DB0 & 0xC0
DB1 & 0xB0
full DB16
full DB17
```

Normal power, mode, temperature, and fan changes therefore do not generate louver feedback noise unless they also alter one of the candidate louver fields.

Representative output:

```text
[mhi.louver_diag]: MARK CASE_START V1_UP__H3_CENTER__D1_ON ...
[mhi.louver_diag]: TX command mask=0x00000040 db0=... db1=... db16=... db17=...
[mhi.louver_diag]: TX candidates vertical{...} horizontal{...} 3d{...}
[mhi.louver_diag]: RX feedback db9=... db0=... db1=... db16=... db17=... xor=...
[mhi.louver_diag]: RX candidates vertical{...} horizontal{...} 3d{...}
[mhi.louver_diag]: MARK CASE_RESULT V1_UP__H3_CENTER__D1_ON observed{...}
```

## ESPHome marker action

Merge `examples/mhi_louver_diag_marker.yaml` into the device configuration. It adds a Home Assistant action named from the ESPHome node:

```text
esphome.<node_name>_louver_diag_mark
```

The marker action writes the Home Assistant test-case label into the same ESPHome log stream as the TX and RX trace.

The existing `api:` and `logger:` blocks can be merged; they do not need to be replaced.

## Home Assistant matrix script

Add `home_assistant/mhi_louver_diag_full_matrix.yaml` to `scripts.yaml`, or copy its script body through the Home Assistant YAML script editor.

When starting the script, provide:

```text
vertical_entity    MHI vertical-vane select
horizontal_entity  MHI horizontal-vane select
three_d_entity     MHI 3D Auto switch
marker_action      esphome.<node_name>_louver_diag_mark
```

The script covers the complete public control matrix:

```text
5 vertical states
x 8 horizontal states
x 2 3D Auto states
= 80 cases
```

Default timing:

```text
1 second after vertical command
1 second after horizontal command
15 seconds after 3D command
```

The default run takes approximately 23 minutes. Increase the settle time if the returned state is still moving between markers.

## Capture procedure

1. Flash the spike build.
2. Confirm DEBUG logging and the `mhi.louver_diag` tag are enabled.
3. Open the ESPHome live log.
4. Run the Home Assistant matrix script.
5. Save the complete log from `BEGIN full_matrix` through `END full_matrix`.
6. Do not change vane or 3D controls manually during the run.

## What the trace should answer

For each requested control state, the resulting log should show:

```text
which DB0/DB1 bits changed for vertical vane;
which DB16/DB17 bits changed for horizontal vane;
whether DB17 bit 2 tracks 3D Auto persistently;
whether 3D Auto changes the returned horizontal state;
whether horizontal commands clear or preserve the returned 3D state;
whether any currently ignored DB16/DB17 bits correlate with a state;
whether command bytes and returned feedback bytes use the same representation;
how many intermediate feedback states occur before the final HA state.
```

No confirmation timeout or retry changes are included in this spike. That keeps the capture focused on discovering the protocol representation before changing matching policy.
