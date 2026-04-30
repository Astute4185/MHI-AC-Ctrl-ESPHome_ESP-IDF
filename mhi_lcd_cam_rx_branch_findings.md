# MhiLcdCamRxEngine Branch Findings

Date: 2026-04-30  
Branch context: `MhiLcdCamRxEngine`  
Status: **Keep as reference branch only. Do not merge.**

## Executive decision

The `lcd_cam_rx` backend is technically valid as a clean receive path, but it does **not** deliver the expected runtime/load improvement over the existing GPIO transport.

The branch should not be merged or maintained as a production option. It is useful as a research/reference branch for future asynchronous receive work.

## Original objective

The goal was to evaluate an ESP32-S3 external-clock receive backend using an I2S/LCD-CAM style sampler.

The intended benefit was:

- improve receive stability compared with GPIO/bit-banged sampling
- reduce timing sensitivity
- reduce CPU / main-loop blocking cost
- keep the public API simple with one optional selector:

```yaml
MhiAcCtrl:
  id: mhi_ac_ctrl
  transport_backend: gpio
```

or:

```yaml
MhiAcCtrl:
  id: mhi_ac_ctrl
  transport_backend: lcd_cam_rx
```

The intended default was:

```yaml
transport_backend: gpio
```

## What was proven

### 1. Backend selection works

The component can select between:

- `gpio`
- `lcd_cam_rx`

The default path remains GPIO when `transport_backend` is omitted.

### 2. `lcd_cam_rx` can produce clean accepted frames

The LCD-CAM backend successfully produced clean 33-byte published frames on the ESP32-S3 test target.

Representative LCD-CAM runtime result:

```text
ok=2194
ok20=0
ok33=2194
short=0
sig=0
basechk=0
extchk=0
mismatch_top=none
lcdcam_calls=585
```

This proves the backend is not just a scaffold. It can capture and publish valid extended frames.

### 3. Internal gates were effective

The following internal protections were useful and should be kept in mind for any future design:

- extension-gap gate
- publish gate
- resync cooldown / partial-frame suppression

These allowed the backend to avoid feeding bad frames into the normal validator/decode path.

### 4. Validator contract stayed clean

The working model remained:

- publish clean frames, or
- publish nothing

Backend-specific recovery did not need to be pushed into normal validator-side decode logic.

### 5. ESPHome build compatibility was proven

The branch compiled and flashed on the test ESP32-S3 target using ESPHome 2026.3.3 / ESP-IDF 5.5.3.

## What failed

### 1. No meaningful performance improvement

The main reason to pursue `lcd_cam_rx` was to reduce blocking / CPU cost versus GPIO.

That did not happen.

Representative GPIO result:

```text
gpio_wall_avg_us=49736
gpio_work_avg_us=45278
gpio_work_busy=89.64%
```

Representative LCD-CAM result:

```text
lcdcam_wall_avg_us=50731
lcdcam_work_avg_us=45156
lcdcam_work_busy=87.59%
```

A later LCD-CAM sample was similar:

```text
lcdcam_wall_avg_us=51079
lcdcam_work_avg_us=45423
lcdcam_work_busy=88.57%
```

Practical conclusion: LCD-CAM is approximately the same blocking cost as GPIO in this implementation.

### 2. ESPHome still reports long blocking operations

Both backends still trigger ESPHome long-operation warnings.

GPIO example:

```text
MhiAcCtrl took a long time for an operation (85 ms), max is 30 ms
```

LCD-CAM example:

```text
MhiAcCtrl took a long time for an operation (99 ms), max is 30 ms
```

This means the implementation still monopolizes the component loop for too long.

### 3. The implementation is not truly asynchronous

The current LCD-CAM path still behaves like a blocking transport exchange. It may use hardware-assisted capture internally, but the component-level behaviour is still dominated by the MHI frame cadence / wait window.

The practical effect is:

- ESPHome loop is still blocked
- wall time is still around 50 ms per exchange
- measured work time is still around 45 ms per exchange
- no maintainable runtime win over GPIO

### 4. Maintenance cost is not justified

The LCD-CAM backend adds:

- ESP32-S3-specific logic
- peripheral-specific implementation complexity
- extra diagnostics and branch surface
- more code paths to test and maintain

Because it does not reduce blocking or CPU cost, that maintenance burden is not worth taking into the main branch.

## Decision

Do **not** merge `MhiLcdCamRxEngine`.

Keep the branch only as:

- reference implementation
- proof that LCD-CAM-style receive can produce clean frames
- source material for future asynchronous transport work

The stable/default implementation should remain the GPIO transport.

## Recommended branch disposition

Leave the branch available but mark it clearly as non-production / not planned for merge.

Suggested branch note:

> `MhiLcdCamRxEngine` demonstrated that an ESP32-S3 LCD-CAM-style receive backend can produce clean accepted 33-byte frames, but it did not reduce ESPHome loop blocking or transport work time compared with the existing GPIO backend. The branch is retained for reference only and should not be merged without a redesigned asynchronous receive architecture.

## What to keep from this work

If this idea is revisited later, preserve these learnings:

1. **LCD-CAM receive is viable for clean frame capture**
   - The backend can publish valid `ok33` frames.
   - The internal gates are effective.

2. **The public config model was good**
   - One selector is enough:
     - `transport_backend: gpio`
     - `transport_backend: lcd_cam_rx`
   - GPIO should remain the default.
   - Avoid exposing backend tuning knobs unless absolutely necessary.

3. **The validator boundary should stay clean**
   - Backend should publish clean frames or nothing.
   - Do not move backend-specific rescue logic into the validator.

4. **Performance must be judged by component blocking behaviour**
   - Clean capture alone is not enough.
   - If ESPHome still reports long operation warnings, the architecture has not solved the real runtime problem.

## What a future attempt would need

A future LCD-CAM/I2S/RMT receive path should not be a blocking exchange replacement.

It would need a genuinely asynchronous architecture:

- hardware/DMA/ISR capture into a ring buffer
- short `loop()` calls
- frame sync performed incrementally
- completed frames published later
- no full-frame blocking wait inside the component loop
- no long ESPHome operation warnings
- clear separation between RX capture, frame sync, TX timing, and decode/publish

Without that architecture change, the LCD-CAM backend is just a cleaner capture path with roughly the same blocking profile as GPIO.

## Final summary

`lcd_cam_rx` worked functionally but failed the business case.

It proved:

- external-clock capture can work
- clean 33-byte frames are possible
- backend gating can protect the validator

It did **not** prove:

- lower CPU cost
- lower ESPHome blocking time
- enough benefit to justify maintenance

Final decision: **do not merge; keep branch for reference only.**
