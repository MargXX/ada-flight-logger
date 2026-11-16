# ada-flight-logger

Flight data logger firmware for the Brown Rocketry payload, written in Ada for the Raspberry Pi Pico (RP2040). It reads an LSM9DS1 IMU and a BMP390 barometer over I2C and logs timestamped samples to W25Q128 SPI flash, raising its logging rate when it detects launch.

This is an improved version of [`terrarium_pico`](https://github.com/Brown-Rocketry/IREC-2025-2026/tree/main/terrarium_pico), the firmware flown in April 2026, on the same board and wiring. That firmware had no launch awareness, so it had to log continuously at a low rate to hold about 21 hours of pre-flight data on the 16 MB chip, and a chip that size can't sustain more than about 8 Hz for that long. As a result, only about 222 samples covered the 45.2 second flight. This version keeps the same simple single-loop design and fixes what the recovered data showed was holding it back.

**Status:** flown in November 2026. The flight log was recovered with no sequence gaps, 0 missed deadlines and 0 errors. Results are in [First flight](#first-flight-november-2026).

---

## Improvements over `terrarium_pico`

The sample counts below are for a flight of the same 45.2 s length. This firmware's first flight was shorter, about 37.5 s, and logged 3,748 samples.

| Area | `terrarium_pico` | This firmware | Why it matters |
| --- | --- | --- | --- |
| Logging rate | Steady 4.9 Hz | 10 Hz on the ground, 100 Hz after launch | About 4,500 samples over a flight of the same length, instead of about 222, without filling the flash on the pad |
| Launch awareness | None | Detects launch from the accelerometer | The rate rises when the data matters most |
| Timing | Flat 200 ms delay added to the work | Fixed 10 ms deadline, with misses counted | Variation in the work can't build up as drift, and missed deadlines show in the log |
| Flash erases | Sector erase during logging, about 261 ms instead of the usual 204 ms | None, the chip is erased before the campaign | Erases can't interrupt boost |
| UART | Message on every page while logging | None in the logger, only in the bench tools | Debug output can't slow down logging |
| Samples | 28 bytes with magnetometer, 9 per page | 22 bytes without magnetometer, 11 per page | About 22% more samples on the chip, 720,896 instead of 589,824 |
| Log contents | Samples and a sequence number | Also missed deadlines, errors, rate and format version on every page | A recovered log shows how the firmware performed |
| Barometer setup | Mostly reset defaults, with an output rate too fast for the oversampling | Set explicitly, and rejected settings are caught at boot | Measurements have enough time to finish |
| Reading data back | Decimal text, edited per run | Every page as raw hex | Nothing is left out, and the data can be decoded afterward |
| Code | One 672-line file | Separate device drivers with enum-selected modes, plus four small programs | Easier to read, and launch detection and packing have host tests |

### Launch detection

Every 10 ms tick reads the accelerometer. Launch is declared after 10 consecutive readings (100 ms) above 3 g.

In the April 2026 log, boost was the only sustained stretch above 3 g in 25 hours, though that log only sampled at about 5 Hz. Boost also looked short, with about 0.8 s above 1.5 g, which is why the accelerometer is checked on every tick rather than at the 10 Hz logging rate.

Flight rate lasts 10 minutes, then the logger drops back to ground rate and re-arms. Each minute at flight rate uses about 140 KB, 0.8% of the chip, so a false trigger on the pad costs about 8% of the flash and can't stop a real launch from being detected.

---

## First flight, November 2026

The logger sat on the pad for 6.1 hours at ground rate, detected launch with no false triggers, and logged 10 minutes at flight rate. The whole log used 20,212 pages, 31% of the chip.

| Measurement | Value | Source |
| --- | --- | --- |
| Apogee | 358 m (1,173 ft) at 8.0 s | Barometer |
| Peak boost acceleration | 18.2 g measured, 17.1 g vertical with gravity removed. Past the rated ±16 g, see below | Accelerometer, gyro |
| Burn time | About 0.7 s | Accelerometer |
| Max vertical speed | 101 m/s (331 ft/s) at burnout | Accelerometer and gyro, integrated |
| Ejection | 10.2 s, about 2.2 s after apogee | Accelerometer |
| Descent under parachute | 11.7 m/s | Barometer |
| Landing | About 37.5 s, at least 36.8 g with the X axis clipped | Accelerometer |
| Roll rate in boost and coast | About 85-120 deg/s | Gyro |
| Nose tilt | Under 10 deg until 7 s, then about 60 deg by ejection | Accelerometer and gyro |
| Flight samples, liftoff to landing | 3,748 | Log |
| Missed deadlines, errors, sequence gaps | 0, 0, 0 | Log |

- **Scale check:** gravity on the pad read 1,353 counts, 0.990 g at 0.732 mg/LSB, confirming the "±16 g" setting spans about ±24 g.
- **Rated range:** the datasheet only specifies the accelerometer within ±16 g, so readings from 16 g to 24 g are recorded but not guaranteed linear. 42 samples went past 16 g on some axis: the first 0.39 s of boost (X up to 18.1 g), ejection (X 23.5 g), the second deployment at 15.5 s (Y 18.1 g) and landing. The boost peak, and the IMU speed and altitude integrated through it, carry that uncertainty. The charts shade these samples.
- **Timing:** every gap at flight rate was exactly 10 ms.
- **IMU vs barometer:** integrating vertical acceleration twice gave a 328 m apogee, 30 m under the barometer. Their vertical speeds agree from about 2 s to 7 s.

---

## Programs

| Program | Purpose |
| --- | --- |
| `flight_logger` | The flight firmware. No UART. |
| `flight_reader` | Sends each written page over UART as one line of hex, with nothing else on the line |
| `sensor_test` | Prints sensor readings and the barometer calibration over UART |
| `flash_erase` | Erases the whole chip after `E` is sent over UART to confirm |
| `flash_dump_swd` | Copies flash into RAM on request so `tools/swd_dump.sh` can read it over SWD. No UART. |

## Source layout

| Path | Contents |
| --- | --- |
| `src/*.adb` | The programs above, each with its own clock, pin and I2C setup |
| `src/swd_mailbox` | Shared RAM between `flash_dump_swd` and the debugger |
| `src/drivers/` | `lsm9ds1`, `bmp390` and `w25q128`, with modes selected by enum |
| `src/core/` | `launch_detect` and `sample_log`, which have no hardware access |
| `src/debug_uart` | UART for the bench tools only |
| `tests/` | Host tests for `src/core` |
| `tools/swd_dump.sh` | Reads the flash over SWD into a raw image |
| `tools/plot_flight.py` | Charts altitude and acceleration from a log, and the raw gyro with `--raw` |
| `tools/plot_attitude.py` | Combines gyro and accelerometer to chart vertical speed, ground-frame acceleration and nose tilt |

---

## Using it

### Before each campaign

1. Save anything on the chip with `flight_reader`.
2. Run `flash_erase` and send `E`. This is required: the logger never erases, and it shows a boot fault if the chip holds data in another format.
3. Run `sensor_test` and save the calibration line. The log does not store it, and raw barometer values can't be converted without it.
4. Flash `flight_logger`.

### LED on `flight_logger`

| LED | Meaning |
| --- | --- |
| Changes about every 1.1 s | Logging at ground rate |
| Changes about every 0.11 s | Logging at flight rate |
| Fast blink | Boot fault: flash or a sensor isn't responding, or the chip needs erasing. Keeps retrying. |
| Solid on | Flash full, logging stopped |

A brownout restart resumes at the first blank page, so existing data is kept. The partly filled page in RAM, up to 11 samples, is lost.

### Reading data back

Over SWD, preferred. Reads the whole chip in about 3.5 minutes into a raw image, 256 bytes per page. Pass a page count to read less, for example `4096` for a short bench log.

```bash
tools/swd_dump.sh flight.bin
```

Over UART: flash `flight_reader` and start a capture at 115200 baud within 5 seconds of power-up. The LED goes solid when it finishes. A full chip takes about 49 minutes. On a Mac, long UART captures through the Debug Probe have dropped or garbled stretches, so check the page sequence numbers or use SWD.

```bash
screen -L -Logfile flight.hex /dev/ttyUSBx 115200
```

### Plotting

Needs [uv](https://docs.astral.sh/uv/), which installs numpy and matplotlib on the first run. Both scripts take a `.bin` from `swd_dump.sh` or a `.hex` from `flight_reader`, plus the barometer calibration line from `sensor_test`, either as hex or as a file holding it. Each writes a PNG next to the log.

```bash
uv run tools/plot_flight.py flight.bin --calib calibration.txt          # altitude and acceleration
uv run tools/plot_flight.py flight.bin --calib calibration.txt --raw    # unfiltered, with gyro
uv run tools/plot_attitude.py flight.bin --calib calibration.txt       # gyro and accel combined
```

Add `--show` to open a window to zoom in, or `--after SECONDS` to set how much of the flight to show.

---

## Logging layout

All values are LSB first. Each line from `flight_reader` is one 256-byte page.

### Sample, 22 bytes

| Offset | Size | Field | Notes |
| --- | --- | --- | --- |
| 0-3 | 4 | Timestamp | Microseconds since boot, wraps about every 71 minutes |
| 4-9 | 6 | Accel X, Y, Z | Signed 16-bit raw, 0.732 mg/LSB at the "±16 g" setting, which spans about ±24 g |
| 10-15 | 6 | Gyro X, Y, Z | Signed 16-bit raw, 70 mdps/LSB at ±2000 dps |
| 16-18 | 3 | Pressure | BMP390 raw 24-bit |
| 19-21 | 3 | Temperature | BMP390 raw 24-bit |

### Page, 256 bytes

| Offset | Size | Field |
| --- | --- | --- |
| 0-241 | 242 | 11 samples |
| 242-245 | 4 | Missed tick deadlines since boot |
| 246-247 | 2 | Failed sensor reads or flash writes since boot, stops at the maximum |
| 248 | 1 | Rate when the page was written: 0 ground, 1 flight |
| 249-250 | 2 | Reserved, zero |
| 251 | 1 | Format version, currently `0xA2` |
| 252-255 | 4 | Page sequence number. All `0xFF` means never written. |

A failed sensor read logs zeros for that sensor and adds to the error count.

---

## Capacity

| Rate | Time to fill an empty chip |
| --- | --- |
| 10 Hz ground | About 20 hours, or about 18.4 hours after one 10-minute flight |
| 100 Hz flight | About 2 hours |

The first flight's log ended at about 25 hours, likely from the batteries, so power and flash now run out on similar timescales.

---

## Known limits

- **Early boost is slightly under-sampled.** There is no pre-launch buffer, so the 100 ms before detection is logged at 10 Hz.
- **Barometer data repeats at flight rate.** It updates at 50 Hz while samples are logged at 100 Hz.
- **Timestamps wrap every 71 minutes.** The plotting scripts unwrap them.
- **Boost exceeds the accelerometer's rated range.** The "±16 g" setting reads to about ±24 g, but linearity is only specified to ±16 g, and boost reached 18.1 g on X. The LSM9DS1 has no higher range, so a higher-g accelerometer would be needed to stay in spec.
- **Landing can clip the accelerometer.** The first flight's landing pinned the X axis at the ±24 g limit for two samples.
- **The barometer misreads in boost and at ejection.** Air moving fast past the sensor during boost made it jump to about 125 m by 1 s, well above the true altitude, and the ejection charge put a spike of about 2,000 m in the reading. `plot_flight.py` uses a 0.5 s median filter so neither counts as apogee.
- **Gyro tracking drifts.** Tilt and IMU altitude are most reliable early in the flight. `plot_attitude.py` stops integrating altitude at ejection.
- **The logger doesn't check that the rest of the chip is blank.** It only checks page 0 and looks for the first blank page, so skipping `flash_erase` after other tools have written to the chip could corrupt data.

---

## Open questions

- [x] Do the 3 g and 100 ms trigger settings hold up against real handling? Yes for the first flight. There were no false triggers over 6.1 hours on the pad, and launch was detected about 0.04 s after the first logged reading above 1.5 g.
- [x] Does a tick fit in 10 ms on hardware? Yes. A 9-minute bench log, including 3 minutes at flight rate, had 0 missed deadlines and every gap exactly 10,000 or 100,000 µs.
- [x] Are the 1 ms chip select delays safe to remove? Yes on the bench. The same log wrote 2,002 pages with 1 µs delays, 0 flash errors and no sequence gaps.
- [x] Is 20 hours at ground rate enough for the pad wait? Yes for the first flight. The pad wait was 6.1 hours, and the full log used 31% of the chip.

---

## Testing

Bench results from November 2026, on the flight PCB.

- [x] Run the host tests: `cd tests && alr build && ./bin/host_tests`.
- [x] Run `sensor_test`. Every setup line says `ok`, and the flash ID matches.
- [x] Check the accel scale against gravity. At rest the magnitude averaged 1,314 counts, or 0.96 g at the datasheet's 0.732 mg/LSB, within the sensor's tolerance. The `FS_XL = 01` setting is called ±16 g, but at that sensitivity the full count range is about ±24 g. The 1366 counts/g used for the trigger is correct. A ±16 g span would have read 2,048 counts at rest.
- [x] Bench run: 2,002 pages with sequence 0 to 2001, format `A2`, 0 missed deadlines, 0 errors.
- [x] Shake test: launch triggered 200 ms after the first logged reading above 3 g, and 100 Hz spacing started there. The shake peaked at 15.8 g with no axis clipping.
- [x] Flight rate lasted exactly 30,000 ticks (299.99 s) with the earlier 5-minute setting, then every gap returned to exactly 100 ms. Confirmed with 10 minutes on the first flight, which logged 600.0 s at flight rate.
- [x] Barometer restart: one 9-minute run logged a single frozen pressure and temperature value with no errors. The BMP390 keeps power across a Pico reset and had ignored the mode change. Setup now soft resets it, reads the mode back and waits for a new result. After 6 resets in a row, starting from the frozen state, every boot logged live values.
- [ ] Measure a tick with a spare GPIO and a logic analyzer.

---

## Build and flash

Requires [Alire](https://alire.ada.dev/), which fetches the ARM compiler and dependencies on the first build, plus OpenOCD and a CMSIS-DAP probe for SWD. Hardware and wiring are unchanged: see the [`terrarium_pico` README](https://github.com/Brown-Rocketry/IREC-2025-2026/blob/main/terrarium_pico/README.md) for the pinout. The UART tools need GP8 (TX) and GP9 (RX) connected.

```bash
alr build --release
```

Replace `flight_logger` with the program you want:

```bash
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
  -c "adapter speed 5000" \
  -c "program bin/flight_logger verify reset exit"
```

---

## License

MIT. See [LICENSE](LICENSE).

---

Developed with AI assistance.
