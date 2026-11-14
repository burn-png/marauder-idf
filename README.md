# Dominion ESP-SDR firmware

An original ESP-IDF firmware project for the **Esp32 Marauder V7 by JustCallMeKoko** hardware (developed and tested on a **V7.2** board; V7.2 is the battery-polarity hardware revision). Dominion keeps the ESP-SDR receive snapshot engine and builds a compact spectrum instrument around it: a CP437 interface, fixed-point FFT, scrolling Magma waterfall, passive 2.4 GHz scan, SD IQ8 capture, and offline IQ playback through the same FFT/display path.

The ESP-SDR receiver obtains 256 complex samples per RF snapshot at a nominal 16 MS/s; the task waits between snapshots. This is not a continuous stream, a calibrated measurement receiver, or arbitrary-frequency SDR coverage. The original ESP32 Marauder V7 accepts direct-PLL receive-tune requests from 1842 to 2800 MHz. Frequencies above 2484 MHz (the board's 2.4 GHz band edge) are shown in red as unverified/out-of-band requests; requests above 2651 MHz also exceed the local LO helper's conservative PLL envelope. The 2484 MHz entry in the passive channel-scan list is Wi-Fi channel 14; it is separate from manual tuning. Accepting a tune request does not guarantee lock, sensitivity, or calibrated reception. Requested span/zoom changes only the display projection inside the captured slice; the actual nominal sample rate is 16 MS/s, so a 54 MHz displayed span would not be a valid sampled bandwidth. SSTV decoding is not included: the sample rate exceeds the 1 MS/s cutoff requested for adding it, and the receiver captures sparse snapshots rather than continuous audio samples. A separate lab-only test action can ask Espressif's PHY certification API to send one fixed 802.11 test frame; it is not an SDR waveform or replay transmitter.

## Board map

The TFT setup matches the Marauder v7 `User_Setup_dual_nrf24.h` reference:

| Function | Marauder v7 connection |
| --- | --- |
| Shared SPI MOSI / SCLK / MISO | GPIO 23 / 18 / 19 |
| ILI9341 CS / DC | GPIO 17 / 26 |
| TFT reset | Board reset (no ESP GPIO pulse) |
| Backlight | GPIO 32 |
| TFT MISO / touch controller | Not connected / touch disabled; shared MISO is for SD |
| SPI SD CS | GPIO 4 (SPI2 shared with display) |
| GPS module TX -> ESP RX / module RX <- ESP TX (UART2) | GPIO 21 / 22 at 9600 baud |
| Joystick Up / Down / Left / Right / Center | GPIO 36 / 35 / 13 / 39 / 34 (active-low) |
| Status LED | GPIO 15 |

The V7 switches are scanned by one shared debounced input manager. GPIO 34, 35, 36, and 39 are input-only ESP32 pads and do not have internal pull resistors; the input setup leaves those pads in input mode and relies on the V7 board's switch biasing. GPIO 13 uses the internal pull-up. The upstream firmware marks all five switches active-low with pull-up behavior.

The V7 pin selection follows JustCallMeKoko's [`configs.h`](https://github.com/justcallmekoko/ESP32Marauder/blob/develop/esp32_marauder/configs.h) and GPS UART argument order in [`GpsInterface.cpp`](https://github.com/justcallmekoko/ESP32Marauder/blob/develop/esp32_marauder/GpsInterface.cpp). The upstream macros name GPIO21 `GPS_TX`, but pass it as the MCU UART receive pin; GPIO22 is passed as MCU UART transmit.

The user-supplied photo showed garbled text over a bright, noisy waterfall. The text fault was traced to an incomplete glyph table. The project now includes a full 256-glyph 8x8 CP437 bitmap, rendered with the correct bit order. The spectrum trace region occupies y=36–63, frequency labels sit at y=66, and waterfall scroll begins at y=76. The live spectrum uses a compact connected trace with red peak markers; its plot coordinates are local to the graph region. Display contrast expands low-level FFT bins without changing stored IQ data. The display clear path normalizes the ILI9341 scroll origin before menus are redrawn, and each waterfall row is written against its new origin to avoid stale rows at wraparound. Menu selection, text fields, headers, footers, and status fields use changed-region updates. The waterfall uses the Magma LUT and its saved floor/ceiling. The provided photo is a reference, not a post-change hardware test.

## Features

- Receiver groups live spectrum, frequency/gain controls, DSP, the Level signal meter, Looking Glass, passive channel scan, and the Rolling RSSI Detector. The detector plots a rolling 200-sample history of peak FFT-bin level at the selected center frequency; LEFT/RIGHT tunes, UP/DOWN adjusts the trigger level, and CENTER toggles detection. A threshold crossing is shown as DETECTED and red graph trace. This is relative FFT level, not calibrated RSSI or dBm. Live spectrum and waterfall use Magma, Inferno, and grayscale palettes; the graph shows center frequency plus the left and right frequencies at the plotted span edges. The connected trace uses a 24-pixel plot range with red peak markers and suppresses the baseline for empty bins. Receive > DSP and trace controls includes a persistent TRACE AUTO FIT option (default on) that scales the trace to robust 5th–98th-percentile levels and smooths scale changes between frames; turn it off to use the configured manual trace floor/ceiling. The waterfall separately preserves its own floor/ceiling and low-level fallback. The DSP panel provides OFF, 3/5/7-bin mean, 5/7-bin triangular, 5-bin Gaussian, and 3/5-bin median smoothing, temporal averaging (1/2/4/8 frames), rectangular/Hann windowing, and peak hold. Display zoom, tune step, independent waterfall and spectrum trace floor/ceiling controls, and pause controls remain available.
- Looking Glass starts with a responsive 2412–2484 MHz scan range; MIN/MAX can be adjusted across the accepted 1842–2800 MHz request range, and RANGE LOCK preserves width when tuning either bound. The sweep skips past frequencies that fail to capture rather than stalling, and supports 4/8/16/32 MHz steps, 1/2/4/8 capture dwell, and WATERFALL, LIVE-V, or PEAK-V display modes. The quick panel also exposes shared AGC/manual gain, display span (1/2/4/8/16 MHz), and the bin smoothing filter; focus SWEEP and press Center to pause/resume. Very wide scans take longer to refresh. Spectrum resolution remains fixed by the receiver's 256-sample snapshots; this hardware has no audio output for the PortaPack squelch beep or audio jump features. Frequencies above 2484 MHz remain unverified/out of band and above 2651 MHz exceed the conservative PLL envelope.
- Passive channel sweep across the 14 2.4 GHz Wi-Fi channel centers; high-level detections can be logged to `RECON.CSV`. This does not decode packets or perform active probing.
- Receiver > Level (Signal Meter) tunes a selected frequency and displays smoothed relative FFT-bin level and strongest-bin offset while the operator compares readings by manually rotating a directional antenna. It provides visual feedback only: it does not transmit, decode packets, produce calibrated dBm, or determine bearing; optional GPS is the receiver's position only.
- IQ8 snapshot capture to `RX####.C8`, with a companion `.META` file for start/end UTC and receiver coordinates when available, boot-relative time, tuned frequency, nominal sample clock, format, sample count, elapsed duration, altitude, and fix quality. Position metadata describes the receiver's location, not the origin of a received signal. Capture data consists of sparse 256-sample snapshots separated by acquisition gaps; it is not contiguous time-series IQ. Offline replay concatenates those stored snapshots for spectral inspection and does not recreate the gaps or original elapsed time. `WARDRIVE.CSV` records valid GPS positions while recording.
- Offline IQ8 replay feeds samples into the local FFT and waterfall only. Playback has next-file, play/pause, progress, and loop behavior. It has no RF output path. SDR++ live-source support is not included: these files contain sparse IQ snapshots with acquisition gaps, not a continuous SDR++ stream.
- RF Replay is a separate compatibility application. It inspects IQ format, sample rate, tuned frequency, and contiguous-versus-sparse capture metadata, distinguishes offline FFT playback from RF replay, and can save a per-capture `.RRC` configuration. The one-packet bench test does not enable captured-IQ replay; saved replay configurations still record `TRANSMIT_ALLOWED=0`.
- TX bench test sends one fixed 32-byte 802.11 PHY test frame on channel 1 at 1 Mb/s, with the SDK's maximum 31.75 dB power backoff. Two Center presses are required. It has no user payload, arbitrary waveform, repeat, replay, or continuous mode. Use only in a shielded or conducted lab setup; this test path has not been measured on a physical Marauder V7.
- GPS dashboard parses checksummed, talker-independent RMC and GGA sentences, validates coordinates, and shows valid/stale state, coordinates, UTC, altitude, speed, course, quality, satellite count, HDOP, update rate, parser counters, and UART errors only when reported by the module. GPS tracks can be named, started, paused, resumed, stopped, browsed, and reviewed point-by-point as versioned CSV files under `GPS/TRACKS/`; saved positions can be browsed and deleted under `GPS/LOCATIONS.CSV`.
- Capture browser with file size, metadata preview, and paired IQ/metadata deletion; file manager protects non-capture files from deletion.
- IQ Trim scans IQ8 blocks for silence and writes a trimmed copy; it never changes the source capture.
- RF profile manager saves, loads, duplicates, and deletes versioned text settings files under `RFPROFILES/`. Profiles contain receiver/spectrum settings and remain separate from captures. Files can be imported/exported by copying `.RFP` files to/from that directory on the SD card.
- SD manifest discovery reads `APPS/*.APP` metadata and seeds an `ANALOG_TV.APP` metadata-only demo if absent. It does not load arbitrary code: there is no external-app runtime or ABI loader, and the analog-TV manifest does not add video hardware or decoding.
- SD Media Viewer in Utilities searches the SD card root and subdirectories (up to four levels) for `.JPG`, `.JPEG`, `.PNG`, and `.MP4` files. Use an MBR-partitioned FAT16/FAT32 card; exFAT is unsupported by this build. Paths can be up to 63 characters and filenames can include letters, numbers, spaces, `_`, `-`, `.`, and parentheses. Opening the viewer retries a failed boot-time mount. Baseline JPEG and non-interlaced PNG images are decoded on a temporary larger-stack task and streamed/scaled to fit the 240×260 image area without a full-frame framebuffer; transparent PNG pixels are composited onto black. JPEGs beyond the decoder's 1/8 scale limit are rejected with an on-screen message rather than cropped. MP4 files are recognized but not played: this original ESP32/V7 build has no supported MP4/H.264 video decoder/render pipeline; progressive reading cannot replace a missing codec or frame memory.
- Utilities > Storage Diagnostics shows mount stage/error, OS errno, root-entry and media counts, example filenames, and read-only presence checks for the firmware's seeded `APPS/ANALOG_TV.APP` and `WARDRIVE.CSV`, to distinguish a readable but unexpected/empty volume from an SD/FAT mount failure or listing problem. Root-directory scans resolve directly to `/sdcard`. It never writes a diagnostic probe or formats the card. GPS track summaries and graphs read CSVs as streams rather than reopening the file for every small block.
- Receiver controls provide AGC or manual gain (index 0–127, saved in NVS) and a software DC-bin mode that either shows the center bin as sampled or interpolates it from its neighbors. LO offset is explicitly unavailable. The original ESP32 capture clock is fixed at a nominal 16 MS/s; the selectable sample rates shown by other ESP-SDR front ends are not supported by this backend. The FFT uses 256 points and captures sparse 8+8-bit IQ snapshots. Joystick input settings include repeat delay/rate, debounce interval, long-press duration, repeat acceleration, and vertical/horizontal inversion. Menus use Up/Down and Center; Left backs out. Spectrum tuning uses Left/Right with accelerated repeat and Up/Down to zoom the display span.
- Directional presses are scanned on a 10 ms UI cadence with a 12 ms default debounce; the first movement is a press event and does not wait for hold-repeat. The status bar updates only changed values and shows the app, internal-heap use, `SD: OK`/`SD: NG`, battery gauge data when available, and GPS-fix state. The debug page reports PSRAM separately when present.
- Settings opens directly from Home, with Back at the top of its list. Unavailable TX, transceiver, remote, and HackRF placeholder pages have been removed from navigation. Games and their reaction timer code have been removed.
- Like the V7 firmware, the status bar reads the optional MAX17048's state-of-charge register every three seconds over I2C (SDA GPIO33, SCL GPIO16) and displays its integer percentage. If the gauge is absent or unreadable it shows `BAT:--`; bus/device setup is retried on later updates. No battery percentage is guessed from voltage, and no charging state is fabricated.
- Joystick-operated notepad: Up/Down cycles A–Z, backspace, and space; Center appends the selected character to `NOTES.TXT`.
- Frequency manager, joystick-operated arithmetic calculator, antenna length calculator (quarter/half/5/8 wave), stopwatch, hardware-random 16-hex-character password generator, and detailed runtime diagnostics for radio capture success/errors and last error, FFT frame/peak/floor coverage, tuning/gain/mode, queue depth, internal/DMA/PSRAM heap, task stack watermarks, SD mount state, GPS fix age, and NMEA counters.
- NVS preferences, schema validation, legacy preference migration, and defaults recovery.

## Controls

The authentic V7 controls are the five-way tactile joystick, not a single BOOT button. Up/Down navigates, Center confirms, and Left backs out. The initial direction press acts immediately after debounce; a sustained hold repeats after the configured delay and accelerates. In Spectrum and Frequency Setup, Left/Right tunes by the saved frequency step and accelerated repeats; Up/Down changes visible span; Center pauses/resumes where applicable; holding Center exits. Left remains a tuning control rather than an exit in those views. In Looking Glass, any arrow selects the next quick setting while it is unselected; Center selects/deselects it, with a highlight showing the focus/edit state. While selected, all four arrows adjust the value; Center toggles SWEEP or RANGE LOCK immediately when either field is focused, and holding Center exits. The LNA/VGA label refers to the receiver's single shared gain index, not separate hardware controls. In the SD file browser (Utilities > SD file browser, which replaces the separate media viewer and file manager), Up/Down moves through a multi-row folder listing (hold to accelerate and page), Center opens a folder, image or text file, Left goes up one folder (and leaves at the top level), Right shows file info and the delete action. In an image, Left/Right show the previous/next image in the folder and Center returns to the list. In file and profile tools, Up/Down selects entries, Center performs the current action (destructive file removal still requires a second confirmation), and Left backs out. Settings opens from Home directly, with Back first; Up/Down selects a row, Right or Center changes it, and Left returns to Home.

## Build and flash from VS Code

1. Install Espressif's **ESP-IDF** extension and ESP-IDF v6.1 toolchain. Open this project folder in VS Code and wait for the extension to finish configuring.
2. In the ESP-IDF status bar, choose target **esp32**. The project defaults to an 8 MB flash image header for the v7 board's 8 MB flash.
3. Click **Build Project** in the ESP-IDF status bar (or run **ESP-IDF: Build Project** from the Command Palette).
4. Connect the Marauder, choose its serial port in the ESP-IDF status bar (the current local VS Code setting is COM6), then click **Flash Device**. Use the full-flash action only when the extension indicates it is needed.
5. Click **Monitor Device** to view startup logs. Close other browser serial monitors or terminal sessions before connecting to the same COM port.

In an ESP-IDF PowerShell terminal, the equivalent commands are:

```powershell
. 'C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1'
idf.py set-target esp32
idf.py build
idf.py -p COM6 flash monitor
```

Change `COM6` to the port shown by Windows. If PowerShell says `idf.py` is not recognized, open **ESP-IDF PowerShell** from the Start menu or dot-source the installed `Microsoft.v6.1.PowerShell_profile.ps1` first. If an old `sdkconfig` still reports a 2 MB flash size, run `idf.py fullclean`, set the target again, and rebuild so `sdkconfig.defaults` is applied.

An SD card is optional at boot. On the V7 shared SPI bus, MOSI/SCLK/MISO are GPIO 23/18/19 and SD CS is GPIO 4. GPS uses UART2 with the module TX wired to ESP RX GPIO21 and module RX wired to ESP TX GPIO22, matching the upstream V7 `Serial2.begin(..., GPS_TX, GPS_RX)` pin order. If the card does not mount, receive display and menus still start; capture, profiles, notepad, and file operations report storage unavailable. Browse any folder from Utilities > SD file browser: folders first, then names case-insensitively; only one screenful of entries is held in RAM, so folders of any size work. Deleting is limited to user-data types (`.c8`, `.meta`, `.jpg`, `.png`, `.txt`, `.log`, `.csv`, `.gpx`, ...) with a second confirmation; folders and system files are protected. A `.c8` capture's `.META` sidecars are removed with it. Image decoding streams from SD and yields between chunks to keep the UI responsive. The project never formats the SD card automatically. On a successful mount, `APPS/ANALOG_TV.APP` is created only if it does not already exist; it is a metadata demo, not a launchable app.

GPS track files use `MARAUDER_GPS_TRACK=1`, `FORMAT_VERSION=1`, then a CSV header with UTC, decimal-degree latitude/longitude, altitude in meters, speed in m/s, course, fix quality, satellites, and HDOP. A field is `nan` when the receiver did not report it. `GPS/LOCATIONS.CSV` stores name, decimal latitude/longitude, altitude meters, and UTC. Dashboard coordinate display can be set to decimal degrees, degrees/minutes, or degrees/minutes/seconds; units can be metric or imperial. Track viewer distance is labeled as straight-line start-to-end distance, not route length.

An `.APP` under `APPS/` is a text file. A manifest-only file (`MARAUDER_APP=1`, `API=1`, `NAME=`, `CATEGORY=`, `VERSION=`, `DESCRIPTION=`) is listed but cannot run. Adding `RUNTIME=VM1` and a `CODE=<hex>` line makes it runnable: Utilities > SD apps shows `RUNTIME VM1: CENTER = RUN`. VM1 (`main/appvm.cpp`) is a small sandboxed stack machine: an app can compute, read the joystick, wait, and draw rectangles, text and numbers in a 240x256 window. It cannot reach memory outside its own 1280-word array, the SD card, GPIO, the radio or the network; code is limited to 4096 bytes, the stack to 64 entries, and every run slice is step-bounded, so a bad app faults (shown on screen) rather than hanging the device. Hold Center exits. This is not native code loading. `tools/appasm.py` is the assembler and `tools/make_snake.py` builds `sdcard_examples/APPS/SNAKE.APP`; `sh tests/appvm_host/run.sh` tests the machine and the game.

## Transmitter (Fox beacon)

Home > **Transmitter** > **Fox beacon**. The original ESP32 has no DAC replay engine, so the only transmit path
it offers is Espressif's PHY test-mode carrier (`esp_phy_wifi_tx_tone`): one unmodulated carrier. The SDK
documents it for Wi-Fi channels 1-14 only. The beacon lets you pick 1802-2802 MHz on a 5 MHz grid
(2407 + 5 x channel; 2412 and 2437 are Wi-Fi channels 1 and 6) by passing out-of-range channel numbers (negative
ones below 2412 MHz) straight to the SDK. Whether the silicon honours those numbers is untested; if it ignores or
rejects them you will see no carrier outside the Wi-Fi channels. Raw IQ/arbitrary-tone TX is not possible on this chip.

| Row | Meaning |
| --- | --- |
| MODE | STEADY (carrier on until stopped), BURST (random on/off edges), HOP (cycle a frequency set), MORSE (keyed messages) |
| CARRIER | 1802-2802 MHz. LEFT/RIGHT change it by the step size; CENTER on the row cycles the step (5 / 20 / 100 MHz) |
| POWER | 0..32 = -12 to +20 dBm, 1 dB per step. Nominal: the SDK attenuation scale is 0.25 dB units (0 to 31.75 dB below maximum); real output is uncalibrated |
| HOP SET | 1-6-11, 1-4-7-10-13, ALL 13, WIDE 100 MHZ (1802..2802), WIDE 50 MHZ |
| INTERVAL | BURST period, HOP dwell, or MORSE pause between messages |
| DUTY | BURST on-time percentage |
| DURATION | Stop by itself after N minutes (STEADY ignores it and runs until stopped) |
| MESSAGE | Morse messages: from `morse.txt` if present (read-only), otherwise a message you type (CENTER edits) |
| CALLSIGN | From `callsign.txt` if present, otherwise one you type. Never defaulted |
| WPM, ID EVERY, MSG ORDER | Morse speed, CW ID period (STEADY/BURST/HOP, needs a callsign), messages in order or random |
| ACTION | CENTER arms, CENTER again starts; while transmitting, CENTER stops. Holding CENTER leaves the page and stops TX |

**Settings are live.** Changing a row while the beacon is on applies at once (frequency and power re-key the carrier;
during Morse the change lands between elements).

**SD files** (SD card root, re-read whenever the page opens):
- `morse.txt` - messages separated by commas, any number up to 24, e.g. `the quick brown fox jumps over the lazy dog, i love green apples, never eat soggy waffles`. Text is upper-cased and anything Morse cannot send is dropped.
- `callsign.txt` - just the callsign, e.g. `KX88AF`. There is no default callsign.

Settings are saved to NVS when you start or leave. Receive is paused while transmitting and re-tuned when TX ends.
This is untested on hardware: the carrier frequency, power, and the speed of key-up/key-down (Morse and burst edges use the SDK's
tone start/stop calls) have not been measured. Use a cable and a 30-60 dB attenuator, or a licensed callsign ID.

## Advanced DSP (Receiver > DSP)

Passive receive and offline analysis only. Nothing in this section transmits, and the transmit restrictions are unchanged. Every option defaults to the original behaviour, so with the defaults the output is bit-identical to the previous firmware (checked by `tests/dsp_host`). The existing five settings are the first page, **BASIC**. Settings are stored in their own NVS namespace (`dsp_adv`), apply live and persist across reboots; they are written when you leave a group or the page.

**Units.** Bin values are linear FFT magnitudes (0-255), relative only. There is no gain/antenna/ADC calibration, so nothing on these pages is dBm; "dB" means a ratio between two bins or frames. Bin spacing is 62.5 kHz (16 MS/s / 256), the 240 kept bins span about 15 MHz, and reported frequencies follow the on-screen axis.

**What the hardware allows.** The backend delivers sparse 256-sample IQ snapshots, not a continuous stream. So there is no sample-domain filtering, no continuous demodulation and no overlap-add. IQ pre-processing works inside one snapshot (statistics, DC, quadrature, scale). Everything temporal operates on successive frame results and resets on retune; Looking Glass therefore processes each frame on its own.

Pipeline: IQ stats -> optional DC/balance correction -> window -> fixed-point FFT -> **spectrum stage** (DC suppress, adjacent-bin rejection, frequency mask and notches, transient/impulse/outlier rejection, noise-floor tracking and subtraction, temporal filter, detection and tracking) -> original smoothing/averaging (unchanged) -> **display stage** (trace and waterfall processed independently). Stored IQ captures are never modified: processing happens on the spectrum, after the capture is written.

| Group | Settings |
|---|---|
| FFT / WINDOW | window (RECT, HANN, HAMMING, BLACKMAN, BLK-HARRIS, FLAT-TOP, BARTLETT, TRIANGULAR, LEGACY), IQ DC correction, IQ balance (blind gain + quadrature fix), IQ normalize |
| FREQ FILTERS | low-pass / high-pass / band-pass / band-stop, centre, width, roll-off, reject depth, three notches + width, DC-bin suppression, adjacent-bin rejection |
| NOISE REDUCTION | floor subtraction strength, floor rise rate, narrowband guard, impulse / transient / outlier rejection |
| SIGNAL DETECT | CFAR (CA / GO / OS), guard and training cells, threshold, minimum prominence, track drop time, persistence window |
| TEMPORAL | EMA, adaptive EMA, median-of-3, median-of-5, alpha, adaptive threshold |
| DISPLAY | trace smoothing, peak decay, min hold, noise-floor line, range mode (legacy / floor+span / floor+peak) and span, waterfall suppression, interpolation, contrast, smoothing |

Extra pages: **SIGNALS** (tracked peaks: frequency, level, SNR versus local CFAR noise, width at half prominence, prominence, persistence %, drift), **IQ STATS**, and **PERFORMANCE** (measured on the device).

### Windows: resolution versus leakage

Computed from the exact 256-point coefficients the firmware uses (`tests/dsp_host/window_metrics.py` and the host test). Equivalent noise bandwidth (ENBW) sets frequency resolution and the noise-floor level; the sidelobe level sets how far a strong signal smears into weak neighbours; scalloping is the worst-case level error for a tone between two bins.

| Window | Highest sidelobe | ENBW (bins) | Resolution (kHz) | Scalloping |
|---|---|---|---|---|
| RECT | -13 dB | 1.00 | 62 | 3.92 dB |
| BARTLETT / TRIANGULAR | -26 dB | 1.33 | 83 | 1.8 dB |
| HANN | -31 dB | 1.50 | 94 | 1.42 dB |
| HAMMING | -43 dB | 1.36 | 85 | 1.75 dB |
| BLACKMAN | -58 dB | 1.73 | 108 | 1.10 dB |
| BLK-HARRIS | -92 dB | 2.00 | 125 | 0.83 dB |
| FLAT-TOP | -93 dB | 3.77 | 236 | 0.01 dB |

Narrow windows resolve close signals but leak; wide windows reject leakage and read amplitude accurately but merge close signals and raise the noise floor. The 8-bit input limits useful dynamic range to roughly 48 dB, so the -92 dB windows cannot show their full benefit on this hardware. Weights are Q15, built once when you change the window, and the output is rescaled by the window's coherent gain so a bin-centred tone reads the same in every window. `LEGACY` keeps the original rect/Hann behaviour (no gain correction).

### Honest limits

- IQ balance/DC figures are estimates from 256 samples, smoothed across frames. A strong real-valued tone or true DC signal biases them. Correcting DC removes a genuine signal at the centre frequency too.
- The noise floor tracker follows the lower envelope, so it under- rather than over-estimates the mean noise. A carrier can never become "the floor" (the floor is capped near 1.5x the frame median). Narrowband guard keeps signals above 2x the floor at 87% or more of their level; impulse and outlier rejection spare anything that was already present in the previous frame.
- CFAR thresholds are relative to the local reference cells. The default is OS-CFAR at 6x; on pure noise it produced no persistent tracks in 300-frame host tests (see `tests/dsp_host/false_alarm.cpp`), but real environments differ, so tune THRESHOLD.
- Detection runs on the cleaned spectrum before the original smoothing/averaging stage.

### Measured cost

| Item | Result | How measured |
|---|---|---|
| Firmware image | 934,576 B of the 1,048,576 B app partition (11% free) | ESP-IDF 5.5.1 build |
| DSP code | dsp_adv 10.7 kB, dsp_cfg 6.1 kB (mostly menu strings) | `size` on the xtensa objects |
| DSP static RAM | 6.7 kB (bss), no heap allocation, no fragmentation | `size` on the xtensa object |
| Stack | stage 1,008 B, FFT 2,096 B (already existed at 2,080 B); radio task stack is 8,192 B | `-fstack-usage` with the target compiler |
| Relative CPU (x86 host) | all options on: about 4x the FFT time; defaults: FFT only | `tests/dsp_host/bench.cpp` |
| ESP32 microseconds per stage, stack and heap headroom | **read on the PERFORMANCE page** | measured live with `esp_timer`, task stack high-water and heap counters |

I could not run the firmware on an ESP32 here, so absolute on-device timings are not reported in this document; the PERFORMANCE page shows them for your board.

### Host tests

`sh tests/dsp_host/run.sh` builds the DSP modules for a PC and checks: defaults equal the legacy FFT bit for bit, window coefficients against the published table, I/Q DC and image-rejection, detection and tracking of three tones, false alarms on noise, every filter mask, narrowband signals surviving subtraction, impulse/transient/outlier behaviour, temporal median, and config sanitising.

## GPS map, navigation and Remote ID (GPS menu)

New GPS menu entries: **Offline map**, **Nav / speedometer**, **Remote ID (passive)**. Existing GPS pages are unchanged.

**Offline map.** Put a standard sorted OSM XML export at `maps/regional_map.osm` on the SD card. Open the map, press Center for the quick settings and run *Rebuild map index*: the file is streamed once (never loaded into RAM) into per-cell polyline files under `map_cache/regional_map/`. Indexing runs in small slices from the UI loop, shows progress, and refuses to start (showing `NEED xK HAVE yK`) when internal heap is too low. Rendering uses a 56-row band canvas (6.7 KB) and is also sliced, so the SDR task is never blocked. While indexing, the canvas and renderer are freed so the indexer (about 16 KB peak) fits; the index uses an 8x8 cell grid. Without a map, GPS, trace, Remote ID and range overlays still draw over a black background.
- Controls: Up/Down zoom, Left re-centre on GPS, Right pan mode (arrows pan, Center done), Center quick settings, hold Center exits. Quick settings: zoom, follow, north-up/heading-up, trace on/off/clear, HUD, Remote ID markers and scan, range rings, layer presets, units, trip reset, rebuild index, load `maps/rf_range.csv`.
- Map symbols: receiver GPS = arrow (hollow dashed circle when the fix is lost; dashed circle = accuracy); aircraft = diamonds (filled = active, hollow = stale >10 s, X = lost >30 s) with reported heading line and short trail. The two are never drawn alike.
- Limits: needs the standard OSM ordering (nodes, ways, relations; unsorted node ids are rejected); relations/multipolygons are not rendered (large lakes and some buildings drawn as multipolygons will be missing); no street names or POIs; at most 1,000,000 nodes and 384 nodes per way (the tail of longer ways is dropped and counted); the first index of a large region takes minutes. Use a small regional extract.

**Nav / speedometer.** Dial plus speed, heading, latitude/longitude, altitude, fix type, satellites, HDOP/PDOP/VDOP, accuracy, UTC, fix age, trip distance/time/average/maximum. Every value says where it came from: speed `GNSS` or `DERIVED`, heading `GNSS` (course over ground, only above 1 m/s), `TRACK EST` (bearing of recent movement) or `HELD` (old), accuracy `GST` (receiver-measured) or `HDOPx5 EST`. Nothing is shown as live after the fix is lost.

**Remote ID (passive).** Receive-only. Uses the ESP32 Marauder `RemoteIdDecoder`/`RemoteIdModel` sources unmodified (verified by host tests against hand-encoded ASTM F3411 packets). Classic ESP32 receives Wi-Fi beacon and Wi-Fi NAN frames through the promiscuous monitor on channel 6 plus a channel sweep; the SDR receive task is paused while scanning and resumes when scanning stops or the screen closes. BLE legacy reception exists in `rid_ble.cpp` but only compiles in when the Bluetooth controller is enabled in menuconfig (off by default), so by default the screen shows `BLE OFF`. Marauder's BLE 5 extended-advertising path needs a BLE5 chip and is not available on this ESP32. The aircraft-reported position is shown separately from the receiver GPS, with report age and state. The operator-location field is deliberately never stored, drawn or used, and no operator position is inferred from aircraft telemetry.

**RF delay-to-distance.** `main/rf_range.cpp` implements d = c·t for hardware-timestamped round-trip (minus known turnaround and calibration) or synchronised one-way measurements, with uncertainty rings (distance ±2σ). It rejects software timestamps, packet arrival times and anything not flagged as hardware-timestamped. **This board has no hardware time-of-flight capability, so nothing is measured on-device**; rings appear only for measurements you import in `maps/rf_range.csv` (`label,mode,time_ns,turnaround_ns,cal_ns,time_unc_ns,cal_unc_ns,sync_unc_ns,hw_timestamped,obs_lat,obs_lon`, mode `RT` or `OW`). Wi-Fi round-trip time or packet timing will not give valid propagation time with this firmware.

**Memory.** Static RAM is now about 140 KB of 180 KB, so heap is tight. The image viewer, indexer and map canvas allocate on demand and report `NEED/HAVE` instead of crashing. PNG decoding (zlib) needs the most heap; JPEG is lighter. Progressive JPEGs are not supported (reported as such).

**Partition table.** The image no longer fits a 1 MB app partition, so `partitions.csv` (3 MB factory app) is now used; flash it with a full flash (`idf.py flash`), not an app-only update at the old layout.

## Status bar

`TTT C 42% M 38% B 87% T152F SD G`: app, CPU load, internal RAM used, battery (MAX17048), die temperature in degrees
Fahrenheit, SD state, GPS fix. Every field has a fixed width, so shrinking values can no longer leave stale digits
(the previous bar left `BAT:990` / `I10%9` artefacts). The temperature field turns amber from 175 F and flashes red at
190 F or above (clears below 187 F). The ESP32 has no supported temperature driver; this reads the legacy on-die
sensor, which is coarse (several degrees F) and tends to read high, so treat it as a relative warning, not a calibrated number.
CPU load needs `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y` and `CONFIG_FREERTOS_USE_TRACE_FACILITY=y`; both are set in `sdkconfig`
and `sdkconfig.defaults`. If your IDF build complains about the old `sdkconfig`, delete it and let `sdkconfig.defaults` regenerate it.

The battery gauge (MAX17048, SDA GPIO33) is probed on SCL GPIO16 (Marauder V7) and GPIO27 (V7.1), and is reset once like Marauder's driver does. **V7.2** is a battery-polarity revision; no public source was found that states its gauge pins, so V7.2 is *not* assumed to match either. The firmware auto-detects which of the two known SCL pins answers at 0x36 and keeps all other pin assignments unchanged. The Device Info page shows `HW MARAUDER V7.2` and the detected gauge SCL pin (or `NONE`); if it says NONE on your board, report the real SCL GPIO.

Units are imperial by default (GPS altitude/speed/distance, antenna length in inches, temperature in F). The GPS page still has the
metric toggle.

## Mayhem-inspired compatibility

`FEATURE_MATRIX.md` inventories the Mayhem application categories and app catalog consulted for this project and marks each row implemented, partial, or unavailable for this hardware/firmware. The ESP-SDR path snapshots only the 2.4 GHz range. The TX bench app sends a single fixed certification test frame; it does not provide raw-IQ transmit, RF replay, transceiver messaging, remote control, or disruption functions. Narrowband decoders outside the observed RF range, continuous streaming functions, and HackRF-specific controls are not represented as working controls.

The official ESP-SDR repository describes its original ESP32 implementation as receive-only. Marble's public raw-IQ TX paths target other ESP32 variants; the original ESP32 path is not published. This firmware uses the separate Espressif PHY certification API only for a single fixed test frame. Captured files remain unsupported for RF transmission, and local FFT playback is never labeled as over-the-air replay. See [TX_PORT_RESEARCH.md](TX_PORT_RESEARCH.md) for the source review and limits.

## Source layout and validation

- `main/app_main.cpp`: radio/DSP, UI state, capture/replay, profiles, utilities, GPS app, RF Replay compatibility app, and diagnostics tasks.
- `main/input.cpp`: shared active-low joystick sampling, debouncing, long press, configurable hold repeat, acceleration, and input inversion.
- `main/dsp.cpp`: fixed-point 256-point FFT and 256-step heatmap palettes.
- `main/display.cpp`: v7 ILI9341 setup, DMA scanlines, CP437 text, spectrum and scrolling waterfall.
- `main/media_player.cpp`: streamed JPEG/PNG rendering (buffers allocated per image) and explicit MP4 unsupported notice.
- `main/browser.cpp`, `include/browser.hpp`, `include/dir_window.hpp`: SD file browser model (bounded-RAM paging); host-tested.
- `main/nav.cpp`, `main/geo.hpp`: navigation engine (speed, heading estimate, accuracy, trip, trace) and geodesy; host-tested.
- `main/osm.cpp`, `main/map_ui.cpp`: offline OSM indexer/renderer and the map, speedometer and Remote ID screens.
- `main/rid.cpp`, `main/rid/`: Remote ID adapter over the unmodified ESP32 Marauder decoder (MIT, see `main/rid/NOTICE.md`).
- `main/rf_range.cpp`: RF delay-to-distance maths and CSV import.
- `main/settings.cpp`: versioned NVS preferences and validation.
- `main/storage.cpp`: shared-SPI FAT mount, capture/metadata, GPS logging, and bounded SD operations.
- `main/gps.cpp`: checksummed NMEA RMC/GGA parser and honest fix/quality/staleness statistics; no synthetic fix is generated.
- `main/battery.cpp`: bounded optional MAX17048 I2C state-of-charge reads; no fallback voltage or charging estimate.
- `components/esp_sdr_backend/`: original-ESP32 ESP-SDR receiver adapter, RX gain controls, one-shot certification packet hook, and upstream SRAM/linker safeguards.
- `main/idf_component.yml`: pinned ESP-IDF JPEG ROI and libpng decoder dependencies.

The project builds with ESP-IDF v6.1. The display clear path resets the ILI9341 vertical-scroll origin before drawing screens, addressing black patches left on menus after spectrum and scan views. The latest ESP-IDF size report is 844,634 bytes total image size (the padded `.bin` is 844,752 bytes; 19% remains in the 1 MiB app partition) and 95,571 bytes static DRAM. The pre-patch baseline was 827,802 image bytes and 93,875 bytes DRAM. Build validation passed. Physical board testing remains necessary for joystick feel, screen artifacts, RX gain response, battery gauge detection, SD status, and GPS. This build has no transmit operation.

## References and licenses

- [ESPARGOS ESP-SDR](https://github.com/ESPARGOS/esp-sdr) and its [capture explanation](https://github.com/ESPARGOS/esp-sdr/blob/9b61e7465bee6da15209c5c09308043d08c46a1d/docs/how-it-works.md).
- [Marble ESP-SDR Zephyr port](https://github.com/cyber-murmel/esp-sdr-zephyr) and [TX API](https://github.com/cyber-murmel/esp-sdr-zephyr/blob/3214c16a7c9f136414cc5ae9d245ea290c98a6ce/include/esp_sdr/esp_sdr_tx.h).
- [Official ESP32 Marauder V7 board page](https://github.com/justcallmekoko/ESP32Marauder/wiki/Marauder-v7), [supported hardware table](https://github.com/justcallmekoko/ESP32Marauder/wiki/Supported-Hardware), and [upstream board pin definitions](https://github.com/justcallmekoko/ESP32Marauder/blob/master/esp32_marauder/configs.h).
- [Marauder V7 fuel gauge instructions](https://github.com/justcallmekoko/ESP32Marauder/wiki/Battery-Fuel-Gauge) and [battery IC connections](https://github.com/justcallmekoko/ESP32Marauder/wiki/IC-Connections) describe the optional MAX17048 gauge and I2C wiring. The board's charging/power IC does not give this firmware a documented charging-state GPIO or battery-voltage ADC pin.
- [PortaPack Mayhem firmware](https://github.com/portapack-mayhem/mayhem-firmware) and [Mayhem application wiki](https://github.com/portapack-mayhem/mayhem-firmware/wiki/Applications) were used as feature references; Mayhem code, branding, and UI assets are not included.
- CP437 font source and attribution: [FONT_LICENSE.md](FONT_LICENSE.md). ESP-SDR-derived code is GPL-3.0-or-later; its upstream license is in `components/esp_sdr_backend/UPSTREAM_LICENSE`.

## Experimental unlocks (Utilities > Debug tools)

Two checkboxes at the bottom of the Debug tools screen. UP/DOWN selects, CENTER toggles. Both are off at every boot and are never written to flash.

- **TUNE 100-6000 MHZ** widens every tuning limit (spectrum, Frequency manager, fox hunt, Looking Glass sweep) from 1842-2800 MHz to 100-6000 MHz. These are direct PLL tune attempts to find where the silicon actually locks and receives; nothing is calibrated outside the 2.4 GHz band. Turning it off clamps the current center back into range.
- **Z UNLOCK UP TO 64**: Z (the displayed slice in MHz) normally tops out at 16 because the dump engine captures 16 MHz per frame. With the unlock, Z can be 32 or 64. Only the middle 16 MHz holds data; the rest is drawn blank so the real capture edge is visible. Turning it off clamps Z to 16.

Saved settings are always stored in the locked range, so a reboot never reads back an unlocked value.

## Fox transmit: reading a failed start

Pressing START (ARM) then CENTER again used to fall back to "START (ARM)" with no reason shown, because the start result was overwritten by a generic hint. The result now stays on screen:
- `NO MEM NEED 8K HAVE nK`: the transmit task needs an 8 KB stack in one internal block and the heap has less.
- `START ERR <code>` / `TX ERR <code>`: the radio session or tone call failed; the code is the ESP error name.
- The line under it shows the last step reached: `START REQUESTED`, `TASK STARTED`, `CW BEGIN`, `CW TONE ON`, `RUNNING`, or `STOPPED (...)`.
- While or after transmitting, `TX STACK FREE a/b B` shows the lowest unused stack of the transmit task. If `a` is under about 1000, the stack is too small; tell me the number.

## Fox MULTI mode (custom frequency list)

MODE = MULTI hops through a list of up to 12 frequencies you enter yourself, at 1 MHz resolution. On the FREQ LIST row press CENTER to open it.
- **UP/DOWN** move between rows.
- **LEFT/RIGHT** on a frequency row tunes it by the current STEP (1, 5, 20 or 100 MHz; hold to repeat).
- **CENTER** on a frequency row switches it on or off. At least one stays on.
- **+ ADD** appends a new frequency, one step above the last entry (or at the carrier if the list is empty).
- **STEP** (LEFT/RIGHT or CENTER) cycles the tuning step. It is shared with the CARRIER row, which now also offers 1 MHz.
- **- REMOVE** deletes the frequency you last had selected (the list keeps at least one).
- **DONE** saves and closes. INTERVAL sets the dwell on each frequency; edits apply live while transmitting.

The SDK tone only takes 5 MHz channel numbers. For a frequency that is not on that grid the firmware starts the tone on the nearest channel and then retunes the synthesizer with the same ROM call the receive path uses. That fine-tune is **experimental and unverified**: the Fox page stage line reads `RUNNING (FINE TUNE UNVERIFIED)` while one is active. Check the real frequency on a receiver before trusting it.
Saved Fox settings from older builds reset to defaults once because the stored format changed.

## App device calls

VM1 apps can now read and set the SDR center and Z, read the latest spectrum frame and GPS, run a passive Wi-Fi scan, and key the fox carrier, all through the SYS opcode. See `APP_API.md`. `sdcard_examples/APPS/WIFISCAN.APP` is a working example. Bluetooth and arbitrary frame transmit are not exposed.

## File browser: instant Back

The SD file browser keeps the last two folder levels you descended through in RAM (about 3 KB, allocated only while browsing). Pressing Left to go back up restores that folder and your selection without reading the card. Deleting a file, jumping elsewhere or leaving the browser clears the cache, and if memory is short it quietly reads the card as before. Rows whose text has not changed are already skipped by the display's text cache, so no extra culling was needed there.

VM1 apps can also read and write SD files; see `APP_API.md`. Try `sdcard_examples/APPS/SDLIST.APP`.
