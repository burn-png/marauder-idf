# Mayhem feature inventory and Dominion compatibility with Marauder V7 hardware

> Target hardware: **Marauder V7.2** (battery-polarity revision). No V7.2 GPIO change is assumed; the MAX17048 SCL pin (16 or 27) is auto-detected and shown on Device Info.

This inventory follows the Mayhem application category index and the Receivers, Transmitters, Transceivers, Utilities, Games, and Settings wiki pages reviewed on 2026-11-05. A status describes this project, not the status of Mayhem itself.

**Status key:** **Complete** = implemented in this project for the stated hardware scope; **Partial** = a narrower equivalent exists; **Unavailable** = hardware, architecture, or safety policy prevents it. “Unavailable” rows remain listed so unsupported capabilities are explicit. TX entries are deliberately disabled; passive/offline tools do not imply permission to transmit.

## Main categories and shared interface

| Mayhem feature/category | Hardware needs | Marauder v7 compatibility | Project implementation | Status |
| --- | --- | --- | --- | --- |
| Receivers / Spectrum Analyzer | RF receiver, display, user controls | Original ESP32 ESP-SDR snapshot RX; direct-PLL requests 1842–2800 MHz, above 2484 MHz marked unverified/out of band | 256-point FFT, trace, hold, averaging, tune and display zoom; Rolling RSSI Detector plots relative peak-bin level and triggers above an adjustable threshold | Partial |
| Transmitters | RF modulator/transmitter path | ESP32 PHY certification test API only; no ESP-SDR raw-IQ TX path | One fixed one-shot channel 1 test packet; captured-IQ transmission remains disabled | Partial |
| Transceivers | Separate RX and TX paths, audio/PTT | No supported TX/audio path here | No transceiver operation or placeholder menu page | Unavailable |
| Recon | Tunable receiver and persistent storage | Passive 2.4 GHz snapshots only | Channel activity sweep and CSV logging | Partial |
| Capture | IQ receiver and SD | Supported as finite snapshots | IQ8 files and companion metadata | Partial |
| Offline Replay | IQ receiver pipeline and SD | Offline DSP playback is supported; no TX | Reprocess IQ8 into local FFT and waterfall; explicitly offline | Partial |
| RF Replay | Contiguous IQ source and supported transmitter hardware | Original ESP32 Marauder v7 has receive-only ESP-SDR path; no transmitter | Separate app browses captures and metadata, checks format/rate/frequency/continuity, reports RF TX unavailable, saves per-capture `.RRC` configuration with TX disabled | Architecture implemented; RF TX unavailable |
| GPS | GPS UART, receiver and SD | UART2 MCU RX/TX GPIO21/22, 9600 baud; no controllable GPS power line | Dashboard, checksum-validated talker-independent RMC/GGA parsing, coordinate validation, stale-fix indication, NMEA/UART diagnostics, named track recording, pause/resume/stop, CSV browse/point review, named saved locations and delete; selectable coordinate format and units | Partial |
| Remote | Compatible remote-control endpoint/protocol | No Mayhem remote link | No remote operation or placeholder menu page | Unavailable |
| Looking Glass | Wideband receiver, tunable LO, display | Narrow RF snapshots only; upper tune requests unverified | Responsive default scan range; adjustable/lockable bounds; skip failed tunes; selectable hop/dwell, waterfall/live/peak views, pause, RX gain, display span, and smoothing filter | Partial |
| Utilities | Display, controls, SD, peripherals | Varies by utility | See detailed utility rows | Partial |
| Games | Display and controls | Five-way tactile joystick, no touch | No games are shipped in this firmware | Unavailable |
| Settings | Persistent storage and controls | NVS and SD available | Persistent receiver/DSP settings and reset | Partial |
| HackRF | HackRF hardware/control protocol | Not a HackRF/PortaPack | No HackRF operation or placeholder menu page | Unavailable |
| External apps | Versioned app ABI/runtime | Sandboxed bytecode VM (VM1), no native code loading | `.APP` manifests listed; apps with `RUNTIME=VM1` + `CODE=` run (draw/joystick/timing only); example Snake; Analog TV demo stays metadata-only | Partial (host-tested; untested on hardware) |
| Menu organization, selection, scrolling, back navigation | Display and controls | Five-way tactile joystick | Receiver menu groups live spectrum, signal meter, Looking Glass, passive scan, and Rolling RSSI Detector; shared 10 ms input scan, 12 ms default debounce, immediate press event, configurable hold-repeat; dirty-row/text rendering; direct Settings entry with Back first | Partial |
| Status bar | Heap, SD, optional battery gauge, GPS | Internal heap and SD status available; optional MAX17048 on GPIO33/16; no documented battery ADC or charging-state signal | Changed-value-only app/heap/SD/battery/gps status; SD reads exactly `SD: OK` or `SD: NG`; battery unknown without gauge | Partial |
| Search across applications | Text input/query controls | No text keyboard | No general text search | Unavailable |
| Status / confirmations | Display and controls | Supported | Status text, unavailable notices, destructive actions identified in menus | Partial |

## Receiver application catalog

Selected receiver applications from the Mayhem catalog are listed below. Except where noted, the Marauder backend only supplies short 2.4 GHz IQ snapshots; these named decoders require bands, continuous IQ/audio, demodulators, or protocol implementations that this project does not provide.

| Mayhem receiver app | Hardware needs | Marauder v7 compatibility | Project implementation | Status |
| --- | --- | --- | --- | --- |
| 2-Tone RX | Narrowband audio/demodulator | Outside provided decoder path | No decoder | Unavailable |
| ACARS | VHF receiver and FSK decoder | Outside RF range | No decoder | Unavailable |
| ADS-B | 1090 MHz receiver and decoder | Outside RF range | No decoder | Unavailable |
| AFSK | Audio/baseband demodulator | No audio demodulation path | No decoder | Unavailable |
| AIS Boats | 162 MHz receiver and decoder | Outside RF range | No decoder | Unavailable |
| Analog TV | Video-capable receiver | No video/baseband path | SD manifest demo only; no decoder or launchable app | Unavailable |
| APRS RX | 144 MHz receiver and packet decoder | Outside RF range | No decoder | Unavailable |
| Audio | Tunable receiver and audio output | No audio codec/baseband path | No audio receiver | Unavailable |
| Detector | Continuous RSSI/squelch receiver | Only intermittent snapshots | Threshold detection used by channel scan | Partial |
| EPIRB RX | 406 MHz receiver and decoder | Outside RF range | No decoder | Unavailable |
| ERT Meter | Sub-GHz receiver and protocol decoder | Outside RF range | No decoder | Unavailable |
| Flex RX | Paging-band receiver and decoder | Outside RF range | No decoder | Unavailable |
| Fox Hunt | Tunable receiver and directional antenna | Limited to ESP-SDR bins | Passive frequency tuning, relative FFT level and peak offset, manual antenna-orientation comparison; no TX, decoding, calibrated dBm, or automatic bearing | Partial |
| FPV Detect | 5.8 GHz receiver | Outside RF range | No detector | Unavailable |
| FT8 RX | HF receiver, audio and FT8 decoder | Outside RF range and no audio path | No decoder | Unavailable |
| gfxEQ | Audio path and display | No audio input pipeline | No audio spectrum view | Unavailable |
| Level | Tunable RSSI receiver | Relative FFT-bin energy only | Receiver submenu; spectrum magnitude and peak view | Partial |
| Looking Glass | Tunable wideband receiver | 2.4 GHz snapshots; broader tuning requests are unverified | Hopped waterfall with adjustable/lockable bounds, skip-on-error tuning, selectable hop/dwell and display modes, pause, shared gain, display span, and smoothing | Partial |
| Meteor LRPT | 137 MHz receiver and image decoder | Outside RF range | No image decoder | Unavailable |
| Morse RX | Narrowband audio/baseband decoder | No demodulated audio | No decoder | Unavailable |
| NOAA | 137 MHz receiver and image decoder | Outside RF range | No image decoder | Unavailable |
| NRF | nRF24 compatible packet receiver | ESP-SDR does not decode nRF packets | No packet decoder | Unavailable |
| POCSAG | Paging-band receiver and decoder | Outside RF range | No decoder | Unavailable |
| ProtoView | OOK/baseband protocol analyzer | No supported demodulated baseband | No protocol analyzer | Unavailable |
| Radio | AM/FM demodulator and audio output | No audio/baseband path | No broadcast receiver | Unavailable |
| Radiosonde | VHF receiver and telemetry decoders | Outside RF range | No decoder | Unavailable |
| RDS RX | FM demodulator/RDS decoder | Outside RF range and no audio demodulation | No decoder | Unavailable |
| RTTY RX | Narrowband demodulator and decoder | No audio/baseband path | No decoder | Unavailable |
| Scanner | Tunable receiver, list and squelch | Only 2.4 GHz channels are observed | Passive 14-channel scan and activity log | Partial |
| Search | Wideband sweep and signal detector | Sweep restricted to supported range and sparse snapshots | Basic channel scan; no arbitrary-range search | Partial |
| Signal Hunter | Continuous signal detector and auto-capture | No continuous stream; capture is manual | No automatic threshold-triggered capture | Unavailable |
| Spectrum Analyzer | Tunable receiver and display | Supported as a low-duty-cycle view | Spectrum trace, peak hold and waterfall | Partial |
| SSTV RX | Audio demodulator and image decoder | No audio demodulation | No decoder | Unavailable |
| SubCar | Sub-GHz receiver and protocol decoder | Outside RF range | No decoder | Unavailable |
| SubGhzD | Sub-GHz receiver and protocol decoder | Outside RF range | No decoder | Unavailable |
| Tetra RX | TETRA receiver and decoder | Outside RF range | No decoder | Unavailable |
| Time Sink | Continuous IQ/time-domain path | Snapshot only | No time-domain viewer | Unavailable |
| TPMS RX | 315/433 MHz receiver and decoder | Outside RF range | No decoder | Unavailable |
| VOR RX | 108–118 MHz receiver and audio decoder | Outside RF range | No decoder | Unavailable |
| Weather | Sub-GHz sensor receiver and protocol decoders | Outside RF range | No decoder | Unavailable |
| WeatherFax / WeFax | HF receiver and image decoder | Outside RF range | No decoder | Unavailable |
| WMBus RX | 868 MHz receiver and protocol decoder | Outside RF range | No decoder | Unavailable |

## Transmitter application catalog

Mayhem transmitter applications are listed individually below. The original ESP32 Marauder v7 project has no supported RF transmit path. No transmitter is implemented, and no IQ capture can be retransmitted over the air.

| Mayhem transmitter app | Hardware needs | Marauder v7 compatibility | Project implementation | Status |
| --- | --- | --- | --- | --- |
| 2-Tone TX | RF modulator | No TX path | Disabled | Unavailable |
| ADS-B(S) TX | RF modulator | No TX path; aviation signal simulation excluded | Disabled | Unavailable |
| Adult Toys | BLE transmitter | No supported TX path | Disabled | Unavailable |
| APRS TX | VHF transmitter | No TX path | Disabled | Unavailable |
| BHT Xy/EP | RF transmitter | No TX path | Disabled | Unavailable |
| BLE TX | BLE transmitter | No supported TX path | Disabled | Unavailable |
| BLESpam | BLE transmitter; disruptive spoofing | No supported TX path; disruptive use excluded | Disabled | Unavailable |
| Burger Pager | RF transmitter | No TX path | Disabled | Unavailable |
| CVS Spam | RF transmitter; disruptive paging | No TX path; disruption excluded | Disabled | Unavailable |
| EPIRB TX | Distress beacon transmitter | No TX path; distress signaling excluded | Disabled | Unavailable |
| Flex TX | RF transmitter | No TX path | Disabled | Unavailable |
| FlipperTX | Sub-GHz transmitter | No TX path | Disabled | Unavailable |
| GPS Sim | GNSS transmitter/spoofer | No TX path; spoofing excluded | Disabled | Unavailable |
| Hopper | RF transmitter | No TX path | Disabled | Unavailable |
| Jammer | RF power amplifier | No TX path; interference excluded | Disabled | Unavailable |
| KeeLoq TX | RF transmitter and rolling-code files | No TX path; retransmission excluded | Disabled | Unavailable |
| Key fob TX | Sub-GHz transmitter | No TX path | Disabled | Unavailable |
| LGE Tool | RF transmitter | No TX path | Disabled | Unavailable |
| MDC 1200 TX | RF transmitter | No TX path | Disabled | Unavailable |
| Morse TX | RF transmitter | No TX path | Disabled | Unavailable |
| OOK | Sub-GHz transmitter | No TX path | Disabled | Unavailable |
| OOK Brute | RF transmitter and sequential code generation | No TX path; brute-force excluded | Disabled | Unavailable |
| OOK Editor | RF transmitter | No TX path | Disabled | Unavailable |
| P25 TX | RF transmitter | No TX path | Disabled | Unavailable |
| POCSAG TX | RF transmitter | No TX path | Disabled | Unavailable |
| RDS | FM transmitter | No TX path | Disabled | Unavailable |
| RTTY TX | RF transmitter | No TX path | Disabled | Unavailable |
| SAME TX | RF transmitter; emergency alert signal | No TX path; alert simulation excluded | Disabled | Unavailable |
| Security+ TX | Sub-GHz transmitter and rolling-code system | No TX path; unauthorized access excluded | Disabled | Unavailable |
| Signal gen | RF modulator | No TX path | Disabled | Unavailable |
| Soundboard | RF transmitter and audio playback | No TX path | Disabled | Unavailable |
| Spectrum Painter | RF transmitter | No TX path | Disabled | Unavailable |
| SSTV | RF transmitter | No TX path | Disabled | Unavailable |
| TEDI/LCR | RF transmitter | No TX path | Disabled | Unavailable |
| TouchTunes | RF transmitter | No TX path | Disabled | Unavailable |
| TPMS TX | RF transmitter | No TX path | Disabled | Unavailable |
| VOR TX | RF transmitter | No TX path; navigation spoofing excluded | Disabled | Unavailable |

## Transceiver, capture, replay, and RF profile features

| Mayhem feature | Hardware needs | Project implementation | Status |
| --- | --- | --- | --- |
| Microphone Transceiver | RF TX/RX, microphone, audio codec, PTT | Receive-only hardware; compatibility notice | Unavailable |
| KISS-TNC | Two-way AX.25 modem and RF path | No modem/transmit path | Unavailable |
| Capture application | Continuous IQ and SD throughput | Finite IQ8 snapshots with metadata | Partial |
| User capture naming | Keyboard or character-entry interface | Sequential `RX####.C8` names | Partial |
| Capture timestamps/frequency/rate/format/count/duration | SD metadata | `.META` companion with UTC if GPS fix, uptime and technical metadata | Complete for available fields |
| Capture browser, integrity, delete | SD/filesystem | Size display; IQ8 length checked for replay; paired delete | Partial |
| Offline IQ playback | IQ source and DSP | IQ8 replay through local FFT only; distinct from RF transmission | Partial |
| RF Replay application | Capture metadata and transmitter compatibility | App is present; RF TX remains disabled on this RX-only board | Capture format/frequency/rate/continuity inspection; `.RRC` saved configs set `TRANSMIT_ALLOWED=0` | Architecture implemented; RF transmit unavailable |
| Playback position/pause/resume/loop | DSP control | Progress, pause/resume, fixed loop | Partial |
| Seek and saved playlists | Seekable file index, playlist editor | Not implemented | Unavailable |
| RF Profiles | Persistent receiver/DSP configuration | Versioned `.RFP` files; save/load/duplicate/delete; copying files imports/exports | Partial |
| Profile naming/descriptions/markers/annotations | Text editor and persistent profile schema | Auto-named RF00–RF31; no profile description/marker editor | Partial |
| Associate capture with RF profile | Capture/profile metadata link | Not implemented | Unavailable |

## Utilities catalog

| Mayhem utility | Hardware needs | Project implementation | Status |
| --- | --- | --- | --- |
| Antenna Length | Calculator/UI | Calculates quarter-, half-, and 5/8-wave lengths at current center using 0.95 velocity factor | Complete |
| Calculator | UI | Joystick digit/operator selection and two-operand arithmetic | Partial |
| Cart Lock | RF transmitter | Not implemented | Unavailable |
| Debug | Heap, task and hardware diagnostics | Heap/frame/error/last capture time screen | Partial |
| File Manager | SD browser and file operations | Lists SD entries; deletion limited to IQ/meta captures | Partial |
| Flash Utility | Bootloader and image validation | On-device instructions point to ESP-IDF flashing in VS Code; no SD-image flasher | Partial |
| Frequency Manager | Frequency lists and editor | Cyclic 5 MHz tune step within supported band; no saved named list | Partial |
| IQ Trim | IQ file reader/writer | Finds quiet IQ8 blocks and writes a separate trimmed copy | Complete |
| SD file browser (media viewer + file manager) | Display, joystick and SD | Multi-row folder browser with bounded-RAM paging; JPEG (baseline, any size up to 3840 px wide via 1/8 decode and sampling) and non-interlaced PNG viewing with previous/next; text preview; info; guarded delete; MP4 recognised but no decoder | Partial (host-tested model; image path compile-checked, untested on hardware) |
| Offline OSM map | SD, display, GPS | Streaming indexer and band renderer, pan/zoom/follow, trace, HUD; no relations/multipolygons | Partial (host-tested; untested on hardware) |
| Nav / speedometer | GPS | Speed, heading estimate, accuracy, trip; source of each value shown | Implemented (host-tested; untested with a live receiver) |
| Remote ID (passive) | Wi-Fi promiscuous | Marauder decoder; Wi-Fi beacon + NAN; BLE legacy only with BT controller enabled | Partial (decoder host-tested; on-air reception untested) |
| RF delay-to-distance | None on this board | Maths + CSV import only; no on-device measurement | Interface only (hardware unsupported) |
| Metronome | Audio output | No audio output | Unavailable |
| Notepad | SD and text-entry interface | Append-only A–Z/space/backspace editor to `NOTES.TXT` | Partial |
| Playlist Editor | SD and replay playlist runtime | No playlists supported | Unavailable |
| Rand Pwd | Hardware random source and display | Generates a fresh 16-character hexadecimal value from ESP32 hardware RNG | Complete |
| SD Over USB | USB mass storage controller | Marauder v7 target has no USB device interface | Unavailable |
| Stopwatch | Timer and UI | Start/stop stopwatch with elapsed time display | Partial |
| Tuner | Audio input | No audio input path | Unavailable |
| WardriveMap | GPS, storage, map renderer | Valid GPS positions are logged to CSV; no on-device map | Partial |
| Waterfall Designer | Palette editor | Magma/Inferno/gray palette setting and floor/ceiling | Partial |
| WAV Viewer | Audio decode/output | No audio path | Unavailable |
| Wipe SD Card | SD destructive action | Deliberately omitted | Unavailable |

## Games catalog

Mayhem games include RF-dependent Battleship; those radio interactions are not implemented. Signal Catch is operated from the Marauder v7 joystick.

| Mayhem game | Hardware needs | Project implementation | Status |
| --- | --- | --- | --- |
| 2048 | Display and directional controls | Not implemented | Unavailable |
| Battleship | Two devices and RF link | No transceiver path | Unavailable |
| Blackjack | Display and controls | Not implemented | Unavailable |
| Breakout | Display and paddle controls | Not implemented | Unavailable |
| Digital Rain | Display | Not implemented | Unavailable |
| Dino Game | Display and controls | Not implemented | Unavailable |
| Doom | Display, controls, substantial memory | Not implemented | Unavailable |
| Morse P | Audio input/output and keying | No audio path | Unavailable |
| Pac-Man | Display and directional controls | Not implemented | Unavailable |
| Snake | Display and directional controls | Not implemented | Unavailable |
| Space Invaders | Display and controls | Not implemented | Unavailable |
| Tetris | Display and controls | Not implemented | Unavailable |
| Signal Catch (original) | Display, timer, joystick | Removed to reduce firmware footprint | Not shipped | Unavailable |

## Settings and hardware capabilities

| Mayhem setting/capability | Marauder v7/project status | Implementation status |
| --- | --- | --- |
| AppManager, hide/autostart, blacklist | No app runtime/launcher | Unavailable |
| Per-app SD settings | A single NVS settings record is used | Partial |
| Audio and CTCSS | No audio/TX pipeline | Unavailable |
| Battery/charge display | Optional MAX17048 on GPIO33/16; charging status is not exposed | Reads SOC every three seconds; shows unknown if gauge is absent |
| Joystick repeat/debounce/long press/inversion | Marauder v7 five-way switches | Shared input manager; repeat delay/rate, debounce, long press, acceleration, horizontal/vertical inversion in NVS | Partial |
| Touch calibration / touchscreen | Upstream v7 setup has touch CS disabled | Unavailable |
| Config mode | Not applicable to this application | Unavailable |
| Frequency converter/offset | Does not apply to narrow ESP-SDR center control | Unavailable |
| Date/time | GPS RMC UTC or GGA time when available; uptime otherwise | Partial |
| Display mode / IPS selection | Fixed ILI9341 setup | Fixed board setup | Partial |
| Encoder sensitivity/direction | No rotary encoder in this board setup | Unavailable |
| RX/TX frequency correction | No calibrated correction control | Unavailable |
| Hard Reset | Destructive SD/reset utility omitted | Unavailable |
| Menu color/theme | Fixed instrument palette | Partial |
| Persistent memory management | NVS settings and restore defaults | Partial |
| Clock output/reference input | No supported clock-control path | Unavailable |
| Antenna bias | Not exposed by this receiver backend | Unavailable |
| SD speed control | Shared SPI SD at fixed conservative clock | Partial |
| Splash configuration | No splash settings | Unavailable |
| TX limit | TX is absent and disabled | Unavailable |
| UI/touch/backlight timeout | Backlight GPIO is on; timeout/brightness control absent | Partial |
| Center frequency | ESP-SDR direct-PLL tune requests within 1842–2800 MHz on original ESP32; above 2484 MHz is unverified and above 2651 MHz exceeds the conservative PLL envelope | Partial |
| Start/stop frequency and span | No arbitrary bandwidth; zoom affects visible bins only | Partial |
| Frequency step | Persistent 1–100 MHz cyclic step | Complete |
| AGC/manual gain | AGC or manual index 0–127 from the original ESP32 RX gain register; persisted in NVS | Implemented; requires board response check |
| Sample rate | Capture backend nominally 16 MS/s snapshots | Fixed; no selectable presets |
| FFT size/window/averaging/hold | FFT is 256 bins; Hann, averaging, peak hold controls | Partial |
| Detector/overlap/RBW/dBFS calibration | Not provided by snapshot backend | Unavailable |
| Spectrum floor/range and markers | Relative intensity scaling; plotted left/center/right frequency labels; no calibrated dBFS | Partial |
| Waterfall Magma palette/intensity | Magma, Inferno, grayscale and persistent floor/ceiling; independent from trace scaling | Complete |
| Waterfall history depth/speed/clear | Fixed display scroll area and frame cadence | Partial |
| GPS position/fix/time/quality | Checksummed RMC/GGA, truthful stale/no-fix state, reported altitude/speed/course/quality/satellites/HDOP, UTC and parse/UART counters | Partial |
| GPS named tracks and saved locations | SD CSV logging, viewer, selection and deletion | Versioned `GPS/TRACKS/*.CSV`, timed point append, pause/resume/stop, point-by-point review, named `GPS/LOCATIONS.CSV` entries and deletion; viewer reports point count, endpoints, elapsed UTC time and straight-line endpoint distance | Partial |
| FFT window selection | Rect/Hann | RECT, HANN, HAMMING, BLACKMAN, BLK-HARRIS, FLAT-TOP, BARTLETT, TRIANGULAR (+LEGACY); Q15 table, coherent-gain corrected, persistent | Partial |
| Spectral filters | None | Low/high/band-pass and band-stop masks, 3 notches, DC-bin suppression, adjacent-bin rejection on the processed spectrum (IQ captures untouched) | Partial |
| Noise reduction | Bin smoothing + averaging | Tracked noise floor + subtraction, EMA, adaptive EMA, median-3/5, impulse/transient/outlier rejection, narrowband guard | Partial |
| Signal detection | Rolling RSSI detector | CA/GO/OS-CFAR, prominence, half-prominence width, sub-bin position tracking, relative SNR, persistence; relative FFT levels, not dBm | Partial |
| Display processing | Auto-fit trace, peak hold | Independent trace/waterfall smoothing, peak decay, min hold, floor line, floor-anchored range modes, waterfall suppression/interpolation/contrast | Partial |
| IQ pre-processing | None | Per-snapshot DC/gain/quadrature estimate and optional correction, normalisation; no continuous-stream claims | Partial |
| DSP self-measurement | None | On-device PERFORMANCE page (stage microseconds, stack, heap, CPU) | Partial |
| External application ABI launch | No safely supported loadable-app ABI | Discovery only; launch unavailable |

## References

- [Mayhem application categories](https://github-wiki-see.page/m/portapack-mayhem/mayhem-firmware/wiki/Applications)
- [Receiver app list](https://github-wiki-see.page/m/portapack-mayhem/mayhem-firmware/wiki/Receivers)
- [Transmitter app list](https://github-wiki-see.page/m/portapack-mayhem/mayhem-firmware/wiki/Transmitters)
- [Transceivers](https://github-wiki-see.page/m/portapack-mayhem/mayhem-firmware/wiki/Transceivers)
- [Utilities](https://github-wiki-see.page/m/portapack-mayhem/mayhem-firmware/wiki/Utilities)
- [Games](https://github-wiki-see.page/m/portapack-mayhem/mayhem-firmware/wiki/Games)
- [Settings](https://github-wiki-see.page/m/portapack-mayhem/mayhem-firmware/wiki/Settings)
- [Recon](https://github-wiki-see.page/m/portapack-mayhem/mayhem-firmware/wiki/Recon), [Capture](https://github-wiki-see.page/m/portapack-mayhem/mayhem-firmware/wiki/Capture), [Replay](https://github-wiki-see.page/m/portapack-mayhem/mayhem-firmware/wiki/Replay), and [Looking Glass](https://github-wiki-see.page/m/portapack-mayhem/mayhem-firmware/wiki/Looking-Glass)
