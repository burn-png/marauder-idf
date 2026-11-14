# VM1 device calls (SYS opcode)

VM1 apps run in a sandbox (own memory, bounds-checked, step-limited). The **SYS** opcode (60) is their only way to the device.
Push three arguments `a b c` (use 0 for unused ones), then the function number, then SYS. The result is left on the stack.
All values are 16-bit. Unknown functions return -1. There are no permission prompts.

| fn | Call | Returns |
|----|------|---------|
| 0 | API version | 1 |
| 1 | Get SDR center MHz | MHz |
| 2 | Set SDR center (a = MHz, within the current tuning limits; not saved) | 1 ok, 0 refused |
| 3 | Get Z span MHz | MHz |
| 4 | Set Z span (a = 1..max, not saved) | 1 ok, 0 refused |
| 5 | SDR bin a (0..239) from the latest frame | 0..255 |
| 6 | Strongest bin index | 0..239, -1 if no frame |
| 7 | Strongest bin level | 0..255, -1 if no frame |
| 8 | GPS fix | 1 / 0 |
| 9 | GPS satellites | count |
| 10 | GPS latitude, degrees x 100 | -9000..9000 |
| 11 | GPS longitude, degrees x 100 | -18000..18000 |
| 12 | GPS speed, mph x 10 | 0.. |
| 20 | Passive Wi-Fi scan (blocks about 2 s, receive only, no probe requests, up to 16 results) | count, -1 on failure |
| 21 | Scan result a: RSSI dBm | dBm |
| 22 | Scan result a: channel | channel |
| 23 | Scan result a: auth mode (ESP-IDF `wifi_auth_mode_t`) | number |
| 26 | Draw SSID of scan result a at window (b, c), white | characters drawn |
| 40 | Fox carrier on: a = MHz, b = power 0..32 | 1 started, 0 refused |
| 41 | Fox carrier off | 1 |
| 42 | Fox carrier running? | 1 / 0 |
| 50 | Largest free internal heap block, KB | KB |

SD card calls (whole card, read and write; paths are relative to the card root and `..` is refused). Names live in the app's own
memory, one character per word, 0 terminated; the **STR** opcode (61, `Asm.str`) stores an inline string there. "Address" means a
word index into that memory.

| fn | Call | Returns |
|----|------|---------|
| 30 | Open file: a = name address, b = 0 read, 1 write (truncates), 2 append. One file open at a time; opening closes the previous | 1 ok, 0 fail |
| 31 | Open file size, low 16 bits | 0..65535 as int16 |
| 32 | Open file size, high 16 bits | |
| 33 | Read b bytes into memory at a, one byte per word | bytes read (0 at end), -1 bad range |
| 34 | Write b bytes from memory at a (low 8 bits of each word) | bytes written, -1 bad range |
| 35 | Seek: a = offset low 16 bits, b = offset high 16 bits | 1 / 0 |
| 36 | Close | 1 |
| 37 | List a folder: a = name address, empty string = card root. Keeps the first 16 entries, folders first | count, -1 failure |
| 38 | Copy entry a's name to memory at b | name length, -1 |
| 39 | Entry a is a folder? | 1 / 0, -1 |
| 43 | Entry a size in KB | KB, -1 |
| 44 | File exists: a = name address | 1 / 0 |

Drawing helpers: **27** draws the 0-terminated string stored at address a at window (b, c); **29** sets the colour (0..7) used by 26 and 27.

Leaving the app (hold CENTER) stops the fox carrier and frees the scan results.

Leaving the app also closes the open file.

Not available: Bluetooth (the controller needs more RAM than this board has free), arbitrary 802.11 frame transmit, raw I/O.

Examples: `sdcard_examples/APPS/WIFISCAN.APP` (`tools/make_wifiscan.py`) and `SDLIST.APP` (`tools/make_sdlist.py`, lists the card and previews a file). See `tools/appasm.py`, `Asm.sys` and `Asm.str`.
