# Remote ID decoder - third-party source

`RemoteIdDecoder.{h,cpp}` and `RemoteIdModel.{h,cpp}` are copied **unmodified** from
ESP32 Marauder (https://github.com/justcallmekoko/ESP32Marauder, `esp32_marauder/`), MIT licence,
Copyright (c) 2020 Just Call Me Koko. The licence text is in `LICENSE-ESP32Marauder`.

They are the same decoder Marauder uses for Wi-Fi Beacon, Wi-Fi NAN and BLE service-data
ASTM F3411 / Open Drone ID messages. They have no Arduino dependency (only `<cstring>`, `<cmath>`,
`<climits>`, `<cstdio>`), so they are reused as-is and the firmware does not carry a second,
incompatible parser. `../rid.cpp` is this project's adapter: the observation/merge logic follows
Marauder's `WiFiScanRemoteId.h` (`processRemoteIdWifiFrame`, `processRemoteIdBlePayload`,
`mergeRemoteIdRecord`) but without Arduino types, SD logging or operator-location handling.

Deliberate difference: the decoder can parse the broadcast "System" message, which contains an operator
location field. This firmware never stores, shows, logs or maps it (see `rid.cpp`, `merge()`).
