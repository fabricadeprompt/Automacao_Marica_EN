# Building and Flashing

## Toolchain

- **Arduino IDE** with the **ESP32 (Espressif)** board package installed via Boards
  Manager.
- **Board:** ESP32 Dev Module (DevKit V1) for the three sensor/relay boxes (Water,
  Pump, Control).
- **External libraries for those three: none.** All dependencies (`WiFi`,
  `Preferences`, `ArduinoOTA`, `esp_now`, `esp_wifi`, `esp_task_wdt`, `HardwareSerial`,
  `WebServer`, `HTTPClient`) ship with the Arduino ESP32 core — nothing to install via
  the Library Manager. The PZEM-004T's Modbus RTU protocol is implemented manually in
  the firmware, with no third-party library.
- **The Cardputer (`main_cardputer.cpp`) is different.** It's not a standalone DevKit —
  it needs the **M5Stack** board package (Boards Manager) and the **M5Cardputer**
  library (Library Manager, which already pulls in M5Unified/M5GFX as a dependency).
  When compiling this file, select the "M5Cardputer" board, not "ESP32 Dev Module".

## Step by step

1. Clone the repository.
2. `cp secrets.h.example secrets.h` and fill in your Wi-Fi network (used by the
   Control box, for the OTA window and Supabase upload) and, optionally, your Supabase
   project — if left blank, the firmware detects it and disables telemetry sending
   automatically.
3. In `marica_protocol.h`, replace the placeholder MACs (`MAC_AGUA`, `MAC_BOMBA`,
   `MAC_CONTROLE`, `MAC_CARDPUTER`) with your boards' real MACs — find them by running
   `Serial.println(WiFi.macAddress())` in `setup()` before starting ESP-NOW.
4. Adjust the fixed IPs (`IP_CONTROLE`, `IP_BOMBA`, `IP_AGUA`, `IP_GATEWAY`,
   `IP_MASCARA`, `IP_DNS`) to match your local network range.
5. Compile and flash the four `.cpp` files — `main_agua.cpp`, `main_bomba.cpp`,
   `main_controle.cpp` on the "ESP32 Dev Module" board, and `main_cardputer.cpp` on the
   "M5Cardputer" board.

## Flashing order

On a fresh install, the order between the three sensor/relay boxes doesn't matter —
they all start from the same `marica_protocol.h`. The coordinated-order rule (sender
before receiver) only applies **later**, when updating a shared struct that's already
running in the field — see `docs/PROTOCOL.md`.

## ESP-NOW channel

`CANAL_SEGURANCA_PADRAO` (channel 2) is explicitly fixed before `esp_now_init()`, in a
specific sequence (promiscuous → set_channel → promiscuous off) validated in external
review. Changing this sequence without repeating that review is not recommended.
