# GPS_MqttsPublish Example

## Purpose
- Periodic GPS → MQTT over TLS for SIM7600/A7670/A7608 with built-in GPS.
- Home Assistant–friendly topics plus LED control and on-demand power telemetry.

## MQTT Topics
- `Sim7600/gps` (publish): JSON with `lat`, `lon`, `alt`, `speed`, `accuracy`, `timestamp` (ISO-8601 Z).
- `Sim7600/availability` (publish): `"offline"` at startup or when GPS fix is lost; `"online"` when a fix is (re)acquired.
- `Sim7600/status` (publish on request): JSON with `battery_mv` and, when available, `solar_mv`, `modem_mv`.
- `Sim7600/command` (subscribe):
  - `"on"` → LED on GPIO12 ON
  - `"off"` → LED OFF
  - `"status"` → sample ADCs and publish to `Sim7600/status`

## SPIFFS Files
- `/config.ini` – MQTT settings
  - `mqtt_server=<hostname>`
  - `mqtt_port=<port>` (default 8883 if blank)
  - `mqtt_username=<username>`
  - `mqtt_password=<password>`
- `/ca.cer` – PEM root CA for the MQTT broker

## Behavior
- Loads MQTT credentials and CA from SPIFFS; if missing/invalid, drops into AT passthrough mode (Serial ↔ SerialAT) with a one-time banner and skips MQTT/GPS logic.
- MQTT over TLS (port from config, default 8883); client ID `SIM7600-GPS`; LWT `Sim7600/availability` = `"offline"`.
- Publishes GPS every 60s; retries fixes every 15s when missing and marks availability offline.
- LED defaults OFF; responds to `Sim7600/command`.
- On `status` command, reads ADCs and publishes to `Sim7600/status`.

## Dependencies
- `utilities.h` (board pins/power, SerialAT setup).
- TinyGSM fork `<TinyGsmClient.h>` (modem/MQTT/GPS).
- SPIFFS/FS `<SPIFFS.h>` for `/config.ini` and `/ca.cer`.
- TLS: CA loaded at runtime from `/ca.cer`.
- ADC helpers `<esp32-hal-adc.h>` for `analogReadMilliVolts`.
- Arduino core/ESP32 toolchain.

## Hardware Notes
- LED on GPIO12.
- Battery ADC requires `BOARD_BAT_ADC_PIN`; solar ADC requires `BOARD_SOLAR_ADC_PIN`; modem voltage via `modem.getBattVoltage()` when `MODEM_CONNECTED_ADC_PIN` is defined. ADC uses 12-bit width, 11 dB attenuation; readings doubled for the divider.
- GPS requires a modem variant with built-in GPS (not SIM7080G concurrently with LTE).
