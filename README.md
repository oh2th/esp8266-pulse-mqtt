# ESP8266 Pulse MQTT Water Meter

This project is a pulse counter for ESP8266, designed for water meters (or any pulse-based metering). It reads pulses on pin D6 (with 50ms debounce), calculates water flow and total consumption, and reports the data to an MQTT broker in JSON format. Configuration is managed via a web portal or an initial config file.

## Features

- **Pulse Input:** Reads pulses on D6 (10 litres/pulse by default, configurable).
- **Debounce:** 50ms debounce for reliable counting.
- **MQTT Reporting:** Publishes JSON to a configurable topic prefix + `/state` every 60 seconds:

  ```json
  {
    "uptime": 12345,
    "measure_water": 1.23,
    "meter_water": 0.456
  }
  ```

  - `uptime`: seconds since boot
  - `measure_water`: flow rate in litres/minute
  - `meter_water`: total consumption in m³ (three decimals)

- **MQTT Control:** Subscribe to `<topic_prefix>/set` and send a float or `{"meter_water": <value>}` to set the meter reading.
- **Web Portal:** Accessible at `/config` for WiFi, MQTT, topic, pulse/litre, and meter value configuration. Portal is always available (captive portal if WiFi fails).
- **Persistence:**
  - `meter_water` is stored in EEPROM (only updated if changed by 0.01 m³).
  - Full config is stored in LittleFS (updated every 24 hours or via portal).
- **Startup Logging:** Serial output shows configuration and MQTT connection status.

## Configuration

You can configure the device in two ways:

### 1. Initial Configuration File

Create a `data` folder in your project root and add a `config.json` file like this:

```json
{
  "wifi_ssid": "your_wifi_ssid",
  "wifi_pass": "your_wifi_password",
  "mqtt_host": "mqtt.example.com",
  "mqtt_port": 1883,
  "meter_water": 0.0,
  "litres_per_pulse": 10,
  "mqtt_topic": "watermeter/kitchen"
}
```

Upload this file to the device's filesystem with:

```sh
pio run -t uploadfs
```

This will pre-load your configuration so the device can connect to WiFi and MQTT on first boot.

### 2. Web Portal

On first boot or if no config is found, the device starts in AP mode (`PulseMeterConfig`). Connect to this WiFi and browse to [http://192.168.4.1/config](http://192.168.4.1/config) to set up:

- WiFi SSID and password
- MQTT broker and port
- MQTT topic prefix (e.g. `watermeter/kitchen`)
- Litres per pulse
- Initial meter value (m³)

You can revisit `/config` at any time to change settings.

## MQTT Topics

- **Publish:** `<topic_prefix>/state`
- **Subscribe:** `<topic_prefix>/set`
  - Accepts either a float (e.g. `12.345`) or JSON (`{"meter_water": 12.345}`) to set the meter value.

## Build & Flash

1. Install [PlatformIO](https://platformio.org/) and required libraries:

   - ESP Async WebServer
   - ESP Async TCP
   - AsyncMqttClient
   - ArduinoJson
   - Ticker

2. Clone this repo and edit `platformio.ini` for your board if needed.

3. Build and upload:

   ```sh
   pio run -t upload
   ```

4. Open serial monitor for logs:

   ```sh
   pio device monitor
   ```

5. (Optional) Upload initial configuration:

   ```sh
   pio run -t uploadfs
   ```

## Hardware

- ESP8266 board (e.g. NodeMCU)
- Pulse output from water meter (dry contact or open collector to D6, GND)

## Notes

- Avoid excessive writes to EEPROM/LittleFS to prevent flash wear.
- If WiFi fails, device starts a captive portal for configuration.
- All configuration changes via the portal trigger an automatic reboot.
