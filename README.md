# atomvm_sensors

I2C sensor drivers for [AtomVM](https://github.com/atomvm/AtomVM).

## Sensors

| Module | Sensor | Measurements |
|--------|--------|--------------|
| `bme680` | Bosch BME680 | Temperature, pressure, humidity, gas resistance (VOC) |
| `sgp30` | Sensirion SGP30 | eCO2 (ppm), TVOC (ppb) |
| `veml6030` | Vishay VEML6030 | Ambient light (lux), white channel |
| `i2c_scanner` | — | Bus scan utility, prints detected devices |

## I2C Bus Configuration

All sensors share a single I2C bus:
- **SDA:** GP4
- **SCL:** GP5
- **Speed:** 100 kHz
- **Peripheral:** I2C0

Default I2C addresses:
- `0x10` — VEML6030
- `0x58` — SGP30
- `0x76` — BME680

## Usage

```erlang
%% Open the I2C bus
I2C = i2c:open([{sda, 4}, {scl, 5}, {peripheral, 0}, {clock_speed_hz, 100000}]),

%% BME680: temperature, pressure, humidity
{ok, Bme} = bme680:init(I2C, 16#76),
{ok, TempC, PressHpa, HumRh} = bme680:read(Bme),

%% SGP30: air quality
{ok, Sgp} = sgp30:init(I2C),
{ok, ECO2, TVOC} = sgp30:measure(Sgp),  %% call once per second

%% VEML6030: ambient light
Veml = veml6030:init(I2C, 16#10),
{ok, Lux} = veml6030:read_lux(Veml),
```

## Adding as a dependency

In your `rebar.config`:

```erlang
{deps, [
    {atomvm_sensors, {git, "https://github.com/etnt/atomvm_sensors.git", {branch, "main"}}}
]}.
```

## Platform

Tested on AtomVM v0.7.0-alpha.1 running on Raspberry Pi Pico (RP2040),
as well as AtomVM (master) on ESP32-S3.

## License

Apache-2.0 OR LGPL-2.1-or-later
