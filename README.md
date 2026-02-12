# ESP32-S3 Dual Eye Renderer

Dual 3D eye renderer for ESP32-S3 Supermini using 1.8" SPI TFT displays.

## Hardware
- ESP32-S3 Supermini
- 1.8" ST7735 TFT (128x160, green tab)
- SPI @ 40MHz
- Short (~1 cm) wiring

## Wiring
| Signal | GPIO |
|------|------|
| RST | 4 |
| DC | 5 |
| CS (Left) | 16 |
| CS (Right) | 8 |
| SCK | 6 |
| MOSI | 7 |
| BL | 3.3V |
| VCC | 3.3V or 5V (module dependent) |
| GND | GND |

## Status
- Display init verified
- Single-eye render testing
- Dual-eye realism tuning in progress
