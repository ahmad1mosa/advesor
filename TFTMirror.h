#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_SSD1306.h>
#include <limits.h>

// Simple mirroring helper to copy Adafruit_SSD1306 1bpp buffer to an Adafruit_ST7789 TFT
// without changing existing OLED drawing code. Call flush() whenever you would have
// called display.display().

class TFTMirror {
public:
  TFTMirror(int8_t tftCsPin, int8_t tftDcPin, int8_t tftRstPin, int8_t tftBacklightPin = -1)
  : _tft(tftCsPin, tftDcPin, tftRstPin),
    _ssd(nullptr),
    _blk(tftBacklightPin),
    _tftWidth(240),
    _tftHeight(280),
    _xOffset(INT16_MIN),
    _yOffset(INT16_MIN),
    _inited(false) {}

  // spiSck/spiMiso/spiMosi/spiSs default to the pins you requested: SCK=22, MISO=-1, MOSI=21, SS=-1
  void begin(uint16_t tftWidth = 240,
             uint16_t tftHeight = 280,
             uint8_t rotation = 0,
             int8_t spiSck = 22,
             int8_t spiMiso = -1,
             int8_t spiMosi = 21,
             int8_t spiSs = -1) {
    _tftWidth = tftWidth;
    _tftHeight = tftHeight;
    SPI.begin(spiSck, spiMiso, spiMosi, spiSs);
    _tft.init(_tftWidth, _tftHeight);
    _tft.setRotation(rotation);
    _tft.fillScreen(0x0000); // black
    if (_blk >= 0) {
      pinMode(_blk, OUTPUT);
      digitalWrite(_blk, HIGH); // turn backlight on
    }
    _inited = true;
  }

  void attachDisplay(Adafruit_SSD1306* ssd1306) {
    _ssd = ssd1306;
  }

  // Optionally override the centering offsets (top-left corner where 128x64 is drawn)
  void setOffsets(int16_t xOffset, int16_t yOffset) {
    _xOffset = xOffset;
    _yOffset = yOffset;
  }

  // Simple backlight helpers
  void setBacklight(bool on) {
    if (_blk >= 0) {
      digitalWrite(_blk, on ? HIGH : LOW);
    }
  }

  // Copy 1-bpp SSD1306 buffer to 16-bit color TFT (white on black)
  void flush() {
    if (!_inited || _ssd == nullptr) {
      return;
    }

    // Keep the OLED updating as usual
    _ssd->display();

    uint8_t* buf = _ssd->getBuffer();
    const int srcW = _ssd->width();   // typically 128
    const int srcH = _ssd->height();  // typically 64

    // Center by default unless offsets were set explicitly
    const int16_t xOffset = (_xOffset == INT16_MIN) ? (int16_t)((_tftWidth  - srcW) / 2) : _xOffset;
    const int16_t yOffset = (_yOffset == INT16_MIN) ? (int16_t)((_tftHeight - srcH) / 2) : _yOffset;

    // Clear the region we are about to draw into (optional)
    _tft.fillRect(xOffset, yOffset, srcW, srcH, 0x0000);

    // Draw white pixels where buffer bits are set
    // SSD1306 buffer is arranged in vertical bytes: each byte has 8 vertical pixels
    for (int y = 0; y < srcH; y++) {
      int byteRow = (y >> 3) * srcW;
      uint8_t bitMask = (uint8_t)(1u << (y & 7));
      for (int x = 0; x < srcW; x++) {
        if (buf[byteRow + x] & bitMask) {
          _tft.drawPixel(xOffset + x, yOffset + y, 0xFFFF); // white
        }
      }
    }
  }

private:
  Adafruit_ST7789 _tft;
  Adafruit_SSD1306* _ssd;
  int8_t _blk;
  uint16_t _tftWidth;
  uint16_t _tftHeight;
  int16_t _xOffset;
  int16_t _yOffset;
  bool _inited;
};

