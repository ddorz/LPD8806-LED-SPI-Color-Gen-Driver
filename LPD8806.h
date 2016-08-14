#if (ARDUINO >= 100)
#   include <Arduino.h>
#else
#   include <WProgram.h>
#   include <pins_arduino.h>
#endif

class LPD8806 {
    
public:
    // LPD8806 strip functions
    LPD8806(uint16_t n, uint8_t dpin, uint8_t cpin); // Configurable pins
    LPD8806(uint16_t n);                             // Use SPI hardware; specific pins only
    LPD8806(void);                                   // Empty constructor; init pins & strip length later
    void begin(void);
    void show(void);
    void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
    void setPixelColor(uint16_t n, uint32_t c);
    void updatePins(uint8_t dpin, uint8_t cpin);    // Change pins, configurable
    void updatePins(void);                          // Change pins, hardware SPI
    void updateLength(uint16_t n);                  // Change strip length
    uint16_t numPixels(void);
    uint32_t Color(byte r, byte g, byte b);
    uint32_t getPixelColor(uint16_t n);
    // Color generation functions (pixel 'n' set)
    void Wheel(uint16_t n, uint16_t pos);
    void bsWheel(uint16_t n, uint16_t pos, byte v, byte s);
    void sinWheel(uint16_t n, uint16_t pos);
    void cosWheel(uint16_t n, uint16_t pos);
    void bssinWheel(uint16_t n, uint16_t pos, byte v, byte s);
    void bscosWheel(uint16_t n, uint16_t pos, byte v, byte s);
    void hsvWheel(uint16_t n, int16_t h, byte v, byte s);
    void hsvMultiWheel(uint16_t n, int16_t h, byte v, byte s, uint16_t line);
    
private:
    uint8_t *pixels;                                  // Holds LED color values (3 bytes each) + latch
    uint8_t clkpin, datapin, clkpinmask, datapinmask; // Clock & data pin numbers & PORT bitmasks
    uint16_t numLEDs, numBytes;                       // Number of RGB LEDs and size of 'pixels' buffer
    volatile uint8_t *clkport, *dataport;             // Clock & data PORT registers
    boolean hardwareSPI, begun;                       // Indicate if hardware SPI / if begin() previously invoked
    // SPI hardware/software setups
    void startSPI(void);
    void startBitbang(void);
    
};
