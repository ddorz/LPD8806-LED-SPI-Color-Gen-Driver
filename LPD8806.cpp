/* ****************************************************************************
 * Library created for controlling LPD8806-based LED strips, with a number of *
 * functions for generating different colors.                                 *
 *                                                                            *
 * Copyright (C) Adafruit Industries                                          *
 * MIT License                                                                *
 *                                                                            *
 * Copyright (C) Dave Dorzback                                                *
 * MIT License                                                                *
 *                                                                            *
/******************************************************************************/

#include "SPI.h"
#include "LPD8806.h"

// Constructor for use with hardware SPI (specific clock/data pins)
LPD8806::LPD8806(uint16_t n) {
  pixels = NULL;
  begun  = false;
  updateLength(n);
  updatePins();
}

// Constructor for use with arbitrary clock/data pins:
LPD8806::LPD8806(uint16_t n, uint8_t dpin, uint8_t cpin) {
  pixels = NULL;
  begun  = false;
  updateLength(n);
  updatePins(dpin, cpin);
}

// If using this constructor, MUST follow up with updateLength()
// and updatePins() to establish the strip length and output pins
LPD8806::LPD8806(void) {
  numLEDs = numBytes = 0;
  pixels  = NULL;
  begun   = false;
  updatePins(); // Must assume hardware SPI until pins are set
}

// Activate hard/soft SPI as appropriate
void LPD8806::begin(void) {
  if(hardwareSPI == true) startSPI();
  else                    startBitbang();
  begun = true;
}

// Change pin assignments post-constructor, switching to hardware SPI
void LPD8806::updatePins(void) {
  hardwareSPI = true;
  datapin     = clkpin = 0;
  // If begin() was previously invoked, init the SPI hardware now:
  if(begun == true) startSPI();
  // Otherwise, SPI is NOT initted until begin() is explicitly called.

  // Note: any prior clock/data pin directions are left as-is and are
  // NOT restored as inputs!
}

// Change pin assignments post-constructor, using arbitrary pins:
void LPD8806::updatePins(uint8_t dpin, uint8_t cpin) {

  datapin     = dpin;
  clkpin      = cpin;
  clkport = dataport = 0;
  clkpinmask = datapinmask = 0;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined(__AVR_ATmega8__) || (__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  clkport     = portOutputRegister(digitalPinToPort(cpin));
  clkpinmask  = digitalPinToBitMask(cpin);
  dataport    = portOutputRegister(digitalPinToPort(dpin));
  datapinmask = digitalPinToBitMask(dpin);
#endif

  if(begun == true) { // If begin() was previously invoked...
    // If previously using hardware SPI, turn that off:
    if(hardwareSPI == true) SPI.end();
    startBitbang(); // Regardless, now enable 'soft' SPI outputs
  } // Otherwise, pins are not set to outputs until begin() is called.

  // Note: any prior clock/data pin directions are left as-is and are
  // NOT restored as inputs!

  hardwareSPI = false;
}

#ifndef SPI_CLOCK_DIV8
  #define SPI_CLOCK_DIV8 4
#endif

// Enable SPI hardware and set up protocol details
void LPD8806::startSPI(void) {
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  SPI.setClockDivider(SPI_CLOCK_DIV8);  // 2 MHz
  // SPI bus is run at 2MHz.

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined(__AVR_ATmega8__) || (__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)

  // Issue initial latch/reset to strip:
  SPDR = 0; // Issue initial byte
  for(uint16_t i=((numLEDs+31)/32)-1; i>0; i--) {
    while(!(SPSR & (1<<SPIF))); // Wait for prior byte out
    SPDR = 0;                   // Issue next byte
  }
#else
  SPI.transfer(0);
  for(uint16_t i=((numLEDs+31)/32)-1; i>0; i--) {
    SPI.transfer(0);
  }
#endif
}

// Enable software SPI pins and issue initial latch
void LPD8806::startBitbang() {
  pinMode(datapin, OUTPUT);
  pinMode(clkpin , OUTPUT);
  if (dataport != 0) {
    // use low level bitbanging when we can
    *dataport &= ~datapinmask; // Data is held low throughout (latch = 0)
    for(uint16_t i=((numLEDs+31)/32)*8; i>0; i--) {
      *clkport |=  clkpinmask;
      *clkport &= ~clkpinmask;
    }
  } else {
    // can't do low level bitbanging, revert to digitalWrite
    digitalWrite(datapin, LOW);
    for(uint16_t i=((numLEDs+31)/32)*8; i>0; i--) {
      digitalWrite(clkpin, HIGH);
      digitalWrite(clkpin, LOW);
    }
  }
}

// Change strip length (see notes with empty constructor, above)
void LPD8806::updateLength(uint16_t n) {
  uint8_t latchBytes = (n + 31) / 32;
  if(pixels != NULL) free(pixels); // Free existing data (if any)
  numLEDs    = n;
  n         *= 3; // 3 bytes per pixel
  numBytes   = n + latchBytes;
  if(NULL != (pixels = (uint8_t *)malloc(numBytes))) { // Alloc new data
    memset( pixels   , 0x80, n);          // Init to RGB 'off' state
    memset(&pixels[n], 0   , latchBytes); // Clear latch bytes
  } else numLEDs = numBytes = 0; // else malloc failed
  // 'begun' state does not change -- pins retain prior modes
}

uint16_t LPD8806::numPixels(void) {
  return numLEDs;
}

// This is how data is pushed to the strip
void LPD8806::show(void) {
  uint8_t  *ptr = pixels;
  uint16_t i    = numBytes;

  // This doesn't need to distinguish among individual pixel color
  // bytes vs. latch data, etc.  Everything is laid out in one big
  // flat buffer and issued the same regardless of purpose.
  if(hardwareSPI) {
    while(i--) {
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined(__AVR_ATmega8__) || (__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
      while(!(SPSR & (1<<SPIF))); // Wait for prior byte out
      SPDR = *ptr++;              // Issue new byte
#else
      SPI.transfer(*ptr++);
#endif
    }
  } else {
    uint8_t p, bit;

    while(i--) {
      p = *ptr++;
      for(bit=0x80; bit; bit >>= 1) {
	if (dataport != 0) {
	  if(p & bit) *dataport |=  datapinmask;
	  else        *dataport &= ~datapinmask;
	  *clkport |=  clkpinmask;
	  *clkport &= ~clkpinmask;
	} else {
	  if (p&bit) digitalWrite(datapin, HIGH);
	  else digitalWrite(datapin, LOW);
	  digitalWrite(clkpin, HIGH);
	  digitalWrite(clkpin, LOW);
	}
      }
    }
  }
}

// Convert separate R,G,B into combined 32-bit GRB color
uint32_t LPD8806::Color(byte r, byte g, byte b) {
  return ((uint32_t)(g | 0x80) << 16) |
         ((uint32_t)(r | 0x80) <<  8) |
                     b | 0x80 ;
}

// Set pixel color from separate 7-bit R, G, B components
void LPD8806::setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
  if(n < numLEDs) { // Arrays are 0-indexed, thus NOT '<='
    uint8_t *p = &pixels[n * 3];
    *p++ = g | 0x80;
    *p++ = r | 0x80;
    *p++ = b | 0x80;
  }
}

// Set pixel color from 'packed' 32-bit GRB (not RGB) value
void LPD8806::setPixelColor(uint16_t n, uint32_t c) {
  if(n < numLEDs) {
    uint8_t *p = &pixels[n * 3];
    *p++ = (c >> 16) | 0x80;
    *p++ = (c >>  8) | 0x80;
    *p++ =  c        | 0x80;
  }
}

// Query color from previously-set pixel (returns packed 32-bit GRB value)
uint32_t LPD8806::getPixelColor(uint16_t n) {
  if(n < numLEDs) {
    uint16_t ofs = n * 3;
    return ((uint32_t)(pixels[ofs    ] & 0x7f) << 16) |
           ((uint32_t)(pixels[ofs + 1] & 0x7f) <<  8) |
            (uint32_t)(pixels[ofs + 2] & 0x7f);
  }

  return 0; // Pixel # is out of bounds
}

// Standard color wheel with 384 positions. Sets n-th pixel
void LPD8806::Wheel(uint16_t n, uint16_t pos) {
    if (n >= numLEDs) {
        return;
    }
    
    byte r, g, b;
    enum {RG, GB, BR};
    
    // Cycles through 3 transitions (RG, GB, BR), with the first
    // color decreasing and the second increasing.
    switch(pos / 128) {
        case RG: r = 127 - pos % 128, g = pos % 128, b = 0;  break;
        case GB: g = 127 - pos % 128, b = pos % 128, r = 0;  break;
        case BR: b = 127 - pos % 128, r = pos % 128, g = 0;  break;
    }
    
    // Combine the R, G, B components into packed 32-bit color
    uint32_t c = ((((g * 256) & 0xff00) << 8) | ((r * 256) & 0xff00) | ((b * 256) >> 8));
    
    // Set pixel color
    uint8_t *p = &pixels[n * 3];
    *p++ = (c >> 16) | 0x80;
    *p++ = (c >>  8) | 0x80;
    *p++ =  c        | 0x80;
}

// Identical to the above function, but with additional parameters for
// value (brightness) and saturation. Range for v/s => 0-255
void LPD8806::bsWheel(uint16_t n, uint16_t pos, byte v, byte s) {
    if (n >= numLEDs) {
        return;
    }
    
    byte r, g, b;
    long s1 = (s+1), v1 = (v+1);
    enum {RG, GB, BR};
    
    switch(pos / 128) {
        case 0: r = 127 - pos % 128, g = pos % 128, b = 0;  break;
        case 1: g = 127 - pos % 128, b = pos % 128, r = 0;  break;
        case 2: b = 127 - pos % 128, r = pos % 128, g = 0;  break;
    }
    
    // Add saturation value into each component
    r = 255 - (((255 - r) * s1) >> 8);
    g = 255 - (((255 - g) * s1) >> 8);
    b = 255 - (((255 - b) * s1) >> 8);
    
    // Add value (brightness) value into each component and combine the R, G, B components
    uint32_t c = ((((g * v1) & 0xff00) << 8) | ((r * v1) & 0xff00) | ((b * v1) >> 8));
    
    // Set pixel
    uint8_t *p = &pixels[n * 3];
    *p++ = (c >> 16) | 0x80;
    *p++ = (c >>  8) | 0x80;
    *p++ =  c        | 0x80;
    
}

// Generate colors using sine function
void LPD8806::sinWheel(uint16_t n, uint16_t pos) {
    if (n >= numLEDs) {
        return;
    }
    
    uint16_t r, g, b;
    
    // Generate R, G, B components from sine function
    pos%=6284;
    r = 63.5 + sin(pos *  0.001) * 63.5;
    g = 63.5 + sin((pos * 0.001) + 2.09) * 63.5;
    b = 63.5 + sin((pos * 0.001) + 4.18) * 63.5;
    
    // Combine the R, G, B components and return a single 32-bit value
    uint32_t c = ((((g * 256) & 0xff00) << 8) | ((r * 256) & 0xff00) | ((b * 256) >> 8));
    
    // Set pixel
    uint8_t *p = &pixels[n * 3];
    *p++ = (c >> 16) | 0x80;
    *p++ = (c >>  8) | 0x80;
    *p++ =  c        | 0x80;
}

// Generate colors using cosine function. Generates the same colors
// as the above function, but with an offset 'pos'
void LPD8806::cosWheel(uint16_t n, uint16_t pos) {
    if (n >= numLEDs) {
        return;
    }
    
    uint16_t r, g, b;
    
    // Generate R, G, B components from sine function
    pos%=6284;
    r = 63.5 + cos(pos *  0.001) * 63.5;
    g = 63.5 + cos((pos * 0.001) + 2.09) * 63.5;
    b = 63.5 + cos((pos * 0.001) + 4.18) * 63.5;
    
    // Combine the R, G, B components
    uint32_t c = ((((g * 256) & 0xff00) << 8) | ((r * 256) & 0xff00) | ((b * 256) >> 8));
    
    // Set pixel
    uint8_t *p = &pixels[n * 3];
    *p++ = (c >> 16) | 0x80;
    *p++ = (c >>  8) | 0x80;
    *p++ =  c        | 0x80;
}

// Generates colors using sine with parameters for value/sat
void LPD8806::bssinWheel(uint16_t n, uint16_t pos, byte v, byte s) {
    if (n >= numLEDs) {
        return;
    }
    
    uint16_t r, g, b;
    long v1 = (v+1), s1 = (s+1);
    
    // Generate R, G, B components from sine function
    pos%=6284;
    r = 63.5 + sin(pos *  0.001) * 63.5;
    g = 63.5 + sin((pos * 0.001) + 2.09) * 63.5;
    b = 63.5 + sin((pos * 0.001) + 4.18) * 63.5;
    
    // Add saturation
    r = 255 - (((255 - r) * s1) >> 8);
    g = 255 - (((255 - g) * s1) >> 8);
    b = 255 - (((255 - b) * s1) >> 8);
    
    // Combine R, G, B components
    uint32_t c = ((((g * v1) & 0xff00) << 8) | ((r * v1) & 0xff00) | ((b * v1) >> 8));
    
    // Set pixel
    uint8_t *p = &pixels[n * 3];
    *p++ = (c >> 16) | 0x80;
    *p++ = (c >>  8) | 0x80;
    *p++ =  c        | 0x80;
}

// Generates colors using cosine with parameters for value/sat
void LPD8806::bscosWheel(uint16_t n, uint16_t pos, byte v, byte s) {
    if (n >= numLEDs) {
        return;
    }
    
    uint16_t r, g, b;
    long v1 = (v+1), s1 = (s+1);
    
    // Generate R, G, B components from sine function
    pos%=6284;
    r = 63.5 + sin(pos *  0.001) * 63.5;
    g = 63.5 + sin((pos * 0.001) + 2.09) * 63.5;
    b = 63.5 + sin((pos * 0.001) + 4.18) * 63.5;
    
    // Add saturation
    r = 255 - (((255 - r) * s1) >> 8);
    g = 255 - (((255 - g) * s1) >> 8);
    b = 255 - (((255 - b) * s1) >> 8);
    
    // Combine R, G, B components
    uint32_t c = ((((g * v1) & 0xff00) << 8) | ((r * v1) & 0xff00) | ((b * v1) >> 8));
    
    // Set pixel
    uint8_t *p = &pixels[n * 3];
    *p++ = (c >> 16) | 0x80;
    *p++ = (c >>  8) | 0x80;
    *p++ =  c        | 0x80;
}

// Hue, value, saturation color wheel
void LPD8806::hsvWheel(uint16_t n, int16_t h, byte v, byte s) {
    if (n >= numLEDs) {
        return;
    }
    
    byte r, g, b, lo;
    long  s1 = (s+1), v1 = (v+1);
    
    h %= 1536;
    if (h < 0) h += 1536;
    lo = h & 255;           // Low byte  = primary/secondary color mix
    
    switch (h >> 8) {       // High byte = sextant of colorwheel
        case 0: r = 255,      g = lo,       b = 0;        break; // R to Y
        case 1: r = 255 - lo, g = 255,      b = 0;        break; // Y to G
        case 2: r = 0,        g = 255,      b = lo;       break; // G to C
        case 3: r = 0,        g = 255 - lo, b = 255;      break; // C to B
        case 4: r = lo,       g = 0,        b = 255;      break; // B to M
        default: r = 255,     g = 0,        b = 255 - lo; break; // M to R
    }
    
    // Saturation
    r = 255 - (((255 - r) * s1) >> 8);
    g = 255 - (((255 - g) * s1) >> 8);
    b = 255 - (((255 - b) * s1) >> 8);
    
    // Value (brightness)
    uint32_t c = ((((g * v1) & 0xff00) << 8) | ((r * v1) & 0xff00) | ((b * v1) >> 8));
    
    // Set pixel
    uint8_t *p = &pixels[n * 3];
    *p++ = (c >> 16) | 0x80;
    *p++ = (c >>  8) | 0x80;
    *p++ =  c        | 0x80;
}

// Hue, value, saturation color wheel with multiple wheel lines
void LPD8806::hsvMultiWheel(uint16_t n, int16_t h, byte v, byte s, uint16_t line) {
    if (n >= numLEDs) {
        return;
    }
    
    byte r, g, b, lo;
    long  s1 = (s+1), v1 = (v+1);
    enum {RGB, RG, GB, BR};
    
    switch (line) {                 // Hue
        case RGB:
            h %= 1536;
            if (h < 0) h += 1536;
            lo = h & 255;           // Low byte  = primary/secondary color mix
            switch (h >> 8) {       // High byte = sextant of colorwheel
                case 0: r = 255,      g = lo,       b = 0;        break; // R to Y
                case 1: r = 255 - lo, g = 255,      b = 0;        break; // Y to G
                case 2: r = 0,        g = 255,      b = lo;       break; // G to C
                case 3: r = 0,        g = 255 - lo, b = 255;      break; // C to B
                case 4: r = lo,       g = 0,        b = 255;      break; // B to M
                default: r = 255,     g = 0,        b = 255 - lo; break; // M to R
            }
            break;
        case RG: //RG Line only
            h %= 1024;
            if(h < 0) h += 1024;
            lo = h & 255;           // Low byte  = primary/secondary color mix
            switch(h >> 8) {        // High byte = sextant of colorwheel
                case 0: r = 255,        g = lo,       b = 0; break; // R to Y
                case 1: r = 255 - lo,   g = 255,      b = 0; break; // Y to G
                case 2: r = lo,         g = 255,      b = 0; break; // G to Y
                default: r = 255,       g = 255 - lo, b = 0; break; // Y to R
            }
            break;
        case GB: //GB Line only
            h %= 1024;
            if(h < 0) h += 1024;
            lo = h & 255;           // Low byte  = primary/secondary color mix
            switch (h >> 8) {       // High byte = sextant of colorwheel
                case 0: r = 0,  g = 255,      b = lo;       break; // G to C
                case 1: r = 0,  g = 255 - lo, b = 255;      break; // C to B
                case 2: r = 0,  g = lo,       b =  255;     break; // B to C
                default: r = 0, g = 255,      b = 255 - lo; break; // C to G
            }
            break;
        case BR: //BR Line only
            h %= 1024;
            if (h < 0) h += 1024;
            lo = h & 255;           // Low byte  = primary/secondary color mix
            switch (h >> 8) {       // High byte = sextant of colorwheel
                case 0: r = lo,         g = 0, b =  255;     break; // B to M
                case 1: r = 255,        g = 0, b = 255 - lo; break; // M to R
                case 2: r = 255,        g = 0, b =  lo;      break; // R to M
                default: r = 255 - lo,  g = 0,  b =  255;    break; // M to B
            }
            break;
    }
    
    // Saturation
    r = 255 - (((255 - r) * s1) >> 8);
    g = 255 - (((255 - g) * s1) >> 8);
    b = 255 - (((255 - b) * s1) >> 8);
    
    // Value (brightness)
    uint32_t c = ((((g * v1) & 0xff00) << 8) | ((r * v1) & 0xff00) | ((b * v1) >> 8));
    
    // Set pixel
    uint8_t *p = &pixels[n * 3];
    *p++ = (c >> 16) | 0x80;
    *p++ = (c >>  8) | 0x80;
    *p++ =  c        | 0x80;
}
