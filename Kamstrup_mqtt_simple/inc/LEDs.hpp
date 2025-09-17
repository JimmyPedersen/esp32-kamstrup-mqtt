#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "esp_log.h"

class LEDs : public Adafruit_NeoPixel {
public:
    // Enums of colors for the LED (Add more colors if needed)
    enum  LEDColor{
        ColorDef = -1,
        ColorOff = 0x000000, // Off color
        ColorW = 0xFFFFFF,   // White color
        ColorW25 = 0x404040,   // White color (25%)
        ColorR = 0xFF0000,   // Red color
        ColorG = 0x00FF00,   // Green color
        ColorB = 0x0000FF,   // Blue color
        ColorY = 0xFFFF00,   // Yellow color
        ColorC = 0x00FFFF,   // Cyan color
        ColorM = 0xFF00FF,   // Magenta color
        ColorO = 0xFFA500,   // Orange color
        ColorP = 0x800080    // Purple color
    };  

    struct RgbColor {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };

    // ESP-IDF log tag
    static constexpr const char* TAG = "LEDs";

    // Constructor: specify number of LEDs and pin
    LEDs(uint16_t numPixels, uint8_t pin) 
        : Adafruit_NeoPixel(numPixels, pin, NEO_GRB + NEO_KHZ800),   _pin(pin), _pixelCount(numPixels), _isDirty(true) {
        // Initialize the strip with the specified number of pixels and pin
//        this->begin();
            _DefaultColorAfterBlinks = ColorOff;
    }

    // Override the base class begin() method
    void Begin() {
    //    esp_log_level_set(TAG, ESP_LOG_DEBUG);
        ESP_LOGI(TAG, "LED strip initialized with %d pixels on pin %d", _pixelCount, _pin);

        // Call the base class begin method
        Adafruit_NeoPixel::begin();
        _isDirty = true; // Mark as dirty after initialization
    }


    // Getter for _DefaultColor2
    LEDColor getDefaultColor2() const {
        return _DefaultColor2;
    }

    // Setter for _DefaultColor2
    void setDefaultColor2(LEDColor color) {
        _DefaultColor2 = color;
    }

    // Getter for _DefaultColorAfterBlinks
    LEDColor getDefaultColorAfterBlinks() const {
        return _DefaultColorAfterBlinks;
    }

    // Setter for _DefaultColorAfterBlinks
    void setDefaultColorAfterBlinks(LEDColor color) {
        _DefaultColorAfterBlinks = color;
    }

// Override the base class setPixelColor
    void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
        // Call the base class method
        Adafruit_NeoPixel::setPixelColor(n, r, g, b);
        // Set the dirty flag
        _isDirty = true;
    }

    // Override the base class setPixelColor with uint32_t color
    void setPixelColor(uint16_t n, uint32_t c) {
        // Call the base class method
        Adafruit_NeoPixel::setPixelColor(n, c);
        // Set the dirty flag
        _isDirty = true;
    }

    // Custom method to set a single pixel color using an RgbColor struct
    void setPixelColor(uint16_t n, const RgbColor& color) {
        // Call the base class method (or our overloaded version)
        Adafruit_NeoPixel::setPixelColor(n, color.r, color.g, color.b);
        // Set the dirty flag
        _isDirty = true;
    }

    // Custom method to set a range of pixels to the same color
    void fillPixels(uint16_t first, uint16_t count, const RgbColor& color) {
        for (uint16_t i = 0; i < count; ++i) {
            if (first + i < _pixelCount) {
                // Call our overloaded setPixelColor, which sets the dirty flag
                setPixelColor(first + i, color);
            }
        }
    }

    // Custom method to set all pixels to the same color
    void fillAll(const RgbColor& color) {
        fillPixels(0, _pixelCount, color);
    }

    // Override the base class begin() method
    void begin() {
        ESP_LOGI(TAG, "LED strip: begin");
         Adafruit_NeoPixel::begin();
         _isDirty = true; // Clearing is a change
    }

    // Override the base class clear() method
    void clear() {
         Adafruit_NeoPixel::clear();
         _isDirty = true; // Clearing is a change
    }


    // Override the base class show() method to check the dirty flag
    void show() {
        if (_isDirty) {
            _isDirty = false;          // Reset the dirty flag after showing

            // Set pin as output to control the LED strip
            pinMode(_pin, OUTPUT);
            vTaskDelay(1); // Small delay to ensure pin is set before showing
//           ESP_LOGD(TAG, "Updating LED strip on pin %d", _pin);

//            unsigned long start = millis();
            Adafruit_NeoPixel::show(); // Call the base class show()
//            unsigned long end = millis();
//            ESP_LOGD(TAG, "LED strip updated in %lu ms", end - start);

            vTaskDelay(1); // Small delay to ensure pin is set before showing

            // Set pin back to input to allow for button press detection
            pinMode(_pin, INPUT_PULLUP);
        }
    }

    // Set color of a single pixel
    void setColor(LEDColor color, uint16_t pixel = 0) {
//        ESP_LOGD(TAG, "Changed color on led: %u to: %u", pixel, color);
        _changeTS = millis(); // Update the timestamp for the next blink
        if (pixel < Adafruit_NeoPixel::numPixels()) {
             //Adafruit_NeoPixel::fill(color, 0, Adafruit_NeoPixel::numPixels()); // Fill the entire strip with the specified color
            Adafruit_NeoPixel::setPixelColor(pixel, color);
            _isDirty = true; // Mark the strip as dirty (needs to be updated)
        }       
    };

    // Set color of a single pixel and show immediately
    void setColorNow(LEDColor color, uint16_t pixel = 0) {
        setColor(color, pixel);
        show();
    }

    // Function to handle the blinking of the LED
    void blinkHandler() {
        if (_blinks.left > 0 || _blinks.left == -1) { // Check if there are blinks left or infinite blinks
            if (_blinks.on) {
               if((millis() - _changeTS) < _blinks.onTime)
                    return;
 //               ESP_LOGD(TAG, "Blinking on for %d ms", _blinks.onTime);
 //               _changeTS = millis(); // Update the timestamp for the next blink
                if(_blinks.left > 0)
                    _blinks.left--; // Decrease the number of blinks left

                _blinks.on = false; // Set the blinking state to off

                if(_blinks.left == 0)    
                    setColorNow((_blinks.ColorAfterBlinks == ColorDef ? _DefaultColorAfterBlinks : _blinks.ColorAfterBlinks ), _blinks.pixel); // Set second color (Or color after blinks)
                else
                    setColorNow( _blinks.color2, _blinks.pixel); // Set second color (Or color after blinks)
            } else {
                if((millis() - _changeTS) < _blinks.offTime)
                    return;

                _blinks.on = true; // Set the blinking state to on
                setColorNow(_blinks.color, _blinks.pixel); // Set the color to the specified color
//                _changeTS = millis(); // Update the timestamp for the next blink
            }
        }
    }


    // Set up a pixel for blinking with a specified color and blink count
    // count = -1 for infinite blinking
    void Blink(int8_t count, enum LEDColor newColor, int16_t blinkTime = 1000, int16_t pauseTime = -1, uint8_t pixel = 0, enum LEDColor Color2 = ColorOff, enum LEDColor ColorAfterBlinks = ColorDef) {
        //ESP_LOGD(TAG, "Blinking %d times with color 0x%08X on led %u (%u/%u)", count, newColor, pixel, blinkTime, pauseTime);
        _blinks.pixel = pixel; // Set the pin number for the LED strip
        _blinks.color = newColor; // Set the color for blinking
        _blinks.onTime = blinkTime; // Set the time for which the LED should be on
        _blinks.offTime = pauseTime >= 0 ? pauseTime : blinkTime; // Set the time for which the LED should be off
        _blinks.color2 = Color2; // Set the color2 for blinking
        _blinks.ColorAfterBlinks = ColorAfterBlinks; // Set the color after blinking

        if (newColor != ColorOff) {
            _blinks.left = count; // Set the number of blinks left
            _blinks.on = true; // Set the blinking state to on
            _currentColor = newColor; // Set the current color to the new color        
        } else {
            _blinks.left = 0;    // We're done blinking, so set the number of blinks left to 0
            _blinks.on = false; // Set the blinking state to off
            
        }
        setColorNow(_blinks.color,  _blinks.pixel); // Set the color to the specified color
        
       // blinkHandler(); // Call the blink handler to start blinking
    }
/*
    void Blink2(int8_t count, enum LEDColor newColor, int16_t blinkTime = 1000, int16_t pauseTime = -1) {
        Blink(count, newColor, blinkTime, pauseTime, 0, _DefaultColor2, _DefaultColorAfterBlinks); // Call the overloaded Blink method with default ColorAfterBlinks
    }
*/
    // Function to get the number of blinks left
    int8_t getBlinksLeft() const {
        return _blinks.left;
    }

    // Handler function to be called in the main loop
    void Handler(void){
        blinkHandler();     // Call the blink handler
        show();             // Show the updated LED strip (if needed)
    }

private:
    // Structure to hold the blinking state and parameters
    struct Blinks {
        int8_t left;            // Number of blinks left (-1 for infinite)
        enum LEDColor color;            // Color to blink
        enum LEDColor color2;           // Color2 when blinking
        enum LEDColor ColorAfterBlinks; // Color after blinks blink
        int16_t onTime;        // Time to stay on
        int16_t offTime;       // Time to stay off
        uint8_t pixel;          // pixel number on the LED strip
        bool on;                // Blinking state    
    };

    Blinks _blinks;               // Blinking state

    enum LEDColor _currentColor; // Current color of the LED strip
    enum LEDColor _lastColor;    // Last color of the LED strip
    enum LEDColor _DefaultColor2;           // Color2 when blinking
    enum LEDColor _DefaultColorAfterBlinks; // Color after blinks blink
    uint32_t _changeTS;          // Timestamp of the last color change
    uint8_t _pin;                // Pin number for the LED strip
    volatile bool _isDirty;      // Flag to indicate if the strip needs to be updated
    uint16_t _pixelCount;         // Number of pixels in the strip
};