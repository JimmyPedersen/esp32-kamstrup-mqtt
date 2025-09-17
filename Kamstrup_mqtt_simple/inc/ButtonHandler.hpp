#pragma once

#include <Arduino.h>
#include <ArduinoLog.h>
#include <functional>
#include <esp_log.h>

enum ClickType {
    NO_CLICK,
    SINGLE_CLICK,
    DOUBLE_CLICK,
    TRIPLE_CLICK,
    LONG_PRESS_3S_RELEASED,
    LONG_PRESS_5S_RELEASED,
    LONG_PRESS_10S_RELEASED,
    LONG_PRESS_3S_HELD,
    LONG_PRESS_5S_HELD,
    LONG_PRESS_10S_HELD
};

class ButtonHandler {
public:
    // Callback type: void(ClickType)
    using Callback = std::function<void(ClickType)>;

    // Constructor: Initialize with button pin and pressed level (true for HIGH, false for LOW)
    // Example: ButtonHandler buttonHandler(2, true); // Button on pin 2
    ButtonHandler(int buttonPin, bool pressedLevel = false);

    // Set callback to be called on click or long press detection (on release or held)
    void setCallback(Callback cb);

    // Check button state and return ClickType
    // This function should be called in the loop() to detect button clicks
    ClickType checkButtonClick();

private:
    static constexpr const char* TAG = "LEDs";

    int _buttonPin;
    bool _pressedLevel; // true = HIGH, false = LOW
    unsigned long _lastDebounceTime = 0;
    unsigned long _lastClickTime = 0;
    int _clickCount = 0;
    int _buttonState;
    int _lastButtonState;
    bool _longPressDetected = false;
    unsigned long _buttonPressStart = 0;
    int _lastLongPressLevel = 0; // 0 = none, 1 = 3s, 2 = 5s, 3 = 10s
    const unsigned long DEBOUNCE_DELAY = 50; // ms
    const unsigned long CLICK_INTERVAL = 500; // ms (Max time between clicks for multi-click)
    Callback _callback = nullptr;
    unsigned long _lastPressDuration = 0; // <-- Add this line
};

inline ButtonHandler::ButtonHandler(int buttonPin, bool pressedLevel)
    : _buttonPin(buttonPin), _pressedLevel(pressedLevel)
{
    pinMode(_buttonPin, INPUT_PULLUP);
    _buttonState =  _pressedLevel ? HIGH : LOW;
    _lastButtonState = !_buttonState; // Start with the opposite state
}

inline void ButtonHandler::setCallback(Callback cb) {
    _callback = cb;
}


inline ClickType ButtonHandler::checkButtonClick() {

    pinMode(_buttonPin, INPUT_PULLUP);
    vTaskDelay(pdMS_TO_TICKS(1)); // Run every 1ms
    int reading = digitalRead(_buttonPin);
    ClickType detectedClick = NO_CLICK;

    if (reading != _lastButtonState) {
        _lastDebounceTime = millis();
    }

    if ((millis() - _lastDebounceTime) > DEBOUNCE_DELAY) {
        if (reading != _buttonState) {
            _buttonState = reading;

            if (_buttonState == (_pressedLevel ? HIGH : LOW)) { // Button pressed
                _clickCount++;
                _lastClickTime = millis();
                _buttonPressStart = _lastClickTime;
                _longPressDetected = false;
                _lastLongPressLevel = 0;
            } else if (_buttonState == (_pressedLevel ? LOW : HIGH)) { // Button released
                _lastPressDuration = millis() - _buttonPressStart;
                unsigned long pressDuration = _lastPressDuration;
                if (_lastLongPressLevel == 3) {
                    detectedClick = LONG_PRESS_10S_RELEASED;
                    ESP_LOGD(TAG, "Detected LONG_PRESS_10S_RELEASED");
                } else if (_lastLongPressLevel == 2) {
                    detectedClick = LONG_PRESS_5S_RELEASED;
                    ESP_LOGD(TAG, "Detected LONG_PRESS_5S_RELEASED");
                } else if (_lastLongPressLevel == 1) {
                    detectedClick = LONG_PRESS_3S_RELEASED;
                    ESP_LOGD(TAG, "Detected LONG_PRESS_3S_RELEASED");
                }
                if (detectedClick != NO_CLICK) {
                    _clickCount = 0;
                    _longPressDetected = true;
                    _lastLongPressLevel = 0;
                    if (_callback) _callback(detectedClick);
                    _lastButtonState = reading;
                    return detectedClick;
                }
            }
        }
    }

    // Check for long press while button is held down
    if (_buttonState == (_pressedLevel ? HIGH : LOW)) {
        unsigned long heldTime = millis() - _buttonPressStart;
        if (heldTime >= 10000 && _lastLongPressLevel < 3) {
            detectedClick = LONG_PRESS_10S_HELD;
            ESP_LOGD(TAG, "Detected LONG_PRESS_10S_HELD");
            _lastLongPressLevel = 3;
            _longPressDetected = true;
            _clickCount = 0;
            if (_callback) _callback(detectedClick);
            return detectedClick;
        } else if (heldTime >= 5000 && _lastLongPressLevel < 2) {
            detectedClick = LONG_PRESS_5S_HELD;
            ESP_LOGD(TAG, "Detected LONG_PRESS_5S_HELD");
            _lastLongPressLevel = 2;
            _longPressDetected = true;
            _clickCount = 0;
            if (_callback) _callback(detectedClick);
            return detectedClick;
        } else if (heldTime >= 3000 && _lastLongPressLevel < 1) {
            detectedClick = LONG_PRESS_3S_HELD;
            ESP_LOGD(TAG, "Detected LONG_PRESS_3S_HELD");
            _lastLongPressLevel = 1;
            _longPressDetected = true;
            _clickCount = 0;
            if (_callback) _callback(detectedClick);
            return detectedClick;
        }
    }

    if (reading != _lastButtonState && _buttonState == (_pressedLevel ? LOW : HIGH)) {
        _lastLongPressLevel = 0;
    }

    // Check for multi-click ONLY if button is released and not a long press
    if (_clickCount > 0 && (millis() - _lastClickTime) > CLICK_INTERVAL && !_longPressDetected
        && _buttonState == (_pressedLevel ? LOW : HIGH)) {
        if (_clickCount == 1) {
            if (_lastPressDuration < CLICK_INTERVAL) {
                detectedClick = SINGLE_CLICK;
                ESP_LOGD(TAG, "Detected SINGLE_CLICK");
            }
        } else if (_clickCount == 2) {
            detectedClick = DOUBLE_CLICK;
            ESP_LOGD(TAG, "Detected DOUBLE_CLICK");
        } else if (_clickCount >= 3) {
            detectedClick = TRIPLE_CLICK;
            ESP_LOGD(TAG, "Detected TRIPLE_CLICK");
        }
        if (detectedClick != NO_CLICK && _callback) _callback(detectedClick);
        _clickCount = 0;
    }

    _lastButtonState = reading;
    return detectedClick;
}