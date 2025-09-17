#include <stdint.h>
#include <ArduinoLog.h>

////////////////////////////////  Helpers  /////////////////////////////////////////
class Logger : public Logging {
  public:
    Logger() : Logging() {}

    static int getCurrentTime(char* timeBuffer, size_t bufferSize) {
        if (bufferSize < 9) { // Ensure the buffer is large enough for "HH:MM:SS\0"
            return 0;
        }

        struct tm timeInfo;
        if (!getLocalTime(&timeInfo)) {
            return snprintf(timeBuffer, bufferSize, "00:00:00"); // Default to 00:00:00 if time is unavailable
        }

        return snprintf(timeBuffer, bufferSize, "%02d:%02d:%02d", timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
    }

    static void printTimestamp(Print *_logOutput, int level) {
        char c[32];
        int pos = getCurrentTime(c, sizeof(c) - 1);
        int m = snprintf(&c[pos], sizeof(c) - pos - 1, " (%10lu) ", millis());
        _logOutput->print(c);
    }

    void configure() {
        setShowLevel(true);
        setPrefix(printTimestamp);
        setSuffix(NULL); // No suffix needed
        notice("Logging initialized");
    }
};

// Optionally, provide a global instance
extern Logger AppLog;