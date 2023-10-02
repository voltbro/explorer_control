#pragma once

class I_HMI {
public:
    virtual void send_beep(float duration, float frequency) = 0;
    virtual void send_color(uint8_t r, uint8_t g, uint8_t b) = 0;
};
