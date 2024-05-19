#include "Adafruit_Arcada.h"

#include <stdint.h>

/* globals */
Adafruit_Arcada arcada;
uint16_t *framebuffer;

int width, height;

/* statics */
static const double tick_interval = .01;
static double acc = 0.00, curr_time;

void setup(void) {
    while (!Serial); delay(100); // wait for coms
    Serial.begin(9600);

    if (!arcada.arcadaBegin()) { // init libraries
        Serial.println("Failed to initialize Adafruit_Arcada.");
        for (;;);
    }

    arcada.displayBegin(); // init screen
    arcada.setBacklight(255);

    width = arcada.display->width(); // init framebuffer
    height = arcada.display->height();
    if (!arcada.createFrameBuffer(width, height)) {
        Serial.printf("Failed to initialize screenbuffer of size %d x %d\n", width, height);
    }

    Serial.printf("Screenbuffer initialized to %d x %d", width, height);
    curr_time = millis();
}

void loop(void) {
    double new_time = millis();
    double frame_time = new_time - curr_time;
    double fps = 1.0 / frame_time;
    curr_time = new_time;

    acc += frame_time;
    while (acc >= tick_interval) {
        /* tick game */
        Serial.printf("FPS: %f\n", fps);
    }

    /* clear framebuffer */
    memset(framebuffer, 0x00, width * height * sizeof(uint16_t));
    /* render game */
    arcada.blitFrameBuffer(0, 0, false, false);
}