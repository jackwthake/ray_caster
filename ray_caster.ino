#include "Adafruit_Arcada.h"

#include <stdint.h>

#include "src/gameobject.hpp"
#include "src/player.hpp"

#define SECONDS(VOID) millis() / 1000

/* globals */
Adafruit_Arcada arcada;
uint16_t *framebuffer;
int width, height;


void setup(void) {
    //while (!Serial); delay(100); // wait for coms
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

    framebuffer = arcada.getFrameBuffer();
    Serial.printf("Screenbuffer initialized to %d x %d\n", width, height);

    GameObject::add_obj(new Player(2, 2));
}

void loop(void) {
    GameObject::tick_objs(0.01);

    /* clear framebuffer */
    memset(framebuffer, 0x00, width * height * sizeof(uint16_t));
    
    /* render game */
    GameObject::render_objs();

    arcada.blitFrameBuffer(0, 0, false, false);
}