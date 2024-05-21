#include "Adafruit_Arcada.h"

#include <stdint.h>

#include "src/gameobject.hpp"

#define SECONDS(VOID) millis() / 1000

/* globals */
Adafruit_Arcada arcada;
uint16_t *framebuffer;

int width, height;

class test : public GameObject {
public:
    test() : GameObject(0, 0) {}

    void tick(float dt) { Serial.println("Test tick!"); }
    void render() { Serial.println("Test Render!"); }
};

/* statics */
static const float tick_interval = 0.05;
static float acc = 0.00, curr_time, elapsed = 0.0;
static int frames = 0, ticks = 0;

void setup(void) {
    while (!Serial); delay(100); // wait for coms
    Serial.begin(9600);

    curr_time = SECONDS();

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

    GameObject::add_obj(new test());
}

void loop(void) {
    float new_time = SECONDS();
    float frame_time = new_time - curr_time;
    curr_time = new_time;

    acc += frame_time;
    while (acc >= tick_interval) { // fixed timestep
        Serial.printf("FPS: %f, TPS: %f, Elapsed: %f\n", frames / elapsed, ticks / elapsed, elapsed);

        GameObject::tick_objs(tick_interval);

        if (arcada.readButtons() & ARCADA_BUTTONMASK_A) {
            GameObject::destroy_obj(1);
        }

        /* tick game */
        acc -= tick_interval;
        elapsed += tick_interval;
        ++ticks;
    }


    /* clear framebuffer */
    memset(framebuffer, 0x00, width * height * sizeof(uint16_t));
    
    /* render game */
    GameObject::render_objs();

    arcada.blitFrameBuffer(0, 0, false, false);
    ++frames;
}