#include "Adafruit_Arcada.h"

#include <FastLED.h>
#include <stdint.h>

/* globals */
Adafruit_Arcada arcada;
uint16_t *framebuffer;

int width, height, ticks = 0, frames = 0;
struct player_t *player;

/* constants */
static const int JOYSTICK_THRESH = 20; // Joystick Values range from -512 to 511
static const unsigned long TPS_TARGET = 20;

const int game_map[8][8] = {
    {1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 1, 0, 1, 1},
    {1, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 1, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1},
};


struct player_t {
    float x = 2.0, y = 2.0;
    
    /* camera vars */
    std::pair<double, double> dir = std::make_pair(-1, 0.0);
    std::pair<double, double> cam_plane = std::make_pair(0.0, 0.66);

    const float move_speed = 1.0, rot_speed = 1.0;
};


/* convert rgb values to  RGB 565 format */
static uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t BGRColor = b >> 3;
    BGRColor         |= (g & 0xFC) << 3;
    BGRColor         |= (r & 0xF8) << 8;

    return BGRColor;
}


/* set pixel color in framebuffer*/
static void set_pixel(int x, int y, uint16_t color) {
    framebuffer[y * width  + x] = color;
}


/* draw a vertical line at a given x coordinate with a given height centered on the screen's y-axis */
static void draw_line(int x, int line_height, uint16_t color) {
    int draw_start = -line_height / 2 + height / 2;
    int draw_end = line_height / 2 + height / 2;

    if (draw_start < 0) draw_start = 0;
    if (draw_end >= height) draw_end = height - 1;

    for (int y = draw_start; y < draw_end; ++y) {
        set_pixel(x, y, color);
    }
}


/***
 * Player Code
*/
void player_tick(float move_scaler) {
    uint32_t buttons = arcada.readButtons();
    int16_t joystick_x = arcada.readJoystickX();
    int16_t joystick_y = arcada.readJoystickY();

    double old_dir = player->dir.first, old_plane = player->cam_plane.first;

    /* modulate speed by the joystick */
    double rot_speed = (player->rot_speed * move_scaler) * (1 + abs(joystick_x) / 512) * (joystick_x > 0 ? -1 : 1);
    double mov_spd = (player->move_speed * move_scaler) * (1 + abs(joystick_y) / 512) * (joystick_y > 0 ? -1 : 1);

    /* Look left and right */
    if (joystick_x < -JOYSTICK_THRESH || joystick_x > JOYSTICK_THRESH) {
        player->dir = std::make_pair(player->dir.first * cos(rot_speed) - player->dir.second * sin(rot_speed),
                                     old_dir * sin(rot_speed) + player->dir.second * cos(rot_speed));

        player->cam_plane = std::make_pair(player->cam_plane.first * cos(rot_speed) - player->cam_plane.second * sin(rot_speed),
                                           old_plane * sin(rot_speed) + player->cam_plane.second * cos(rot_speed));
    }

    if (joystick_y < -JOYSTICK_THRESH || joystick_y > JOYSTICK_THRESH) {
        if(game_map[(int)(player->y)][(int)(player->x + player->dir.first * mov_spd)] < 1) {
            player->x += player->dir.first * mov_spd;
        }
        if(game_map[(int)(player->y + player->dir.second * mov_spd)][(int)(player->x)] < 1) {
            player->y += player->dir.second * mov_spd;
        }
    }
}


void render() {
    for (int x = 0; x < width; ++x) {
        double camera_x = 2 * x / (double)width - 1;
        std::pair<double, double> ray_dir(player->dir.first + player->cam_plane.first * camera_x,
                                          player->dir.second + player->cam_plane.second * camera_x);
        
        /* DDA ALgo Variables */
        std::pair<int, int> map_pos((int)player->x, (int)player->y);
        std::pair<double, double> side_dist(0.0, 0.0);
        std::pair<double, double> delta_dist((ray_dir.first == 0) ? 1e30 : fabs(1 / ray_dir.first),
                                             (ray_dir.second == 0 ? 1e30 : fabs(1 / ray_dir.second)));

        double wall_dist = 0.0;
        bool hit = false;
        int side;

        std::pair<int, int> step(ray_dir.first < 0 ? -1 : 1,
                                 ray_dir.second < 0 ? -1 : 1);
        
        if (ray_dir.first < 0) {
            side_dist.first = (player->x - map_pos.first) * delta_dist.first;
        } else {
            side_dist.first = (map_pos.first + 1.0 - player->x) * delta_dist.first;
        }

        if (ray_dir.second < 0) {
            side_dist.second = (player->y - map_pos.second) * delta_dist.second;
        } else {
            side_dist.second = (map_pos.second + 1.0 - player->y) * delta_dist.second;
        }

        /* DDA Algo */
        while (!hit) {
            /* Step to next square*/
            if (side_dist.first < side_dist.second) {
                side_dist.first += delta_dist.first;
                map_pos.first += step.first;
                side = 0;
            } else {
                side_dist.second += delta_dist.second;
                map_pos.second += map_pos.second;
                side = 1;
            }

            /* Check for hit */
            if (game_map[map_pos.second][map_pos.first] > 0)
                hit = true;
        }

        /* Calc distance depending on side */
        if (side == 0)
            wall_dist = side_dist.first - delta_dist.first;
        else 
            wall_dist = side_dist.second - delta_dist.second;

        /* Draw column to screen */
        draw_line(x, (int)(height / wall_dist), side == 0 ? rgb(150, 0, 150) : rgb(75, 0, 150));
    }
}


/****
 * Main Functions
*/
void setup(void) {
    while (!Serial); delay(100); // wait for coms
    Serial.begin(9600);

    if (!arcada.arcadaBegin()) { // init libraries
        Serial.println("Failed to initialize Adafruit_Arcada.h");
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

    player = new player_t();
}


void loop(void) {
    /* Tick the game approx every 20ms */
    EVERY_N_MILLISECONDS(TPS_TARGET) {
        player_tick(0.075);

        ++ticks;
    };

    /* Report debug info */
    EVERY_N_SECONDS(1) {
        Serial.printf("FPS: %f, TPS: %f, Elapsed: %lds\n", 
                      (frames / (float)millis()) * 1000, 
                      (ticks / (float)millis()) * 1000, 
                      millis() / 1000);
        Serial.printf("x: %f, y: %f\n", player->x, player->y);
    };


    /* clear framebuffer */
    memset(framebuffer, 0x00, width * height * sizeof(uint16_t));
    render();
    /* render game */
    arcada.blitFrameBuffer(0, 0, false, false);
    ++frames;
}