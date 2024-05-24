#include "Adafruit_Arcada.h"

#include <stdint.h>

/* globals */
Adafruit_Arcada arcada;
uint16_t *framebuffer;

int width, height;

/* statics */
static const int JOYSTICK_THRESH = 20; // Joystick Values range from -512 to 511
static const unsigned long tick_interval = 20;
static unsigned long last_refr = 0.0;

struct player_t *player;

int game_map[8][8] = {
    {1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1},
};


struct player_t {
    float x = 1.0, y = 1.0;
    
    std::pair<double, double> dir = std::make_pair(-1, 0.0);
    std::pair<double, double> cam_plane = std::make_pair(0.0, 0.66);

    const float move_speed = 1.0, rot_speed = 1.0;
};


static uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t BGRColor = b >> 3;
    BGRColor         |= (g & 0xFC) << 3;
    BGRColor         |= (r & 0xF8) << 8;

    return BGRColor;
}


static void set_pixel(int x, int y, uint16_t color) {
    framebuffer[y * width  + x] = color;
}


static void draw_line(int x, int line_height, uint16_t color) {
    int draw_start = -line_height / 2 + height / 2;
    int draw_end = line_height / 2 + height / 2;

    if (draw_start < 0) draw_start = 0;
    if (draw_end >= height) draw_end = height - 1;

    for (int y = draw_start; y < draw_end; ++y) {
        framebuffer[y * width + x] = color;
    }
}


/***
 * Player Code
*/
void player_tick() {
    uint32_t buttons = arcada.readButtons();
    int16_t joystick_x = arcada.readJoystickX();
    int16_t joystick_y = arcada.readJoystickY();

    double old_dir = player->dir.first, old_plane = player->cam_plane.first;
    double rot_speed = (player->rot_speed) /** abs(joystick_x) / 512*/ * (joystick_x > 0 ? -1 : 1);
    double mov_spd = (player->move_speed) /** abs(joystick_y) / 512*/;

    /* Look left and right */
    if (joystick_x < -JOYSTICK_THRESH || joystick_x > JOYSTICK_THRESH) {
        player->dir = std::make_pair(player->dir.first * cos(rot_speed) - player->dir.second * sin(rot_speed),
                                           old_dir * sin(rot_speed) + player->dir.second * cos(rot_speed));
        player->cam_plane = std::make_pair(player->cam_plane.first * cos(rot_speed) - player->cam_plane.second * sin(rot_speed),
                                                     old_plane * sin(rot_speed) + player->cam_plane.second * cos(rot_speed));
    }

    /* TODO: FIx Movement */
    if (joystick_y > JOYSTICK_THRESH) {
        if (game_map[(int)player->y][(int)(player->x + player->dir.first * mov_spd)] == false)
            player->x += player->dir.first * mov_spd;
        if (game_map[(int)(player->y * player->dir.second * mov_spd)][(int)player->x] == false)
            player->y += player->dir.second * mov_spd;
    }

    if (joystick_y < -JOYSTICK_THRESH) {
        if (game_map[(int)player->y][(int)(player->x + player->dir.first * -mov_spd)] == false)
            player->x += player->dir.first * -mov_spd;
        if (game_map[(int)(player->y * player->dir.second * -mov_spd)][(int)player->x] == false)
            player->y += player->dir.second * -mov_spd;
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
            if (game_map[map_pos.first][map_pos.second] > 0)
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

    player = new player_t();
}

void loop(void) {
    if (millis() - last_refr >= tick_interval) {
        Serial.println(millis() - last_refr);
        last_refr += tick_interval;

        player_tick();
    }


    /* clear framebuffer */
    memset(framebuffer, 0x00, width * height * sizeof(uint16_t));
    render();
    /* render game */
    arcada.blitFrameBuffer(0, 0, false, false);
}