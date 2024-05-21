#include "player.hpp"

#include "Adafruit_Arcada.h"

extern Adafruit_Arcada arcada;
extern int width, height;
extern uint16_t *framebuffer;

static int JOYSTICK_THRESH = 20;

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


Player::Player(float x, float y) : GameObject(x, y) {
    this->dir = std::make_pair(-1, 0.0);
    this->cam_plane = std::make_pair(0.0, 0.66);
}


void Player::tick(float dt) {
    uint32_t buttons = arcada.readButtons();
    int16_t joystick_x = arcada.readJoystickX();
    int16_t joystick_y = arcada.readJoystickY();

    double old_dir = this->dir.first, old_plane = this->cam_plane.first;
    double rot_speed = (this->rot_speed * dt) * abs(joystick_x) / 512 * (joystick_x > 0 ? -1 : 1);
    double mov_spd = (this->move_spd * dt) * abs(joystick_y) / 512;

    /* Look left and right */
    if (joystick_x < -JOYSTICK_THRESH || joystick_x > JOYSTICK_THRESH) {
        this->dir = std::make_pair(this->dir.first * cos(rot_speed) - this->dir.second * sin(rot_speed),
                                           old_dir * sin(rot_speed) + this->dir.second * cos(rot_speed));
        this->cam_plane = std::make_pair(this->cam_plane.first * cos(rot_speed) - this->cam_plane.second * sin(rot_speed),
                                                     old_plane * sin(rot_speed) + this->cam_plane.second * cos(rot_speed));
    }

    /* TODO: FIx Movement */
    if (joystick_y > JOYSTICK_THRESH) {
        // if (game_map[(int)this->y][(int)(this->x + this->dir.first * mov_spd)] == false)
            this->x += this->dir.first * mov_spd;
        // if (game_map[(int)(this->y * this->dir.second * mov_spd)][(int)this->x] == false)
            this->y += this->dir.second * mov_spd;
    }

    if (joystick_y < -JOYSTICK_THRESH) {
        // if (game_map[(int)this->y][(int)(this->x + this->dir.first * -mov_spd)] == false)
            this->x += this->dir.first * -mov_spd;
        // if (game_map[(int)(this->y * this->dir.second * -mov_spd)][(int)this->x] == false)
            this->y += this->dir.second * -mov_spd;
    }
}


void Player::render() {
    int x;
    for (x = 0; x < width; ++x) {
        double camera_x = 2 * x / (double)width - 1;
        std::pair<double, double> ray_dir(this->dir.first + this->cam_plane.first * camera_x,
                                          this->dir.second + this->cam_plane.second * camera_x);
        
        /* DDA ALgo Variables */
        std::pair<int, int> map_pos((int)this->x, (int)this->y);
        std::pair<double, double> side_dist(0.0, 0.0);
        std::pair<double, double> delta_dist((ray_dir.first == 0) ? 1e30 : fabs(1 / ray_dir.first),
                                             (ray_dir.second == 0 ? 1e30 : fabs(1 / ray_dir.second)));

        double wall_dist = 0.0;
        bool hit = false;
        int side;

        std::pair<int, int> step(ray_dir.first < 0 ? -1 : 1,
                                 ray_dir.second < 0 ? -1 : 1);
        
        if (ray_dir.first < 0) {
            side_dist.first = (this->x - map_pos.first) * delta_dist.first;
        } else {
            side_dist.first = (map_pos.first + 1.0 - this->x) * delta_dist.first;
        }

        if (ray_dir.second < 0) {
            side_dist.second = (this->y - map_pos.second) * delta_dist.second;
        } else {
            side_dist.second = (map_pos.second + 1.0 - this->y) * delta_dist.second;
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
        int line_height = (int)(height / wall_dist);
        int draw_start = -line_height / 2 + height / 2;
        int draw_end = line_height / 2 + height / 2;

        if (draw_start < 0) draw_start = 0;
        if (draw_end >= height) draw_end = height - 1;

        for (int y = draw_start; y < draw_end; ++y) {
            uint16_t color = side == 1 ? ARCADA_BLUE : ARCADA_RED;
            framebuffer[y * width + x] = color;
        }
    }
}