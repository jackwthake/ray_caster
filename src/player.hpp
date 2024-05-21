#ifndef __PLAYER_HPP__
#define __PLAYER_HPP__

#include "gameobject.hpp"

class Player : public GameObject {
public:
    Player(float x, float y);

    void tick(float dt) override;
    void render() override;

private:
    std::pair<double, double> dir;
    std::pair<double, double> cam_plane;

    const float move_spd = 3, rot_speed = 5;
};

#endif