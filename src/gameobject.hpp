#ifndef __GAMEOBJECT_HPP__
#define __GAMEOBJECT_HPP__

#include <unordered_map>

const unsigned MAX_OBJS = 500;

class GameObject {
public:
    GameObject(void);
    GameObject(float x, float y);
    ~GameObject();

    virtual void tick(float dt) = 0;
    virtual void render() = 0;

    inline void deactivate() { this->is_active = false; }
    inline float getX() const { return this->x; }
    inline float getY() const { return this->y; }
    inline int getID() const { return this->ID; }

    static bool add_obj(GameObject *);
    static unsigned destroy_obj(unsigned ID);

    static void tick_objs(float dt);
    static void render_objs();
protected:
    unsigned ID;
    float x, y;
    bool is_active;
private:
    static unsigned CURR_OBJ;
    static std::unordered_map<unsigned, GameObject*> objs;
};

#endif