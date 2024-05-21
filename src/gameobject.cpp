#include "gameobject.hpp"

unsigned GameObject::CURR_OBJ = 0;
std::unordered_map<unsigned, GameObject*> GameObject::objs = std::unordered_map<unsigned, GameObject*>();

GameObject::GameObject(void) : GameObject(0, 0) {
    this->is_active = false;
}


GameObject::GameObject(float x, float y) {
    this->x = x;
    this->y = y;
    this->ID = ++GameObject::CURR_OBJ;
    this->is_active = true;
}


GameObject::~GameObject() { /* UNIMPLEMENTED */ }

void GameObject::tick_objs(float dt) {
    for (auto [id, obj] : GameObject::objs) {
        if (!obj->is_active) {
            GameObject::destroy_obj(id);
            continue;
        }

        obj->tick(dt);
    }
}

void GameObject::render_objs() {
    for (auto [id, obj] : GameObject::objs) {
        obj->render();
    }
}

bool GameObject::add_obj(GameObject *obj) {
    return GameObject::objs.insert(std::make_pair(obj->ID, obj)).second;
}

unsigned GameObject::destroy_obj(unsigned ID) {
    return GameObject::objs.erase(ID);
}