// 渲染器类，它实现了所有光线追踪的操作。
#pragma once
#include "Scene.hpp"

struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
    void Render(const Scene& scene);

private:
};