#ifndef SCENE_H
#define SCENE_H 
#include "Mathutils.h"
#include "Particle.h"
#include "cstring"
#include "esp_heap_caps.h"
#include "ST77916.h"
#include <vector>
#include <ctime>
#include "esp_attr.h"
#include "esp_lcd_panel_ops.h"
#include <algorithm>
using Vec = Mathutils::Vector2;
using Vector3 = Mathutils::Vector3;
using Mat = Mathutils::Matrix2;

class Scene {
private:
    std::vector<Particle> particles;
    const int static n = 24;
    const int window_size = 360;
    const int displayScale = window_size/n;
    const static int addNum = 1000;
    const int BytePerPixel = 2;
    const real GravityScale = 1000;
    const real dt = 60e-4F / n;
    const real frame_dt = 360;
    const real dx = 1.0F / n;
    const real inv_dx = n;
    const real particle_mass = 1;
    const real vol= 5;
    const real hardening = 10;
    const real E = 1e4;
    const real nu = 0.2;
    const real mu_0 = E / (2 * (1 + nu));
    const real lambda_0 =  E * nu / ((1 + nu) * (1 - 2 * nu));;
    const real boundary = 0.05;
    uint8_t* fluid, *background;
    Vector3 grid[n + 1][n + 1];
    bool isFluid[n][n];
    bool isFluid_old[n][n];
public:

void render(esp_lcd_panel_handle_t panel_handle);
void update(real dt, Vector3 G);
void init(Vec center);
void add_object(Vec center, int num = addNum);
float getdt(){ return dt;};


};
#endif // SCENE_H
