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
    int n, window_size, rowInChunk, ChunkNum, BytePerPixel, displayScale;
    real dt, frame_dt, dx, inv_dx, particle_mass, vol;
    real hardening, E, nu, mu_0, lambda_0;
    uint8_t* fluid, *background;
public:

void render(esp_lcd_panel_handle_t panel_handle);
void update(real dt, Vector3 G);
void init(int window_size_, real frame_dt_, real particle_mass_,
        real vol_, real hardening_, real E_, real nu_);
void add_object(Vec center, int num = 1000);
float getdt(){ return dt;};


};
#endif // SCENE_H
