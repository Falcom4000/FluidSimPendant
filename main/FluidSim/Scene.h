#ifndef SCENE_H
#define SCENE_H 
#include "Mathutils.h"
#include "Particle.h"
#include "Vector3.h"
#include "cstring"
#include "mat.h"
#include <vector>
using dspm::Mat;
using real = float;

class Scene {
private:
    std::vector<Particle> particles;
    int n, window_size;
    real dt, frame_dt, dx, inv_dx, particle_mass, vol;
    real hardening, E, nu, mu_0, lambda_0;

public:

void render();
void update(real dt, Vector3 G);
void init(int window_size_, real frame_dt_, real particle_mass_,
        real vol_, real hardening_, real E_, real nu_);
void add_object(Vec center, int num = 30);
float getdt(){ return dt;};


};
#endif // SCENE_H
