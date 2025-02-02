#include "Scene.h"
#include "esp_attr.h"
const int n_ = 90;
Vector3 grid[n_ + 1][n_ + 1]; // velocity + mass, node_res = cell_res + 1

extern "C" void Scene::init(int window_size_, real frame_dt_, real particle_mass_,
    real vol_, real hardening_, real E_, real nu_)
{
    /* Config paraments*/
    n = n_;
    window_size = window_size_;
    dt = 60e-4F / n;
    frame_dt = frame_dt_;
    dx = 1.0F / n;
    inv_dx = 1.0F / dx;
    particle_mass = particle_mass_;
    vol = vol_;
    hardening = hardening_;
    E = E_;
    nu = nu_;
    mu_0 = E / (2 * (1 + nu));
    lambda_0 = E * nu / ((1 + nu) * (1 - 2 * nu));
    renderBuffer = (uint8_t *)heap_caps_calloc(1,  window_size *2 ,MALLOC_CAP_DMA);
    srand(static_cast<unsigned int>(time(nullptr)));
}

extern "C" void Scene::update(real dt, Vector3 G)
{
    std::memset(grid, 0, sizeof(grid)); // Reset grid
    for (auto& p : particles) { // P2G
        Vec base_coord = Vec(floor(p.x.x * inv_dx - 0.5f), floor(p.x.y * inv_dx - 0.5f));
        Vec fx = p.x * inv_dx - base_coord;
        // Quadratic kernels  [http://mpm.graphics   Eqn. 123, with x=fx, fx-1,fx-2]
        Vec w[3] { Vec(0.5) * sqr(Vec(1.5) - fx), Vec(0.75) - sqr(fx - Vec(1.0)),
            Vec(0.5) * sqr(fx - Vec(0.5)) };
        Mat r, s;
        Mathutils::polar_decomp(p.F, r, s); // Polar decomp. for fixed corotated model
        Mat cauchy = Mat(0.2F * E * (p.Jp - 1));
        auto stress = cauchy * -4 * inv_dx * inv_dx * dt * vol;
        auto affine = stress + p.C * particle_mass;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) { // Scatter to grid
                auto dpos = (Vec(i, j) - fx) * dx;
                Vector3 mv((p.v * particle_mass).x, (p.v * particle_mass).y,
                    particle_mass); // translational momentum
                grid[int(base_coord.x + i)][int(base_coord.y + j)] += (mv + Vector3((affine * dpos).x, (affine * dpos).y, 0)) * w[i].x * w[j].y;
            }
    }
    for (int i = 0; i <= n; i++)
        for (int j = 0; j <= n; j++) { // For all grid nodes
            auto& g = grid[i][j];
            if (g.z > 0) { // No need for epsilon here
                g /= g.z; //        Normalize by mass
                g += G * dt; //                  Gravity
                real boundary = 0.05;
                real x = (real)i / n;
                real y = real(j) / n; // boundary thick.,node coord
                if (x < boundary || x > 1 - boundary || y > 1 - boundary)
                    g = Vector3(0); // Sticky
                if (y < boundary)
                    g.y = std::max(0.0F, g.x); //"Separate"
            }
        }
    for (auto& p : particles) { // Grid to particle
        Vec base_coord = Vec(floor(p.x.x * inv_dx - 0.5f), floor(p.x.y * inv_dx - 0.5f));
        Vec fx = p.x * inv_dx - base_coord;
        Vec w[3] { Vec(0.5) * sqr(Vec(1.5) - fx), Vec(0.75) - sqr(fx - Vec(1.0)),
            Vec(0.5) * sqr(fx - Vec(0.5)) };
        p.C = Mat(0);
        p.v = Vec(0);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                auto dpos = (Vec(i, j) - fx);
                auto grid_ij = grid[int(base_coord.x + i)][int(base_coord.y + j)];
                auto grid_v = Vec(grid_ij.x, grid_ij.y);
                auto weight = w[i].x * w[j].y;
                p.v += grid_v * weight; // Velocity
                p.C += Mat::outer_product(grid_v * weight, dpos) * 4 * inv_dx; // APIC C
            }
        p.x += p.v * dt; // Advection
        p.Jp *= (Mat(1) + p.C * dt).determinant();
    }
}

extern "C" void Scene::add_object(Vec center, int num)
{
    for (int i = 0; i < num; i++) {
        particles.emplace_back(center + Vec::RAND()* 0.20F);
    }
}

extern "C" void Scene::render(esp_lcd_panel_handle_t panel_handle)
{
    // TODO: Finish Render, axis direction
    for (size_t i = 0; i < window_size; i++)
    {
        for (size_t j = 0; i < n - 1; i++)
        {
            if (grid[i/4][j].x > 1)
            {
                memset(renderBuffer + j * 8, 0xFF, 8);
            }
            else
            {
                memset(renderBuffer + j * 8, 0x00, 8);
            }           
        }      
        //memset(renderBuffer, 0xFF/window_size*i, window_size * 2);
        esp_lcd_panel_draw_bitmap(panel_handle, 0, i, 360, i + 1, renderBuffer);       
    }
}