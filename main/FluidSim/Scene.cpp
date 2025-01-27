#include "Scene.h"

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
    srand(static_cast<unsigned int>(time(nullptr)));
}

extern "C" void Scene::update(real dt, Vector3 G)
{
    std::memset(grid, 0, sizeof(grid)); // Reset grid
    for (auto& p : particles) { // P2G
        Vec temp(p.x * inv_dx - Vec(0.5F)); // element-wise floor
        Vec base_coord(int(temp.x), int(temp.y));
        Vec fx = p.x * inv_dx - base_coord;
        // Quadratic kernels  [http://mpm.graphics   Eqn. 123, with x=fx, fx-1,fx-2]
        Vec w[3] { Vec(0.5) * (Vec(1.5) - fx) * (Vec(1.5) - fx), Vec(0.75) - (fx - Vec(1.0)) * (fx - Vec(1.0)),
            Vec(0.5) * (fx - Vec(0.5)) * (fx - Vec(0.5)) }; // sqr(Vec(1.5)) = Vec(1.5) * Vec(1.5)
        auto e = std::exp(hardening * (1.0F - p.Jp));
        // real J = p.F.det(2); //                         Current volume Det????
        Mat r(2, 2), s(2, 2);
        polar_decomp(p.F, r, s); // Polar decomp. for fixed corotated model
        Mat cauchy = dspm::Mat::eye(2) * (0.2F * E * (dspm::Mat::eye(2) * (p.Jp) - 1)); // diagnoal
        auto stress = // Cauchy stress times dt and inv_dx
            -4 * inv_dx * inv_dx * dt * vol * cauchy;
        auto affine = stress + particle_mass * p.C;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) { // Scatter to grid
                auto dpos = (Vec(i, j) - fx) * dx;
                Vector3 mv((p.v * particle_mass).x, (p.v * particle_mass).y,
                    particle_mass); // translational momentum
                Vec tmp = Mat2_times_Vec(affine, dpos);
                grid[int(base_coord.x) + i][int(base_coord.y) + j] += (mv + Vector3(tmp.x, tmp.y, 0) * w[i].x * w[j].y);
            }
        }
    }
    for (int i = 0; i <= n; i++) {
        for (int j = 0; j <= n; j++) { // For all grid nodes
            auto& g = grid[i][j];
            if (g.z > 0) { // No need for epsilon here
                g /= g.z; //        Normalize by mass
                g += G * dt; //                  Gravity
                real boundary = 0.05;
                real x = real(i) / n;
                real y = real(j) / n; // boundary thick.,node coord
                if (x < boundary || x > 1 - boundary || y > 1 - boundary)
                    g = Vector3(0); // Sticky
                if (y < boundary)
                    g.y = std::max(0.0F, g.y); //"Separate"
            }
        }
    }
    for (auto& p : particles) { // Grid to particle
        Vec temp(p.x * inv_dx - Vec(0.5F)); // element-wise floor
        Vec base_coord(int(temp.x), int(temp.y));
        Vec fx = p.x * inv_dx - base_coord;
        Vec w[3] { Vec(0.5) * (Vec(1.5) - fx) * (Vec(1.5) - fx), Vec(0.75) - (fx - Vec(1.0)) * (fx - Vec(1.0)),
            Vec(0.5) * (fx - Vec(0.5)) * (fx - Vec(0.5)) }; // sqr(Vec(1.5)) = Vec(1.5) * Vec(1.5)
        p.C = Mat(2, 2); // zero
        p.v = Vec(0);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                auto dpos = (Vec(i, j) - fx);
                auto grid_v = Vec(grid[int(base_coord.x) + i][int(base_coord.y) + j].x, grid[int(base_coord.x) + i][int(base_coord.y) + j].y);
                auto weight = w[i].x * w[j].y;
                p.v += grid_v * weight; // Velocity
                p.C += 4 * inv_dx * outer_product(grid_v * weight, dpos); // APIC C
            }
        }
        p.x += p.v * dt; // Advection
        p.Jp *= (dspm::Mat::eye(2) + dt * p.C).det(2); // det??
    }
}

extern "C" void Scene::add_object(Vec center, int num)
{
    float radius = 0.05F; // Define the radius around the center
    for (int i = 0; i < num; ++i) {
        Vec offset = generate_random_float(-radius, radius);
        particles.emplace_back(Particle(center + offset, Vec(0.0f, 0.0f)));
    }
}
