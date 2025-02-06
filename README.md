# FluidSimPendant

An ESP32-S3 Programme for Fluid Simulation and Render, which works on a pendant.


Fluid Simulation Loop Duration ~= 7ms; Depends on # of particles. It's based on MLS-MPM.

Fluid Render Loop Duration ~= 60ms; Depends on # of grids, # of Chunks and sleep time between render Loop.

TODO: Make variables const and add a button for pause and researt. Find a way to speed up; Make wall no sticky and visable; change color; make bubble less.