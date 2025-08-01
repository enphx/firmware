#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

struct Trajectory {
  float height;
  float radius;
  float theta;
  float deltaT;
};

inline const Trajectory scanOutisdeDoor[] = {
    {6.5, 11, -5, 100}, {7.5, 11, -20, 200},  {8, 11, -45, 150},
    {8, 8, -60, 400},   {5.5, 7, -60, 150},   {5, 6.5, -60, 100},
    {4.5, 6, -60, 100}, {3.5, 5.5, -60, 100},

};

inline const int scanOutisdeDoorLength = 8;

#endif
