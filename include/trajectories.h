#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

struct Trajectory {
  float asimuthTheta;
  float shoulderTheta;
  float elbowTheta;
  float deltaT;
};

inline const Trajectory scanOutisdeDoor[] = {
    {-2.0f, 43.9f, 93.0f, 100.0f},   {-20.0f, 49.1f, 93.0f, 200.0f},
    {-45.0f, 51.5f, 92.7f, 150.0f},  {-60.0f, 52.6f, 119.5f, 400.0f},
    {-70.0f, 51.3f, 126.8f, 150.0f}, {-80.0f, 47.7f, 129.7f, 100.0f},
    {-80.0f, 43.4f, 132.1f, 100.0f}, {-80.0f, 33.5f, 131.9f, 100.0f},
};

inline const int scanOutisdeDoorLength = 8;

#endif
