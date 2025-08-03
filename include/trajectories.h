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

inline const Trajectory pickupPetOne[] = {
    {6, 7, -60, 300}, {8, 7, -45, 200}, {8, 6, -30, 100}, {8, 5, 0, 100}};

inline const Trajectory scorePetOne[] = {
    {8, 7, 30, 150}, {8, 9, 60, 150}, {8, 11, 90, 150}};

inline const Trajectory scanPetTwo[] = {
    {6.3, 4, 60, 100}, {6, 4.5, 60, 100},   {5.5, 5, 60, 100},
    {4, 5.5, 60, 100}, {3.5, 5.5, 60, 100},
};
inline const int scanOutisdeDoorLength = 8;
inline const int pickupPetOneLength = 4;
inline const int scorePetOneLength = 3;
inline const int scanPetTwoLength = 5;

#endif
