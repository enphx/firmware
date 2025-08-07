#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

struct Trajectory {
  float height;
  float radius;
  float theta;
  float deltaT;
};

inline const Trajectory scanOutisdeDoor[] = {
    {13, 15, -5, 400}, {13, 13, -5, 200}, {13, 11, -20, 200},  {13, 11, -45, 150},
    {9, 8, -60, 400},   {6.5, 8, -70, 150},   {5, 8, -80, 100},
    {4.5, 8, -80, 100}, {3.5, 7, -80, 100},

};

inline const Trajectory pickupPetOne[] = {
    {11, 9, -60, 300}, {14, 8, -45, 200}, {14, 6, -30, 100}, {14, 5, 0, 100}};

inline const Trajectory scorePetOne[] = {
    {14, 7, 30, 150}, {14, 9, 60, 150}, {14, 11, 90, 150}};

inline const Trajectory scanPetTwo[] = {
    {8, 4, 60, 100}, {6.5, 7, 60, 150},   {5.5, 7, 60, 100},
    {5, 6.5, 60, 100}, {4.8, 6.5, 60, 100},
};
inline const Trajectory storePetThree[] = {
  {13, 9, 45, 300},
  {14, 7, 25, 200},
  {14, 3, 5, 200},
};
inline const int scanOutisdeDoorLength = 9;
inline const int pickupPetOneLength = 4;
inline const int scorePetOneLength = 3;
inline const int scanPetTwoLength = 5;
inline const int storePetThreeLength = 3;

#endif
