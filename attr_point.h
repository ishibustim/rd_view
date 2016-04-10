#ifndef ATTR_POINT_H
#define ATTR_POINT_H

#define ATTR_CONSTANT  4
#define ATTR_R         5
#define ATTR_G         6
#define ATTR_B         7
#define ATTR_NX        8
#define ATTR_NY        9
#define ATTR_NZ       10
#define ATTR_S        11
#define ATTR_T        12
#define ATTR_WORLD_X  13
#define ATTR_WORLD_Y  14
#define ATTR_WORLD_Z  15

#define ATTR_SIZE     16

typedef struct {
  float coord[ATTR_SIZE];
} attr_point;

#endif
