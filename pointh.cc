#include "point.h"
#include "pointh.h"
#include "vector.h"

pointh::pointh()
{
  x = 0;
  y = 0;
  z = 0;
  w = 1;
}//end default constructor

pointh::pointh(double newX, double newY, double newZ, double newW)
{
  x = newX;
  y = newY;
  z = newZ;
  w = newW;
}//end constructor

pointh pointh::convert(const point& p)
{
  pointh temp(p.x, p.y, p.z, 1);
  return temp;
}//end convert(point)

pointh pointh::convert(const vector& v)
{
  pointh temp(v.x, v.y, v.z, 1);
  return temp;
}//end convert(vector)
