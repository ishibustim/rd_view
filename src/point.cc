#include "point.h"
#include "vector.h"

point::point()
{
  x = 0;
  y = 0;
  z = 0;
}//end default constructor

point::point(double newX, double newY, double newZ)
{
  x = newX;
  y = newY;
  z = newZ;
}//end constructor

point point::multiply(const double scalar, const point& p)
{
  point temp(p.x * scalar, p.y * scalar, p.z * scalar);
  return temp;
}//end multiply(double, point)

point operator*(const double scalar, const point& p)
{
  return point::multiply(scalar, p);
}//end operator*(double, point)

point point::multiply(const point& p, const double scalar)
{
  return point::multiply(scalar, p);
}//end multiply(point, double)

point operator*(const point& p, const double scalar)
{
  return point::multiply(scalar, p);
}//end operator*(point, double)

void point::copy(point& dest, const point& src)
{
  dest.x = src.x;
  dest.y = src.y;
  dest.z = src.z;
}//end copy

vector point::subtract(const point& p1, const point& p2)
{
  vector temp(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
  return temp;
}//end subtract
