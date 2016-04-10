#include <cmath>
#include "point.h"
#include "pointh.h"
#include "vector.h"

vector::vector()
{
  x = 0;
  y = 0;
  z = 0;
}//end default constructor

vector::vector(double newX, double newY, double newZ)
{
  x = newX;
  y = newY;
  z = newZ;
}//end constructor

vector vector::multiply(const double scalar, const vector& v)
{
  vector temp(v.x * scalar, v.y * scalar, v.z * scalar);
  return temp;
}//end multiply(double, vector)

vector operator*(const double scalar, const vector& v)
{
  return vector::multiply(scalar, v);
}//end operator*(double, vector)

vector vector::multiply(const vector& v, const double scalar)
{
  return vector::multiply(scalar, v);
}//end multiply(vector, double)

vector operator*(const vector& v, const double scalar)
{
  return vector::multiply(scalar, v);
}//end operator*(vector, double)

void vector::copy(vector& dest, const vector& src)
{
  dest.x = src.x;
  dest.y = src.y;
  dest.z = src.z;
}//end copy

double vector::dot(const vector& v1, const vector& v2)
{
  return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
}//end dot

vector vector::cross(const vector& v1, const vector& v2)
{
  vector temp;
  temp.x = (v1.y * v2.z) - (v2.y * v1.z);
  temp.y = ((v1.x * v2.z) - (v2.x * v1.z)) * -1;
  temp.z = (v1.x * v2.y) - (v2.x * v1.y);
  return temp;
}//end cross

double vector::mag2(const vector& v)
{
  return vector::dot(v, v);
}//end mag2

void vector::normalize(vector& v)
{
  double mag = sqrt(mag2(v));
  v.x /= mag;
  v.y /= mag;
  v.z /= mag;
}//end in-place normalize

vector vector::add(const vector& v1, const vector& v2)
{
  vector temp(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
  return temp;
}//end add

vector vector::subtract(const vector& v1, const vector& v2)
{
  vector temp(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
  return temp;
}//end vector subtract
