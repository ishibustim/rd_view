#ifndef POINT_H
#define POINT_H

// Forward declarations for function declarations
class vector;

class point
{
public:
  double x, y, z;

  point();
  point(double newX, double newY, double newZ);

  // Multiplication Functions
  static point multiply(const double scalar, const point& p);
  friend point operator*(const double scalar, const point& p);
  static point multiply(const point& p, const double scalar);
  friend point operator*(const point& p, const double scalar);

  // Copy Functions
  static void copy(point& dest, const point& src);

  // Misc Functions
  static vector subtract(const point& p1, const point& p2);
};//end class

#endif
