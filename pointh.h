#ifndef POINTH_H
#define POINTH_H

// Forward declarations for function declarations
class point;
class vector;

class pointh
{
public:
  double x, y, z, w;

  pointh();
  pointh(double newX, double newY, double newZ, double newW);

  // Conversion Functions
  static pointh convert(const point& p);
  static pointh convert(const vector& v);
};//end class

#endif
