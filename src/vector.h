#ifndef VECTOR_H
#define VECTOR_H

// Forward declarations for function declarations
class point;
class pointh;

class vector
{
public:
  double x, y, z;

  vector();
  vector(double newX, double newY, double newZ);

  // Scalar Multiplication functions
  static vector multiply(const double scalar, const vector& v);
  friend vector operator*(const double scalar, const vector& v);
  static vector multiply(const vector& v, const double scalar);
  friend vector operator*(const vector& v, const double scalar);

  // Copy Functions
  static void copy(vector& dest, const vector& src);

  // Vector Multiplication Functions
  static double dot(const vector& v1, const vector& v2);
  static vector cross(const vector& v1, const vector& v2);

  // Misc Functions
  static double mag2(const vector& v);
  static void normalize(vector& v);
  static vector add(const vector& v1, const vector& v2);
  static vector subtract(const vector& v1, const vector& v2);
};//end class

#endif
