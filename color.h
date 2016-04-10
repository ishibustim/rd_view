#ifndef COLOR_H
#define COLOR_H

struct color
{
  float red, green, blue;

  color(float r = 0.0, float g = 0.0, float b = 0.0);

  void setColor(float r, float g, float b);
  void setColor(const color c);

  float* getRGB();

  friend bool operator==(const color& c1, const color& c2);
};//end class

#endif
