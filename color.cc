#include "color.h"

color::color(float r, float g, float b)
{
  setColor(r, g, b);
}//end constructor

void color::setColor(float r, float g, float b)
{
  if(r < 0)
    red = 0;
  else if(r > 1)
    red = 1;
  else
    red = r;

  if(g < 0)
    green = 0;
  else if(g > 1)
    green = 1;
  else
    green = g;

  if(b < 0)
    blue = 0;
  else if(b > 1)
    blue = 1;
  else
    blue = b;
}//end setColor

void color::setColor(const color c)
{
  setColor(c.red, c.green, c.blue);
}//end setColor

float* color::getRGB()
{
  float* rgb = new float;
  rgb[0] = red;
  rgb[1] = green;
  rgb[2] = blue;
  return rgb;
}//end getRGB

bool operator==(const color& c1, const color& c2)
{
  return (c1.red == c2.red && c1.green == c2.green && c1.blue == c2.blue);
}//end operator==
