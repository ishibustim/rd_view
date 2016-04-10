#include "attr_point.h"
#include "edge.h"

edge::edge()
{
  yLast = 0;

  for(int i = 0; i < ATTR_SIZE; i++)
  {
    p.coord[i] = 0;
    inc.coord[i] = 0;
  }//end for

  next = 0;
}//end default constructor

edge::edge(edge& e)
{
  yLast = e.yLast;

  for(int i = 0; i < ATTR_SIZE; i++)
  {
    p.coord[i] = e.p.coord[i];
    inc.coord[i] = e.inc.coord[i];
  }//end for

  next = 0;
}//end copy constructor
