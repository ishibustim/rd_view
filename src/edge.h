#ifndef EDGE_H
#define EDGE_H

#include "attr_point.h"

struct edge
{
  int yLast;      // Final scan line of edge
  attr_point p;   // The values of the edge on this scan line
  attr_point inc; // The incremental changes for the values from
                  // scanline to scanline

  edge *next;     // A pointer to link the edges together in edge tables

  edge();         // Default constructor
  edge(edge& e);  // Copy constructor
};//end struct

#endif
