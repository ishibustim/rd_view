#ifndef RD_ENGINE_DIRECT_H
#define RD_ENGINE_DIRECT_H

#include "rd_enginebase.h"

// This is a rendering engine that renders directly to the image buffer as
// primitives come in.  A depth buffer is obviously needed.  Transparency is
// not properly handled.

class REDirect: public RenderEngine
{
 public:
  int rd_display(const string& name, const string& type, const string& mode);
  int rd_format(int xresolution, int yresolution);

  int rd_world_begin(void);
  int rd_world_end(void);

  int rd_frame_begin(int frame_no);
  int rd_frame_end(void);

  int rd_background(const float color[]);
  int rd_color(const float color[]);

  int rd_circle(const float center[3], float radius);
  int rd_line(const float start[3], const float end[3]);
  int rd_point(const float p[3]);

  int rd_fill(const float seed_point[3]);
};

#endif /* RD_ENGINE_DIRECT_H */
