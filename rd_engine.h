#ifndef RD_ENGINE_H
#define RD_ENGINE_H

#include "rd_enginebase.h"

#include <string>
using std::string;

extern RenderEngine * render_engine;

void create_default_engine();
// Initializes the list of rendering engines
// Called once in program initialization

int find_engine(const string & name);
// Finds/creates the engine associated with the name and attaches
// it to the render_engine given above

void release_engine();
// Final cleanup and removal of the active rendering engine
// Performed once at program exit.

#endif /* RD_ENGINE_H */
