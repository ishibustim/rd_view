#include "rd_engine.h"
#include "rd_error.h"

// Add new rendering engines to this list
#include "rd_direct.h"

RenderEngine * render_engine = 0;

void create_default_engine()
{
  render_engine = new REDirect;  // The default rendering engine
}

int find_engine(const string & name)
{
  // Cleanup old engine

  delete render_engine;

  // Find new engine
  if(name == "direct")
    render_engine = new REDirect;
  else
    {
      // Can't be found
      render_engine = new REDirect; // Default
      return RD_INPUT_UNKNOWN_ENGINE;
    }

  if(render_engine == 0) // Couldn't allocate
    render_engine = new REDirect;  // Default

  return RD_OK;
}

void release_engine()
{
  delete render_engine;
}

