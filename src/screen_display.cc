#include "rd_display.h"
#include "rd_error.h"
#include "rd_parse.h"
#include "screen_display.h"
#ifdef WIN32
#else
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>  // Standard Colormaps
#endif

// Much of the structure of this code is based on a GLUT implementation,
// with all of the weirdness that implies.

#include <iostream>
using std::cerr;
using std::endl;

#include <cstdlib>

#ifdef WIN32
#include <windows.h> // Sleep()
#else
#include <unistd.h>  // sleep()
#endif

/*********************  Local prototypes ***************************/

static int screen_init_display(void);
static void event_loop(void);

#ifndef WIN32
static void get_color_conversion(XVisualInfo * v);
static unsigned long rgb_to_xpixel(const float color[3]);
#endif

/*********************  Local variables ****************************/

#ifndef WIN32
Display * display;
int s;  // screen
Window window;
Pixmap pixmap;
GC gc;
Colormap colormap;

float  XColorScale[3] = {0.0, 0.0, 0.0};
int XColorShift[3] = {0, 0, 0};

#endif

static int background_color[3] = {0, 0, 0};
static unsigned char * image = 0;  // Copy of screen for simplicity
static bool pixel_update = false;
static bool input_done = false;

#define COLOR_SCALE_MAX   255.999

int screen_init_display_single(void)
{
  int err = RD_OK;

  pixel_update = true;

  err = screen_init_display();

  return err;
}

int screen_init_display_double(void)
{
  int err = RD_OK;

  pixel_update = false;

  err = screen_init_display();

  return err;
}

int screen_init_display(void)
{
#ifndef WIN32

  display = XOpenDisplay(0);
  if(!display)
    return RD_INPUT_UNAVAILABLE_DISPLAY;
    
  s = DefaultScreen(display);

  // Check visual type
  XWindowAttributes  xattr;
  XGetWindowAttributes(display, RootWindow(display, s), &xattr);
  VisualID vid = XVisualIDFromVisual(xattr.visual);
  XVisualInfo vinfo, *vinforesult;
  int nvisuals;
  vinfo.visualid = vid;
  vinforesult = XGetVisualInfo(display, VisualIDMask, &vinfo, &nvisuals);
  
  if(vinforesult[0].c_class != TrueColor){
    // Not worth the effort at this time
    XFree(vinforesult);
    return RD_INPUT_DISPLAY_INITIALIZATION_ERROR;
  }
  
  get_color_conversion(vinforesult);
  
  XFree(vinforesult);

  // Create window
  window = XCreateSimpleWindow(display, RootWindow(display, s), 0, 0,
			       display_xSize, display_ySize, 0, 
			       0, BlackPixel(display, s));

  XSelectInput(display, window, ExposureMask | ButtonPressMask );

  XStoreName(display, window, display_name);

  XMapRaised(display, window);

  // Create backup pixmap to draw into
  pixmap = XCreatePixmap(display, RootWindow(display, s), 
			 display_xSize, display_ySize, 
			 DefaultDepth(display, s));


  // Get graphics context for drawing
  XGCValues gcv;
 
  gcv.graphics_exposures = True;
  
  //  gc = XCreateGC(display, window, GCGraphicsExposures, &gcv);
  gc = XCreateGC(display, window, 0, &gcv);

#endif

  // Create fast pixel read buffer
  image = new unsigned char [display_xSize * display_ySize * 3];

  if(!image)
    return RD_INPUT_INSUFFICIENT_DISPLAY_MEMORY;

  // Window is not guaranteed to be realized until first Expose event 
  // received, so...

  XEvent event;

  while(XPeekEvent(display, &event))
    {
      if(event.type == Expose)
	break;
      
      XNextEvent(display, &event);
    }

  // Call event processing loop - never returns until done
  event_loop();

  return RD_OK;
}

int screen_end_display(void)
{
  delete [] image;
  
#ifndef WIN32
  XFreeGC(display, gc);
  XFreePixmap(display, pixmap);
  XDestroyWindow(display, window);
  XCloseDisplay(display);
#endif
  
  return RD_OK;
}

int screen_init_frame(int frame_no)
{
  static bool first_time = true;
  
  if(first_time){
    // Place first time frame initialization here 
    // Sometimes a deliberate delay is placed here to wait a bit
    // for display initialization to take hold
    // sleep(1);
    
    first_time = false;
  }
  
  screen_clear();
  return RD_OK;
}

int screen_end_frame_single()
{
  return RD_OK;
}

int screen_end_frame_double()
{
  return RD_OK;
}

int screen_write_pixel(int x, int y, const float color [])
{
  if(x < 0 || y < 0 || x >= display_xSize || y >= display_ySize)
    return RD_OK; // Clipped
     
  unsigned char * p = image + (display_xSize * y + x) * 3;
  p[0] = (unsigned char)(color[0] * COLOR_SCALE_MAX);
  p[1] = (unsigned char)(color[1] * COLOR_SCALE_MAX);
  p[2] = (unsigned char)(color[2] * COLOR_SCALE_MAX);

#ifndef WIN32
  XSetForeground(display, gc, rgb_to_xpixel(color));
  XDrawPoint(display, pixmap, gc, x, y);

#endif     
  if(pixel_update)
     {
#ifndef WIN32
       XDrawPoint(display, window, gc, x, y);
       XSync(display, False);
#endif
     }

  return RD_OK;
}

int screen_read_pixel(int x, int y, float color [])
{

  unsigned char * p = image + (display_xSize * (y) + x) * 3;
  color[0] = p[0] / COLOR_SCALE_MAX;
  color[1] = p[1] / COLOR_SCALE_MAX;
  color[2] = p[2] / COLOR_SCALE_MAX;

  return RD_OK;
}

int screen_set_background(const float color [])
{
  background_color[0] = (unsigned char)(color[0] * COLOR_SCALE_MAX);
  background_color[1] = (unsigned char)(color[1] * COLOR_SCALE_MAX);
  background_color[2] = (unsigned char)(color[2] * COLOR_SCALE_MAX);

  return RD_OK;
}

int screen_clear(void)
{
  unsigned char * p = image;
  for(int y = 0; y < display_ySize; y++)
    for(int x = 0; x < display_xSize; x++, p+=3){
      p[0] = background_color[0];
      p[1] = background_color[1];
      p[2] = background_color[2];
    }

#ifndef WIN32
  float bc[3];
  bc[0] = background_color[0] / COLOR_SCALE_MAX;
  bc[1] = background_color[1] / COLOR_SCALE_MAX;
  bc[2] = background_color[2] / COLOR_SCALE_MAX;

  XSetForeground(display, gc, rgb_to_xpixel(bc));
  XFillRectangle(display, pixmap, gc, 0, 0, display_xSize, display_ySize);
  if(pixel_update)  // single buffer mode
    XFillRectangle(display, window, gc, 0, 0, display_xSize, display_ySize);
  
#endif

  return RD_OK;
}

void event_loop(void)
{
#ifndef WIN32
  // Process necessary events while they exist. then do "idle" processing

  while(1)
    {
      XEvent event;

      while(XEventsQueued(display, QueuedAfterFlush))
	{
	  XNextEvent(display, &event);
	  
	  switch(event.type)
	    {
	    case Expose:
	      XCopyArea(display, pixmap, window, gc,
			event.xexpose.x, event.xexpose.y,
			event.xexpose.width, event.xexpose.height,
			event.xexpose.x, event.xexpose.y);
	      /*
	      cerr << "Expose: (" << event.xexpose.x << ", "
		   << event.xexpose.y << "): ("
		   << event.xexpose.width << ", "
		   << event.xexpose.height << ")" << endl;
	      */
	      break;
	    case ButtonPress:
	      if(event.xbutton.button != Button1)
		{
		  // Right button -- quit
		  return;
		}
	      else
		{
		  // Left button -- print pixel value
		  cerr << "(" << event.xbutton.x << ", " << event.xbutton.y << "): ";
		  unsigned char * p = image + (display_xSize * event.xbutton.y + event.xbutton.x) * 3;
		  cerr << p[0] / COLOR_SCALE_MAX << " ";
		  cerr << p[1] / COLOR_SCALE_MAX << " ";
		  cerr << p[2] / COLOR_SCALE_MAX << endl;
		}
	      break;
	    default:
	      break;
	    }
	}
      
      // Process data
      int err;
      if(!input_done)
	{
	  if((err = rd_parse_unit())){
	    if(err == RD_INPUT_EXPECTED_EOF){
	      input_done = true;
	    }
	    else{
	      rd_print_error(err, "input file");
	    }
	  }
	  // Show the work done
	  event.type = Expose;
	  //  event.xexpose.serial = ;
	  event.xexpose.send_event = true;
	  event.xexpose.display = display;
	  event.xexpose.window = window;
	  event.xexpose.x = 0;
	  event.xexpose.y = 0;
	  event.xexpose.width = display_xSize;
	  event.xexpose.height = display_ySize;
	  event.xexpose.count = 0;
	  
	  XSendEvent(display, window, False, ExposureMask, &event);
	}
      else
	{
	  // Just wait for the next event
	  XPeekEvent(display, &event);
	}
    } /* end while(1) */
  
#endif

}

static void get_color_conversion(XVisualInfo * v)
{
  unsigned long value = v->red_mask;
  // Red
  XColorShift[0] = 0;
  while(!(value & 0x01))
    {
      value >>= 1;
      XColorShift[0]++;
    }
  XColorScale[0] = value + 0.999;

  // Green
  value = v->green_mask;
  XColorShift[1] = 0;
  while(!(value & 0x01))
    {
      value >>= 1;
      XColorShift[1]++;
    }
  XColorScale[1] = value + 0.999;

  // Blue
  value = v->blue_mask;
  XColorShift[2] = 0;
  while(!(value & 0x01))
    {
      value >>= 1;
      XColorShift[2]++;
    }
  XColorScale[2] = value + 0.999;
}

static unsigned long rgb_to_xpixel(const float color[3])
{
  unsigned long value = 0;
  
  value = 
    (unsigned long)(color[0] * XColorScale[0]) << XColorShift[0]
    | (unsigned long)(color[1] * XColorScale[1]) << XColorShift[1]
    | (unsigned long)(color[2] * XColorScale[2]) << XColorShift[2];

  return value;
}
