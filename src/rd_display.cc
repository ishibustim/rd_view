#include "rd_error.h"
#include "rd_display.h"
#include "rd_refresh.h"
#include "screen_display.h"
#include "pnm_display.h"

#include <iostream>  // Debug
#include <string>

using std::string;

// *********************  Dummy function stubs *****************************
// These are here to prevent segmentation faults when display is not 
// initialized and functions are called

static int rd_disp_dummy_func(void);
static int rd_disp_dummy_func1_err(void);
static int rd_disp_dummy_func2_err(int);

// *********************  Function pointers ********************************
int (* rd_disp_init_display)(void) = rd_disp_dummy_func;

int (* rd_disp_end_display)(void) = rd_disp_dummy_func;

int (* rd_disp_init_frame)(int frame_no) = rd_disp_dummy_func2_err;

int (* rd_disp_end_frame)(void) = rd_disp_dummy_func1_err;

int (* rd_write_pixel)(int x, int y, const float rgb [] );

int (* rd_read_pixel)(int x, int y, float rgb []);

int (* rd_set_background)(const float rgb []);

int (* rd_clear)(void);

int display_xSize = 640, display_ySize = 480;

static string display_name_buffer;

const char * display_name;

int rd_set_display(const string & name, const string & type, 
		   const string & mode)
{

  display_name_buffer = name;
  display_name = display_name_buffer.c_str();

  display_refresh[refresh_pixel] = false;
  display_refresh[refresh_object] = false;
  display_refresh[refresh_frame] = false;
  
  if(type == "Screen")
    {
      if(mode == "rgbsingle"){
	// Single buffer simulation -- update after every pixel
	rd_disp_init_display = screen_init_display_single;
	rd_disp_end_frame    = screen_end_frame_single;
	display_refresh[refresh_pixel] = true;
	display_refresh[refresh_object] = true; 
	     // Good for premature termination when available
	display_refresh[refresh_frame] = true;
      }
      else if(mode == "rgbdouble"){
	// True double buffer mode - refresh after frame finishes
	rd_disp_init_display = screen_init_display_double;
	rd_disp_end_frame    = screen_end_frame_double;
	display_refresh[refresh_frame] = true;
      }
      else if(mode == "rgbobject"){
	// Really a double buffer, but with breaks (hence refreshes)
	// after each primitive
	rd_disp_init_display = screen_init_display_double;
	rd_disp_end_frame    = screen_end_frame_double;
	display_refresh[refresh_object] = true;
	display_refresh[refresh_frame] = true;
      }
      else
	return RD_INPUT_UNKNOWN_DISPLAY_MODE;

      rd_disp_end_display    = screen_end_display;
      rd_disp_init_frame     = screen_init_frame;
      rd_write_pixel         = screen_write_pixel;
      rd_read_pixel          = screen_read_pixel;
      rd_set_background      = screen_set_background;
      rd_clear               = screen_clear;
    }
  else if(type == "PNM")
    {
      // Hardwire mode for now
      if(mode == "rgb"){
	;
      }
      else
	return RD_INPUT_UNKNOWN_DISPLAY_MODE;

      rd_disp_init_display = pnm_init_display;
      rd_disp_end_frame    = pnm_end_frame;
      rd_disp_end_display  = pnm_end_display;
      rd_disp_init_frame   = pnm_init_frame;
      rd_write_pixel       = pnm_write_pixel;
      rd_read_pixel        = pnm_read_pixel;
      rd_set_background    = pnm_set_background;
      rd_clear             = pnm_clear;
    }
  else
    return RD_INPUT_UNKNOWN_DISPLAY_TYPE;

  return RD_OK;
}

int rd_set_format(int xresolution, int yresolution)
{
  display_xSize = xresolution;
  display_ySize = yresolution;

  return RD_OK;
}

int rd_disp_dummy_func(void)
{
  return RD_OK;
}

int rd_disp_dummy_func1_err(void)
{
  return RD_INPUT_UNKNOWN_DISPLAY_TYPE;
}

int rd_disp_dummy_func2_err(int)
{
  return RD_INPUT_UNKNOWN_DISPLAY_TYPE;
}
