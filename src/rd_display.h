#ifndef RD_DISPLAY_H
#define RD_DISPLAY_H

#include <string>
using std::string;

/* Global values set automatically */

extern int display_xSize, display_ySize;

extern const char * display_name;

/************************************************************************/
/* Generic rendering functions.  When rd_set_display() is called, these
   functions will be connected with the appropriate drivers. */
/************************************************************************/

extern int (* rd_disp_init_display)(void);
/* To initialize the display.  When this function is called, the global
   values above will be known. This function and rd_disp_end_display()
   are usually called automatically by the parser.  The other functions 
   below are not. */

extern int (* rd_disp_end_display)(void);

extern int (* rd_disp_init_frame)(int frame_no);
/* Sets up the display for a new frame.  Complete reinitialization not
   needed. */

extern int (* rd_disp_end_frame)(void);
/* This is the place to force the display or storage of the final pixel
   values */

extern int (* rd_write_pixel)(int x, int y, const float rgb []);
/* Writes the color given in the rgb array to location (x, y). Color 
   component values are in the range 0.0-1.0 with red in position 0; 
   green in 1; and blue in 2. x and y are in the range from 0 up to 
   but not including display_xSize and display_ySize respectively.*/

extern int (* rd_read_pixel)(int x, int y, float rgb []);
/* Reads the color at location (x, y) into r, g, and b.   Color component 
   values are in the range 0.0-1.0. Red, green, and blue are read into
   position 0, 1, and 2 respectively */

extern int (* rd_set_background)(const float rgb []);
/* Save these values in a global variable(s).  Used by rd_clear() to
   clear the background. */

extern int (* rd_clear)(void);
/* Clear the display to the background color */

/***********************************************************************/
/* These function do not need to be implemented by drivers.  They are
   called by the parsing routines. */

int rd_set_display(const string & name, const string & type, 
		   const string & mode);

int rd_set_format(int xresolution, int yresolution);


#endif /* RD_DISPLAY_H */
