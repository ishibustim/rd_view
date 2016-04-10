#include "rd_parse.h"
#include "rd_error.h"
#include "rd_engine.h"
#include "rd_display.h"

#include <iostream>

using std::cerr;
using std::endl;

void usage(char * argv[]);

int main(int argc, char * argv[])
{
  int err = 0;

  if(argc < 2){
    usage(argv);
    return 0;
  }

  // Initialize parser
  err = rd_parse_init(argv[1]);  
  if(err){
    rd_print_error(err, argv[1]);
    return err;
  }

  // Initialize rendering engine

  create_default_engine();
  err = render_engine->rd_render_init();

  if(err){
    rd_print_error(err, argv[1]);
    return err;
  }

  bool done = false;
  do{    
    err = rd_parse_unit();
    switch(err)
      {
      case RD_OK:  // Continue
	break;
      case RD_INPUT_EXPECTED_EOF:
	done = true;
	break;
      case RD_INPUT_UNINITIALIZED_DISPLAY:
	// This must be done out of parser for display types that
	// don't return on initialization
	if((err = rd_disp_init_display()))
	  done = true;
	break;
      default:
	done = true;
	break;
      }
  } while(!done);

  // Check for valid exit
  if(err == RD_INPUT_EXPECTED_EOF){ 
    err = RD_OK;
  }
  else
    rd_print_error(err, argv[1]);
  
  rd_parse_end();
  render_engine->rd_render_cleanup();
  rd_disp_end_display();
  release_engine();
  return err;
}

void usage(char * argv[])
{
  cerr << "Usage: " << argv[0] << "  rd_file" << endl;
}
