#ifndef RD_PARSE_H
#define RD_PARSE_H

/****************************************************************************/
/* Parser for "rd" rendering description files.                             */
/*                                                                          */
/* See rd_error.h for return error codes                                    */
/*                                                                          */
/****************************************************************************/

/* The current input line number.  Useful for error messages. */
extern int rd_input_line;

/* routines avaliable in C and C++ (but with C bindings) */

#if defined (__cplusplus)
extern "C"
{
#endif 

  int rd_parse_init(char * filename);
  /* Opens the file and starts the parsing */

  int rd_parse_unit(void);
  /* Parses and processes the next piece of the file */

  int rd_parse_end(void);
  /* Cleans up the parser and closes the parse file. */

#if defined (__cplusplus)
}
#endif

#endif /* RD_PARSE_H */
