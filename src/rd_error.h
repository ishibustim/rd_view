#ifndef RD_ERROR_H
#define RD_ERROR_H


/************** Some error codes ************************************/

#define RD_OK                                     0
#define RD_INPUT_EXPECTED_EOF                    -1
/* File ended, but in a good place */

#define RD_INPUT_UNINITIALIZED_DISPLAY           -2
#define RD_INPUT_UNAVAILABLE_DISPLAY             -3
#define RD_INPUT_INCOMPATIBLE_DISPLAY            -4
#define RD_INPUT_DISPLAY_INITIALIZATION_ERROR    -5
/* Can't get a full color display */

#define RD_INPUT_UNEXPECTED_EOF                  -6
#define RD_INPUT_FILE_NOT_FOUND                  -7
#define RD_INPUT_UNKNOWN_TOKEN                   -8
/*  Completely unrecognized token */

#define RD_INPUT_ILLEGAL_TOKEN                   -9
/*  Recognized token, but in the wrong place */

#define RD_INPUT_RUNAWAY_STRING                 -10
#define RD_INPUT_UNKNOWN_AXIS                   -11
#define RD_INPUT_OBJ_LIST_OVERFLOW              -12
#define RD_INPUT_CIRCULAR_INSTANCE              -13
#define RD_INPUT_UNKNOWN_VERTEX_TYPE            -14
#define RD_INPUT_ILLEGAL_VERTEX_INDEX           -15
#define RD_INPUT_UNKNOWN_OBJECT_INSTANCE        -16
#define RD_INPUT_DUPLICATE_OBJ_LABEL            -17
#define RD_INPUT_ILLEGAL_FLAG_VALUE             -18
#define RD_INPUT_INSUFFICIENT_DISPLAY_MEMORY    -19
#define RD_INPUT_INSUFFICIENT_ZBUFFER_MEMORY    -20
#define RD_INPUT_INSUFFICIENT_TEXTURE_MEMORY    -21
#define RD_INPUT_INTERLEAVED_BLOCK              -22
/* The end of this block is not the type expected */

#define RD_INPUT_UNKNOWN_DISPLAY_TYPE           -23
#define RD_INPUT_UNKNOWN_DISPLAY_MODE           -24

#define RD_INPUT_UNKNOWN_SURFACE_TYPE           -25
#define RD_INPUT_LIGHT_LIST_OVERFLOW            -26

#define RD_INPUT_UNKNOWN_CURVE_TYPE             -27
#define RD_INPUT_UNKNOWN_PATCH_TYPE             -28

#define RD_INPUT_TRANSFORM_STACK_OVERFLOW       -29
#define RD_INPUT_TRANSFORM_STACK_UNDERFLOW      -30

#define RD_INPUT_MAP_LIST_OVERFLOW              -31
#define RD_INPUT_MAP_FILE_NOT_FOUND             -32
#define RD_INPUT_MAP_BAD_FORMAT                 -33
#define RD_INPUT_MAP_ILLEGAL_LABEL              -34
#define RD_INPUT_MAP_NOT_FOUND                  -35
#define RD_INPUT_MAP_UNKNOWN_MAP_TYPE           -36
#define RD_INPUT_MAP_BAD_INTERP                 -37
#define RD_INPUT_MAP_BAD_BORDER                 -38

#define RD_INPUT_UNKNOWN_ENGINE                 -39

#define RD_INPUT_NON_INTEGER_VALUE              -40
#define RD_INPUT_EXPECTED_VERTEX_INDEX          -41
#define RD_INPUT_EXPECTED_VERTEX_COORD          -42
#define RD_INPUT_EXPECTED_REAL                  -43

#define RD_INPUT_UNKNOWN_OPTION                 -44

#define RD_INPUT_BAD_BSPLINE_DEGREE             -45

#define RD_INPUT_BAD_OBJECT_BLOCK_PARAMETER     -46
#define RD_INPUT_BAD_OBJECT_INSTANCE_PARAMETER  -47

#if defined(__cplusplus)
extern "C"
{
#endif

  void rd_print_error(int err, const char * filename);

#if defined (__cplusplus)
}
#endif

#endif /* RD_ERROR_H */
