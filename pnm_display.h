#ifndef PNM_DISPLAY_H
#define PNM_DISPLAY_H

#if defined (__cplusplus)
extern "C"
{
#endif

  int pnm_init_display(void);

  int pnm_end_display(void);

  int pnm_init_frame(int);

  int pnm_end_frame(void);

  int pnm_write_pixel(int x, int y, const float rgb []);

  int pnm_read_pixel(int x, int y, float rgb []);

  int pnm_set_background(const float rgb []);

  int pnm_clear(void);

#if defined (__cplusplus)
}
#endif

#endif /* PNM_DISPLAY_H */
