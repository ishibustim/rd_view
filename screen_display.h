#ifndef SCREEN_DISPLAY_H
#define SCREEN_DISPLAY_H

int screen_init_display_single(void);
  
int screen_init_display_double(void);

int screen_end_display(void);
  
int screen_init_frame(int frame_no);
  
int screen_end_frame_single(void);

int screen_end_frame_double(void);
  
int screen_write_pixel(int x, int y, const float rgb []);
  
int screen_read_pixel(int x, int y, float rgb []);
  
int screen_set_background(const float rgb []);

int screen_clear(void);


#endif /* SCREEN_DISPLAY_H */
