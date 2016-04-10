#ifndef RD_REFRESH_H
#define RD_REFRESH_H

enum refresh_point_type{refresh_pixel, refresh_object, refresh_frame};

extern bool engine_refresh[3];
extern bool display_refresh[3];

inline bool refresh_query(refresh_point_type i){return engine_refresh[i] 
						  && display_refresh[i];}
#endif /* RD_REFRESH_H */
