#ifndef _VIDEO_DRAW_H
#define _VIDEO_DRAW_H

void video_draw_pixel(unsigned char *rgb, unsigned int rowstride, unsigned int h, int x, int y, unsigned char r, unsigned char g, unsigned char b, unsigned char a);

void video_draw_line(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int x0, int y0, int x1, int y1, unsigned char r, unsigned char g, unsigned char b, unsigned char a);
void video_draw_rectangle(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int x0, int y0, int x1, int y1, unsigned char r, unsigned char g, unsigned char b, unsigned char a);
void video_draw_filled_rectangle(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int x0, int y0, int x1, int y1, unsigned char r, unsigned char g, unsigned char b, unsigned char a);
void video_draw_cross(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int x, int y, unsigned int size, unsigned char r, unsigned char g, unsigned char b, unsigned char a);
void video_draw_arrow(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int x, int y, int dx, int dy, unsigned int size, unsigned char r, unsigned char g, unsigned char b, unsigned char a);
void video_draw_circle(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int xc, int yc, unsigned int ray, unsigned char r, unsigned char g, unsigned char b, unsigned char a);
void video_draw_filled_circle(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int xc, int yc, unsigned int ray, unsigned char r, unsigned char g, unsigned char b, unsigned char a);
void video_draw_arc(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int xc, int yc, unsigned int ray, int x1, int y1, int x2, int y2, unsigned char r, unsigned char g, unsigned char b, unsigned char a);

#define BLACK() 0, 0, 0
#define WHITE() 255, 255, 255
#define GREY(i) (i), (i), (i)

#define RED(i) (i), 0, 0
#define GREEN(i) 0, (i), 0
#define BLUE(i) 0, 0, (i)

#define YELLOW(i) (i), (i), 0

#define ORANGE(i) (i), (i)>>1, 0

#endif
