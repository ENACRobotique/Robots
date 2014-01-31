#include <stdlib.h> // abs

#include "tools.h" // CLAMP

#include "video_draw.h"

void video_draw_pixel(unsigned char *rgb, unsigned int rowstride, unsigned int h, unsigned int x, unsigned int y, unsigned char r, unsigned char g, unsigned char b) {
    if(x>=0 && x*3+2<rowstride && y>=0 && y<h) {
        rgb[y*rowstride + x*3 + 0] = r;
        rgb[y*rowstride + x*3 + 1] = g;
        rgb[y*rowstride + x*3 + 2] = b;
    }
}

void video_draw_line(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned char r, unsigned char g, unsigned char b) {
    // Bresenham algorithm

    x0 = CLAMP(0, x0, w-1);
    y0 = CLAMP(0, y0, h-1);
    x1 = CLAMP(0, x1, w-1);
    y1 = CLAMP(0, y1, h-1);

    int dx = abs((int)x1-(int)x0), sx = x0<x1 ? 1 : -1;
    int dy = abs((int)y1-(int)y0), sy = y0<y1 ? 1 : -1;
    int err = (dx>dy ? dx : -dy)/2, e2;

    for(;;) {
        video_draw_pixel(rgb, rowstride, h, x0, y0, r, g, b);

        if(x0==x1 && y0==y1) break;
        e2 = err;
        if(e2 >-dx) { err -= dy; x0 += sx; }
        if(e2 < dy) { err += dx; y0 += sy; }
    }
}

void video_draw_cross(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, unsigned int x, unsigned int y, unsigned int size, unsigned char r, unsigned char g, unsigned char b) {
    video_draw_line(rgb, w, h, rowstride, x-size, y, x+size, y, r, g, b);
    video_draw_line(rgb, w, h, rowstride, x, y-size, x, y+size, r, g, b);
}

void video_draw_arrow(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, unsigned int x, unsigned int y, int dx, int dy, unsigned int size, unsigned char r, unsigned char g, unsigned char b) {
    video_draw_line(rgb, w, h, rowstride, x, y, x+dx, y+dy, r, g, b);
    video_draw_line(rgb, w, h, rowstride, x+dx, y+dy, x+dx-(dx-dy)*2/10, y+dy-(dx+dy)*2/10, r, g, b);
    video_draw_line(rgb, w, h, rowstride, x+dx, y+dy, x+dx-(dx+dy)*2/10, y+dy-(-dx+dy)*2/10, r, g, b);
}

void video_draw_circle(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, unsigned int xc, unsigned int yc, unsigned int ray, unsigned char r, unsigned char g, unsigned char b) {
    unsigned int x= ray, y= 0;//local coords
    int          cd2= 0;    //current distance squared - radius squared

    video_draw_pixel(rgb, rowstride, h, xc-ray, yc, r, g, b);
    video_draw_pixel(rgb, rowstride, h, xc+ray, yc, r, g, b);
    video_draw_pixel(rgb, rowstride, h, xc, yc-ray, r, g, b);
    video_draw_pixel(rgb, rowstride, h, xc, yc+ray, r, g, b);

    while (x > y)    //only formulate 1/8 of circle
    {
        cd2-= (--x) - (++y);
        if (cd2 < 0) cd2+=x++;

        video_draw_pixel(rgb, rowstride, h, xc-x, yc-y, r, g, b);//upper left left
        video_draw_pixel(rgb, rowstride, h, xc-y, yc-x, r, g, b);//upper upper left
        video_draw_pixel(rgb, rowstride, h, xc+y, yc-x, r, g, b);//upper upper right
        video_draw_pixel(rgb, rowstride, h, xc+x, yc-y, r, g, b);//upper right right
        video_draw_pixel(rgb, rowstride, h, xc-x, yc+y, r, g, b);//lower left left
        video_draw_pixel(rgb, rowstride, h, xc-y, yc+x, r, g, b);//lower lower left
        video_draw_pixel(rgb, rowstride, h, xc+y, yc+x, r, g, b);//lower lower right
        video_draw_pixel(rgb, rowstride, h, xc+x, yc+y, r, g, b);//lower right right
     }
}
