#include <stdlib.h> // abs

#include "tools.h" // CLAMP

#include "video_draw.h"

void video_draw_pixel(unsigned char *rgb, unsigned int rowstride, unsigned int h, int x, int y, unsigned char r, unsigned char g, unsigned char b, unsigned char a){
    if(x>=0 && x*3+2<rowstride && y>=0 && y<h) {
        rgb[y*rowstride + x*3 + 0] = ((int)rgb[y*rowstride + x*3 + 0]*(255 - (int)a) + (int)r*(int)a)/255;
        rgb[y*rowstride + x*3 + 1] = ((int)rgb[y*rowstride + x*3 + 1]*(255 - (int)a) + (int)g*(int)a)/255;
        rgb[y*rowstride + x*3 + 2] = ((int)rgb[y*rowstride + x*3 + 2]*(255 - (int)a) + (int)b*(int)a)/255;
    }
}

void video_draw_line(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int x0, int y0, int x1, int y1, unsigned char r, unsigned char g, unsigned char b, unsigned char a){
    // Bresenham algorithm
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = (dx > dy ? dx : -dy)/2, e2;

    for(;;) {
        video_draw_pixel(rgb, rowstride, h, x0, y0, r, g, b, a);

        if(x0 == x1 && y0 == y1) break;
        e2 = err;
        if(e2 > -dx) { err -= dy; x0 += sx; }
        if(e2 < dy) { err += dx; y0 += sy; }
    }
}

void video_draw_rectangle(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int x0, int y0, int x1, int y1, unsigned char r, unsigned char g, unsigned char b, unsigned char a){
    video_draw_line(rgb, w, h, rowstride, x0, y0, x0, y1, r, g, b, a);
    video_draw_line(rgb, w, h, rowstride, x0, y0, x1, y0, r, g, b, a);
    video_draw_line(rgb, w, h, rowstride, x0, y1, x1, y1, r, g, b, a);
    video_draw_line(rgb, w, h, rowstride, x1, y0, x1, y1, r, g, b, a);
}

void video_draw_filled_rectangle(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int x0, int y0, int x1, int y1, unsigned char r, unsigned char g, unsigned char b, unsigned char a){
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int i;

    if(dx > dy){
        for(i = y0; i != y1; i += sy){
            video_draw_line(rgb, w, h, rowstride, x0, i, x1, i, r, g, b, a);
        }
        video_draw_line(rgb, w, h, rowstride, x0, i, x1, i, r, g, b, a);
    }
    else{
        for(i = x0; i != x1; i += sx){
            video_draw_line(rgb, w, h, rowstride, i, y0, i, y1, r, g, b, a);
        }
        video_draw_line(rgb, w, h, rowstride, i, y0, i, y1, r, g, b, a);
    }
}

void video_draw_cross(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int x, int y, unsigned int size, unsigned char r, unsigned char g, unsigned char b, unsigned char a){
    video_draw_line(rgb, w, h, rowstride, x - size, y, x + size, y, r, g, b, a);
    video_draw_line(rgb, w, h, rowstride, x, y - size, x, y + size, r, g, b, a);
}

void video_draw_arrow(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int x, int y, int dx, int dy, unsigned int size, unsigned char r, unsigned char g, unsigned char b, unsigned char a){
    video_draw_line(rgb, w, h, rowstride, x, y, x + dx, y + dy, r, g, b, a);
    video_draw_line(rgb, w, h, rowstride, x+dx, y + dy, x + dx - (dx - dy)*2/10, y + dy - (dx + dy)*2/10, r, g, b, a);
    video_draw_line(rgb, w, h, rowstride, x+dx, y + dy, x + dx - (dx + dy)*2/10, y + dy - (-dx + dy)*2/10, r, g, b, a);
}

void video_draw_circle(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int xc, int yc, unsigned int ray, unsigned char r, unsigned char g, unsigned char b, unsigned char a){
    // Original code from http://willperone.net/Code/codecircle.php
    // + modifications to draw each pixel exactly once by L. Lacoste <ludovic.lacoste@gmail.com>
    int x = ray, y = 0;//local coords
    int cd2 = 0;    //current distance squared - radius squared

    video_draw_pixel(rgb, rowstride, h, xc - x, yc, r, g, b, a);
    video_draw_pixel(rgb, rowstride, h, xc + x, yc, r, g, b, a);
    video_draw_pixel(rgb, rowstride, h, xc, yc - x, r, g, b, a);
    video_draw_pixel(rgb, rowstride, h, xc, yc + x, r, g, b, a);

    while(x > y + 2){    //only formulate 1/8 of circle
        cd2 -= (--x) - (++y);
        if(cd2 < 0) cd2 += x++;

        video_draw_pixel(rgb, rowstride, h, xc - x, yc - y, r, g, b, a);//upper left left
        video_draw_pixel(rgb, rowstride, h, xc + x, yc - y, r, g, b, a);//upper right right
        video_draw_pixel(rgb, rowstride, h, xc - x, yc + y, r, g, b, a);//lower left left
        video_draw_pixel(rgb, rowstride, h, xc + x, yc + y, r, g, b, a);//lower right right

        video_draw_pixel(rgb, rowstride, h, xc - y, yc - x, r, g, b, a);//upper upper left
        video_draw_pixel(rgb, rowstride, h, xc + y, yc - x, r, g, b, a);//upper upper right
        video_draw_pixel(rgb, rowstride, h, xc - y, yc + x, r, g, b, a);//lower lower left
        video_draw_pixel(rgb, rowstride, h, xc + y, yc + x, r, g, b, a);//lower lower right
    }

    cd2 -= (--x) - (++y);
    if(cd2 < 0) cd2 += x++;

    if(x == y){
        video_draw_pixel(rgb, rowstride, h, xc - x, yc - y, r, g, b, a);//upper left left
        video_draw_pixel(rgb, rowstride, h, xc + x, yc - y, r, g, b, a);//upper right right
        video_draw_pixel(rgb, rowstride, h, xc - x, yc + y, r, g, b, a);//lower left left
        video_draw_pixel(rgb, rowstride, h, xc + x, yc + y, r, g, b, a);//lower right right
    }
}

void video_draw_filled_circle(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int xc, int yc, unsigned int ray, unsigned char r, unsigned char g, unsigned char b, unsigned char a){
    // Original code for empty circle from http://willperone.net/Code/codecircle.php
    // + modifications to draw it filled and draw each pixel exactly once by L. Lacoste <ludovic.lacoste@gmail.com>
    int x = ray, y = 0;//local coords
    int cd2 = 0;    //current distance squared - radius squared

    video_draw_line(rgb, w, h, rowstride, xc - x, yc, xc + x, yc, r, g, b, a);

    while(x > y + 1){    //only formulate 1/8 of circle
        cd2 -= (--x) - (++y);
        if(cd2 < 0) cd2 += x++;
        else{
            video_draw_line(rgb, w, h, rowstride, xc - (y-1), yc - (x+1), xc + (y-1), yc - (x+1), r, g, b, a);
            video_draw_line(rgb, w, h, rowstride, xc - (y-1), yc + (x+1), xc + (y-1), yc + (x+1), r, g, b, a);
        }

        video_draw_line(rgb, w, h, rowstride, xc - x, yc - y, xc + x, yc - y, r, g, b, a);
        video_draw_line(rgb, w, h, rowstride, xc - x, yc + y, xc + x, yc + y, r, g, b, a);
    }

    cd2 -= (--x) - (++y);
    if(cd2 < 0) cd2 += x++;

    if(x == y - 1){
        video_draw_line(rgb, w, h, rowstride, xc - x, yc - y, xc + x, yc - y, r, g, b, a);
        video_draw_line(rgb, w, h, rowstride, xc - x, yc + y, xc + x, yc + y, r, g, b, a);
    }
}

void video_draw_arc(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, int xc, int yc, unsigned int ray, int x1, int y1, int x2, int y2, unsigned char r, unsigned char g, unsigned char b, unsigned char a){
    // Original code from http://willperone.net/Code/codecircle.php
    // + modifications to draw only an arc and draw each pixel exactly once by L. Lacoste <ludovic.lacoste@gmail.com>
    int x = ray, y = 0;//local coords
    int cd2 = 0;    //current distance squared - radius squared

    if((x1 - xc)*(y2 - yc) - (y1 - yc)*(x2 - xc) > 0){
#define ARC_CHECK(X, Y) \
        if((X)*(yc - y1) + (Y)*(x1 - xc) >= 0 && (X)*(yc - y2) + (Y)*(x2 - xc) <= 0){ \
            video_draw_pixel(rgb, rowstride, h, xc + (X), yc + (Y), r, g, b, a); \
        }

        ARC_CHECK(-x, 0);
        ARC_CHECK(x, 0);
        ARC_CHECK(0, -x);
        ARC_CHECK(0, x);

        while(x > y + 2){    //only formulate 1/8 of circle
            cd2 -= (--x) - (++y);
            if(cd2 < 0) cd2 += x++;

            ARC_CHECK(-x, -y);//upper left left
            ARC_CHECK(-y, -x);//upper upper left
            ARC_CHECK(y, -x);//upper upper right
            ARC_CHECK(x, -y);//upper right right
            ARC_CHECK(-x, y);//lower left left
            ARC_CHECK(-y, x);//lower lower left
            ARC_CHECK(y, x);//lower lower right
            ARC_CHECK(x, y);//lower right right
        }

        cd2 -= (--x) - (++y);
        if(cd2 < 0) cd2 += x++;

        if(x == y){
            ARC_CHECK(-x, -y);//upper left left
            ARC_CHECK(x, -y);//upper right right
            ARC_CHECK(-x, y);//lower left left
            ARC_CHECK(x, y);//lower right right
        }

#undef ARC_CHECK
    }
    else{
#define ARC_CHECK(X, Y) \
        if((X)*(yc - y1) + (Y)*(x1 - xc) <= 0 || (X)*(yc - y2) + (Y)*(x2 - xc) >= 0){ \
            video_draw_pixel(rgb, rowstride, h, xc + (X), yc + (Y), r, g, b, a); \
        }

        ARC_CHECK(-x, 0);
        ARC_CHECK(x, 0);
        ARC_CHECK(0, -x);
        ARC_CHECK(0, x);

        while(x > y + 2){    //only formulate 1/8 of circle
            cd2 -= (--x) - (++y);
            if(cd2 < 0) cd2 += x++;

            ARC_CHECK(-x, -y);//upper left left
            ARC_CHECK(-y, -x);//upper upper left
            ARC_CHECK(y, -x);//upper upper right
            ARC_CHECK(x, -y);//upper right right
            ARC_CHECK(-x, y);//lower left left
            ARC_CHECK(-y, x);//lower lower left
            ARC_CHECK(y, x);//lower lower right
            ARC_CHECK(x, y);//lower right right
        }

        cd2 -= (--x) - (++y);
        if(cd2 < 0) cd2 += x++;

        if(x == y){
            ARC_CHECK(-x, -y);//upper left left
            ARC_CHECK(x, -y);//upper right right
            ARC_CHECK(-x, y);//lower left left
            ARC_CHECK(x, y);//lower right right
        }

#undef ARC_CHECK
    }
}
