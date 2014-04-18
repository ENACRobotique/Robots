/*
 * video_draw_strings.h
 *
 *  Created on: 18 mars 2014
 *      Author: ludo6431
 */

#ifndef VIDEO_DRAW_STRINGS_H_
#define VIDEO_DRAW_STRINGS_H_

#include <iconv.h> // iconv*
#include <ft2build.h>
#include FT_FREETYPE_H

typedef struct{
    iconv_t ic;

    FT_Library library;
    FT_Face face;

} sVDSContext;

int video_draw_strings_init(sVDSContext *ctx, const char *font);
void video_draw_strings_deinit(sVDSContext *ctx);
int video_draw_string(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, sVDSContext *ctx, int x, int y, unsigned int sh, char *utf8_s, unsigned char r, unsigned char g, unsigned char b, unsigned char a);

#endif /* VIDEO_DRAW_STRINGS_H_ */
