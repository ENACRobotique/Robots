/*
 * video_draw_strings.c
 *
 *  Created on: 18 mars 2014
 *      Author: ludo6431
 */

#include <stdio.h> // perror
#include <assert.h> // assert
#include <stdint.h> // uint32_t

#include "video_draw.h"

#include "video_draw_strings.h"

int video_draw_strings_init(sVDSContext *ctx, const char *font){
    FT_Error ft_error;

    assert(ctx);

    // iconv initialization
    ctx->ic = iconv_open("WCHAR_T", "UTF-8");
    if(ctx->ic == (iconv_t)-1){
        perror("iconv_open() @ video_draw_string");
        return -1;
    }

    // Freetype initialization
    ft_error = FT_Init_FreeType(&ctx->library);
    if(ft_error){
        fprintf(stderr, "FT_Init_FreeType error #%i\n", ft_error);
        return -1;
    }

    // load font
    ft_error = FT_New_Face(ctx->library, font, 0, &ctx->face);
    if(ft_error){
        fprintf(stderr, "FT_New_Face error #%i\n", ft_error);
        return -1;
    }

    return 0;
}

void video_draw_strings_deinit(sVDSContext *ctx){
    assert(ctx);

    iconv_close(ctx->ic);

    FT_Done_Face(ctx->face);
    FT_Done_FreeType(ctx->library);
}

int video_draw_string(unsigned char *rgb, unsigned int w, unsigned int h, unsigned int rowstride, sVDSContext *ctx, int x, int y, unsigned int sh, char *utf8_s, unsigned char r, unsigned char g, unsigned char b, unsigned char a){
    FT_Error ft_error;
    FT_GlyphSlot slot;
    FT_Vector pen;
    size_t ic_error;
    size_t in_len, out_len;
    int i, j, u, v;
    char *_out_s;
    wchar_t *out_s;

    assert(ctx);

    in_len = strlen(utf8_s);
    out_len = in_len*4;
    _out_s = (char*)malloc(out_len);
    if(!_out_s){
        return -1;
    }
    out_s = (wchar_t *)_out_s;

    slot = ctx->face->glyph;
    pen.x = x<<6;
    pen.y = ((int)h - y)<<6;

    // set character size
    ft_error = FT_Set_Pixel_Sizes(ctx->face, 0, sh);
    if(ft_error){
        free(_out_s);
        fprintf(stderr, "FT_Set_Pixel_Sizes ft_error #%i\n", ft_error);
        return -1;
    }

    ic_error = iconv(ctx->ic, &utf8_s, &in_len, (char**)&out_s, &out_len);
    if(ic_error == (size_t)-1){
        free(_out_s);
        printf("\n");
        perror("iconv() @ video_draw_string");
        return -1;
    }

    if(in_len != 0){
        fprintf(stderr, "in_len not zero\n");
        return -1;
    }

    out_len = out_s - (wchar_t*)_out_s;
    out_s = (wchar_t*)_out_s;

    while(out_len > 0){
        FT_Set_Transform(ctx->face, NULL, &pen);

        ft_error = FT_Load_Char(ctx->face, *out_s, FT_LOAD_RENDER);
        if(ft_error){
            free(_out_s);
            fprintf(stderr, "FT_Load_Char ft_error #%i\n", ft_error);
            return -1;
        }

        out_s++;
        out_len--;

        if(slot->format != FT_GLYPH_FORMAT_BITMAP){
            free(_out_s);
            fprintf(stderr, "unhandled glyph format\n");
            return -1;
        }

        if(slot->bitmap.pixel_mode != FT_PIXEL_MODE_GRAY){
            free(_out_s);
            fprintf(stderr, "unhandled glyph pixel mode\n");
            return -1;
        }

        for(i = (int)h - slot->bitmap_top, v = 0; v < slot->bitmap.rows; v++, i++){
            for(j = slot->bitmap_left, u = 0; u < slot->bitmap.width; u++, j++){
                video_draw_pixel(rgb, rowstride, h, j, i, r, g, b, a*slot->bitmap.buffer[v * slot->bitmap.width + u]/(slot->bitmap.num_grays - 1));
            }
        }

        pen.x += slot->advance.x;
        pen.y += slot->advance.y;
    }

    free(_out_s);

    return 0;
}
