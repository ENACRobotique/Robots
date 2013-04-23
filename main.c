#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "error.h"

#include "math_types.h"
#include "math_ops.h"
#include "tools.h"


#define NOELT ((sAsEl_t)-1)

typedef int8_t sAsEl_t;

sAsEl_t _path[2*N+1];

struct {
    sAsEl_t next;   // in openset
    sAsEl_t prev;   // in path

    sNum_t g_score;
    sNum_t f_score;

    uint8_t closedset:4;  // boolean
    uint8_t openset:4;  // boolean
} _aselts[N*2];

sAsEl_t *a_star(sAsEl_t start, sAsEl_t goal) {
    sAsEl_t os_start, current, prev, curr;
    int i, neighbor;
    sNum_t tmp_g_score, d;
    sSeg_t *seg;

    start = A(start);
    goal = A(goal);

    os_start = start;

    for(i = 0; i < 2*N; i++) {
        _aselts[i].openset = 0;
        _aselts[i].closedset = 0;
    }

    _aselts[os_start].next = NOELT;
    _aselts[os_start].prev = NOELT;
    _aselts[os_start].g_score = 0;  // or the in-place rotation
    _aselts[os_start].f_score = _aselts[os_start].g_score + DIST(O(os_start), O(goal));
    _aselts[os_start].closedset = 0;
    _aselts[os_start].openset = 1;

    while(os_start != NOELT) {

        // get the element with the lowest f_score
        current = os_start;

        if(current == goal) {
            // reconstruct path
            _path[2*N] = NOELT;
            for(i = 2*N-1; i>=0 && current!=NOELT; i--) {
                _path[i] = current;

                current = _aselts[current].prev;
            }

            return &_path[i+1];
        }

        // remove current from openset
        os_start = _aselts[os_start].next;
        _aselts[current].openset = 0;
        // add current to closedset
        _aselts[current].closedset = 1;

        for(neighbor = 0; neighbor < 2*N; neighbor++) {
            // keep only valid neighbors
            if(!lnk[current][neighbor])
                continue;

            // compute g_score of neighbor
            if(_aselts[current].prev != NOELT) {
                if(!check_arc(_aselts[current].prev, current, neighbor))
                    continue;

                tmp_g_score = arc_len(_aselts[current].prev, current, neighbor);
            }
            else {  // no previous element (starting point case)
                tmp_g_score = 0;
            }
            seg = tgt(current, neighbor);
            distPt2Pt(&seg->p1, &seg->p2, &d);
            tmp_g_score += _aselts[current].g_score + d;

            // if already evaluated and worse cost, continue
            if(_aselts[neighbor].closedset && tmp_g_score >= _aselts[neighbor].g_score) {
                continue;
            }

            if(!_aselts[neighbor].openset || tmp_g_score < _aselts[neighbor].g_score) {
                _aselts[neighbor].prev = current;
                _aselts[neighbor].g_score = tmp_g_score;
                _aselts[neighbor].f_score = _aselts[neighbor].g_score + DIST(O(neighbor), O(goal));
                if(!_aselts[neighbor].openset) {
                    prev = NOELT;
                    for(curr = os_start; curr!=NOELT && _aselts[curr].f_score < _aselts[neighbor].f_score; prev = curr, curr = _aselts[curr].next);
                    _aselts[neighbor].next = curr;

                    if(prev != NOELT)
                        _aselts[prev].next = neighbor;
                    else
                        os_start = neighbor;

                    _aselts[neighbor].openset = 1;
                }
            }
        }
    }

    return NULL;
}

ERROR main(int argc, char *argv[]) {
    uint8_t i, j;
    sAsEl_t *path;

    // entry point
    printf("N=%u\n", N);    // number of elements

    printf("sizeof(obs) =%uB\n", sizeof(obs));
    printf("sizeof(tgts)=%uB\n", sizeof(tgts));
    printf("sizeof(lnk) =%uB\n", sizeof(lnk));

    printf("\n");

    // fill
    get_links();

    // show
    printf("possible links matrix:\n  ");
    for(i=0; i<2*N; i++) {
        printf(" %u%c", (i>>1), obs[i>>1].r?(i&1?'b':'a'):' ');
    }
    printf("\n");
    for(i=0; i<2*N; i++) {
        printf("%u%c", (i>>1), obs[i>>1].r?(i&1?'b':'a'):' ');
        for(j=0; j<2*N; j++) {
            printf("%c%c ", j?',':' ', lnk[i][j]?'1':' ');
        }
        printf("\n");
    }

    // A* test
    printf("\n\nA* test\n");
    path = a_star(0, 5);
    printf("path from 0 to 3:\n");
    if(path)
        for(; *path!=NOELT; path++) {
            printf("  obs %u%c\n", O(*path), DIR(*path)?'b':'a');
        }

	return 0;
}

