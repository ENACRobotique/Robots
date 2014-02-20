#include "math_ops.h"
#include <stdlib.h>
#include "a_star.h"

//#define AS_DEBUG

struct {
    sNum_t g_score;
    sNum_t f_score;

    sNum_t seg_len;
    sNum_t arc_len;

    iABObs_t next;   // in openset
    iABObs_t prev;   // in path

    uint8_t closedset:4;  // boolean
    uint8_t openset:4;  // boolean
} _aselts[N*2];

void a_star(iABObs_t start, iABObs_t goal, sPath_t *path) {
    iABObs_t os_start, current, prev, curr;
    int i, neighbor;
    sNum_t tmp_g_score, s_l, a_l;
    sSeg_t *seg;

    if(!path)
        return;

    os_start = start;

#ifdef AS_DEBUG
printf("goal %u%c\n", O(goal), DIR(goal)?'b':'a');
#endif

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
#ifdef AS_DEBUG
printf("current %u%c\n", O(current), DIR(current)?'b':'a');
#endif
        if(current == goal) {
            for (i = 0, curr = current; curr != start; i++, curr = _aselts[curr].prev); // oh putaing!
            path->path_len = i;
            path->path = (sTrajEl_t*) malloc ( path->path_len * sizeof(sTrajEl_t));
            if(!path->path)
                break;

            path->dist = _aselts[goal].g_score;

            for (i = path->path_len-1; i >= 0; i--) {
                seg = tgt(_aselts[current].prev, current);
                path->path[i].p1 = seg->p1;
                path->path[i].p2 = seg->p2;
                path->path[i].obs = obs[O(current)];
                path->path[i].sid = i;
                if(i)
                    path->path[i-1].arc_len = _aselts[current].arc_len;
                path->path[i].seg_len = _aselts[current].seg_len;

                current = _aselts[current].prev;
            }
            
            return;
        }

        // remove current from openset
        os_start = _aselts[os_start].next;
        _aselts[current].openset = 0;
        // add current to closedset
        _aselts[current].closedset = 1;

        for(neighbor = 0; neighbor < 2*N; neighbor++) {
            // keep only valid neighbors
            if(!lnk[current][neighbor] || !obs[O(neighbor)].active)
                continue;
#ifdef AS_DEBUG
printf("  neighbor %u%c\n", O(neighbor), DIR(neighbor)?'b':'a');
#endif

            // compute g_score of neighbor
            if(_aselts[current].prev != NOELT) {
                if(!check_arc(_aselts[current].prev, current, neighbor)) {
#ifdef AS_DEBUG
printf("    bad arc\n");
#endif
                    continue;
                }

#ifdef AS_DEBUG
printf("    arc_len(%u%c, %u%c, %u%c)\n", O(_aselts[current].prev),DIR(_aselts[current].prev)?'b':'a', O(current), DIR(current)?'b':'a', O(neighbor), DIR(neighbor)?'b':'a');
#endif
                tmp_g_score = a_l = arc_len(_aselts[current].prev, current, neighbor);
            }
            else {  // no previous element (starting point case)
                tmp_g_score = 0.;
                a_l = 0.;
            }
            seg = tgt(current, neighbor);
            distPt2Pt(&seg->p1, &seg->p2, &s_l);
#ifdef AS_DEBUG
printf("    tmp_g_score = %.2f + %.2f + %.2f", _aselts[current].g_score, tmp_g_score, s_l);
#endif
            tmp_g_score += _aselts[current].g_score + s_l;
#ifdef AS_DEBUG
printf(" = %.2f\n", tmp_g_score);
#endif

            // if already evaluated and worse cost, continue
            if(_aselts[neighbor].closedset && tmp_g_score >= _aselts[neighbor].g_score) {
#ifdef AS_DEBUG
printf("    worse than before\n");
#endif
                continue;
            }

            if(!_aselts[neighbor].openset || tmp_g_score < _aselts[neighbor].g_score) {
                _aselts[neighbor].prev = current;
                _aselts[neighbor].g_score = tmp_g_score;
                _aselts[neighbor].f_score = _aselts[neighbor].g_score + DIST(O(neighbor), O(goal));
                _aselts[neighbor].arc_len = a_l;
                _aselts[neighbor].seg_len = s_l;
                
                if(!_aselts[neighbor].openset) {
                    prev = NOELT;
                    for(curr = os_start; curr!=NOELT && _aselts[curr].f_score < _aselts[neighbor].f_score; prev = curr, curr = _aselts[curr].next);
                    _aselts[neighbor].next = curr;

                    if(prev != NOELT)
                        _aselts[prev].next = neighbor;
                    else
                        os_start = neighbor;

                    _aselts[neighbor].openset = 1;
#ifdef AS_DEBUG
printf("    adding to openset\n");

#endif
                }
#ifdef AS_DEBUG
else
printf("    updating in openset\n");
#endif
            }
#ifdef AS_DEBUG
else
printf("    nothing...\n");
#endif
        }
    }

    // no solution
    path->path_len = 0;
    path->dist = 0.;
    path->path = NULL;

#ifdef AS_DEBUG
printf("no solution\n");
#endif
}

