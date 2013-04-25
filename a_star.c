#include "math_ops.h"

#include "a_star.h"

//#define AS_DEBUG

struct {
    iABObs_t next;   // in openset
    iABObs_t prev;   // in path

    sNum_t g_score;
    sNum_t f_score;

    uint8_t closedset:4;  // boolean
    uint8_t openset:4;  // boolean
} _aselts[N*2];

iABObs_t *a_star(iABObs_t start, iABObs_t goal) {
    iABObs_t os_start, current, prev, curr;
    int i, neighbor;
    sNum_t tmp_g_score, d;
    sSeg_t *seg;
    static iABObs_t _path[2*N+1];

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
#ifdef AS_DEBUG
printf("current %u%c\n", O(current), DIR(current)?'b':'a');
#endif
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
                tmp_g_score = arc_len(_aselts[current].prev, current, neighbor);
            }
            else {  // no previous element (starting point case)
                tmp_g_score = 0;
            }
            seg = tgt(current, neighbor);
            distPt2Pt(&seg->p1, &seg->p2, &d);
#ifdef AS_DEBUG
printf("    tmp_g_score = %.2f + %.2f + %.2f", _aselts[current].g_score, tmp_g_score, d);
#endif
            tmp_g_score += _aselts[current].g_score + d;
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

    return NULL;
}

