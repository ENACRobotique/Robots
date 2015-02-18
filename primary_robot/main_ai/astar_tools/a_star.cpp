#include "a_star.h"

#include "math_ops.h"
#include "tools.h"

extern "C"{
#include <stdlib.h>
#include <math.h>

#ifdef AS_STATS
#include "millis.h"
#endif
}


void a_star(iABObs_t _start, iABObs_t _goal, sPath_t *path) {
    int i, j;
    iABObs_t tmp;
    sNum_t tmp_g_score, s_l;
    sSeg_t *seg;
    sASNode_t os_start /* openset start */,
            start, goal ,
            neighbor, current, prev, curr,
            next;

    start.o1 = _start;
    start.o2 = _start;
    goal.o1 = _goal;
    goal.o2 = _goal;

#ifdef AS_STATS
    int nb_steps_current = 0, nb_steps_neighbor = 0;
    unsigned int start_us = micros();
#endif

    if (!path)
        return;

    // openset initialization
    os_start.o1 = os_start.o2 = _start;

#ifdef AS_DEBUG
    printf("goal [%u%c->%u%c]\n", O(goal.o1), DIR(goal.o1)?'b':'a', O(goal.o2), DIR(goal.o2)?'b':'a');
#endif

    for (i = 0; i < 2 * N; i++) {
        for (j = 0; j < 2 * N; j++) {
            aselts[i][j].openset = 0;
            aselts[i][j].closedset = 0;
        }
    }

    ASELT(os_start).next = ASNOELT;
    ASELT(os_start).prev = ASNOELT;
    ASELT(os_start).g_score = 0.;  // or the in-place rotation
    ASELT(os_start).f_score = ASELT(os_start).g_score + DIST(O(os_start.o2), O(goal.o1)) - obs[O(os_start.o2)].r;
    ASELT(os_start).closedset = 0;
    ASELT(os_start).openset = 1;

    while (os_start.o1 != ABNOELT) {
        // get the element with the lowest f_score
        current = os_start;
#ifdef AS_STATS
        nb_steps_current++;
#endif
#ifdef AS_DEBUG
        printf("current [%u%c->%u%c]\n", O(current.o1), DIR(current.o1)?'b':'a', O(current.o2), DIR(current.o2)?'b':'a');
#endif
        if (current.o2 == goal.o2) {
#ifdef AS_STATS
            printf("%i \"current\" steps and %i \"neighbor\" steps (%uµs)\n", nb_steps_current, nb_steps_neighbor, micros() - start_us);
#endif
#ifdef AS_DEBUG
            printf("  found solution (reverse order):\n");
#endif

            for (path->path_len = 1, curr = current;
                    curr.o1 != start.o1 && curr.o2 != start.o2;
                    path->path_len++, curr = ASELT(curr).prev); // oh putaing !
            path->path = (sTrajEl_t*) malloc(path->path_len * sizeof(sTrajEl_t));
            if (!path->path)
                break;

            path->dist = ASELT(current).g_score;

            for (i = path->path_len - 1; i >= 0; i--) {
#ifdef AS_DEBUG
                printf("    [%u%c->%u%c]\n", O(current.o1), DIR(current.o1)?'b':'a', O(current.o2), DIR(current.o2)?'b':'a');
#endif

                // segment section
                seg = tgt(current.o1, current.o2);
                path->path[i].p1 = seg->p1;
                path->path[i].p2 = seg->p2;
                distPt2Pt(&seg->p1, &seg->p2, &path->path[i].seg_len);
                // arc section
                path->path[i].obs = obs[O(current.o2)];
                path->path[i].obs.r = fabs(path->path[i].obs.r) * (1 - 2 * DIR(current.o2)); // r>0 CW/A ; r<0 CCW/B
                if (i == path->path_len - 1) {
                    path->path[i].arc_len = 0.;
                }
                if (i > 0) {
                    path->path[i - 1].arc_len = arc_len(ASELT(current).prev.o1, current.o1, current.o2);
                }
                // step id
                path->path[i].sid = i;

                current = ASELT(current).prev;
            }

            return;
        }

        // remove current from openset
        os_start = ASELT(os_start).next;
        ASELT(current).openset = 0;
        // add current to closedset
        ASELT(current).closedset = 1;

        for (tmp = 0; tmp < 2 * N; tmp++) {
            // keep only valid neighbors
            if (!aselts[current.o2][tmp].active /* invalid link */|| !obs[O(tmp)].active /* invalid obstacle */)
                continue;
            neighbor = ASNODE(current.o2, tmp);
#ifdef AS_STATS
            nb_steps_neighbor++;
#endif
#ifdef AS_DEBUG
            printf("  neighbor [%u%c->%u%c]\n", O(neighbor.o1), DIR(neighbor.o1)?'b':'a', O(neighbor.o2), DIR(neighbor.o2)?'b':'a');
#endif

            // compute g_score of neighbor
#ifdef AS_DEBUG
            printf("    check_arc(%u%c, %u%c, %u%c)\n", O(current.o1), DIR(current.o1)?'b':'a', O(neighbor.o1), DIR(neighbor.o1)?'b':'a', O(neighbor.o2), DIR(neighbor.o2)?'b':'a');
#endif
            if (!check_arc(current.o1, neighbor.o1, neighbor.o2)) {
#ifdef AS_DEBUG
                printf("      bad arc\n");
#endif
                continue;
            }

#ifdef AS_DEBUG
            printf("    arc_len(%u%c, %u%c, %u%c)\n", O(current.o1), DIR(current.o1)?'b':'a', O(neighbor.o1), DIR(neighbor.o1)?'b':'a', O(neighbor.o2), DIR(neighbor.o2)?'b':'a');
#endif
            tmp_g_score = arc_len(current.o1, neighbor.o1, neighbor.o2);

            seg = tgt(neighbor.o1, neighbor.o2);
            distPt2Pt(&seg->p1, &seg->p2, &s_l);
#ifdef AS_DEBUG
            printf("    tmp_g_score = %.2f + %.2f + %.2f", ASELT(current).g_score /* previous g_score */, tmp_g_score /* arc length */, s_l /* segment length */);
#endif
            tmp_g_score += ASELT(current).g_score + s_l;
#ifdef AS_DEBUG
            printf(" = %.2f\n", tmp_g_score);
#endif

            // if already evaluated and worse cost, continue
            if (ASELT(neighbor).closedset && tmp_g_score >= ASELT(neighbor).g_score) {
#ifdef AS_DEBUG
                printf("    worse than before (was %.2f)\n", ASELT(neighbor).g_score);
#endif
                continue;
            }

            if (!ASELT(neighbor).openset || tmp_g_score < ASELT(neighbor).g_score) {
                ASELT(neighbor).prev = current;
                ASELT(neighbor).g_score = tmp_g_score;
                ASELT(neighbor).f_score = ASELT(neighbor).g_score + DIST(O(neighbor.o2), O(goal.o1)) - obs[O(neighbor.o2)].r;

                if(!ASELT(neighbor).openset) {
                    prev = ASNOELT;
                    for(curr = os_start; curr.o1!=ABNOELT && ASELT(curr).f_score < ASELT(neighbor).f_score; prev = curr, curr = ASELT(curr).next);
                    ASELT(neighbor).next = curr;

                    if(prev.o1 != ABNOELT) {
                        ASELT(prev).next = neighbor;
                    }
                    else {
                        os_start = neighbor;
                    }

                    ASELT(neighbor).openset = 1;
#ifdef AS_DEBUG
                    printf("    adding to openset");
#endif
                }
                else {
                    prev = ASNOELT;
                    for(curr = os_start; curr.o1!=ABNOELT && ASELT(curr).f_score < ASELT(neighbor).f_score; prev = curr, curr = ASELT(curr).next);

                    if(neighbor.o1 != curr.o1 || neighbor.o2 != curr.o2) { // update needed
                        next = ASELT(neighbor).next;

                        ASELT(neighbor).next = curr;

                        if(prev.o1 != ABNOELT) {
                            ASELT(prev).next = neighbor;
                        }
                        else {
                            os_start = neighbor;
                        }

                        for(; curr.o1!=ABNOELT && (ASELT(curr).next.o1!=neighbor.o1 || ASELT(curr).next.o2!=neighbor.o2); curr = ASELT(curr).next);

                        if(curr.o1!=ABNOELT)
                        ASELT(curr).next = next;
                    }

#ifdef AS_DEBUG
                    printf("    updating in openset");
#endif
                }
#ifdef AS_DEBUG
                printf(" (f_score = %.2f + %.2f = %.2f)\n", ASELT(neighbor).g_score, DIST(O(neighbor.o2), O(goal.o1)) - obs[O(neighbor.o2)].r, ASELT(neighbor).f_score);
#endif
            }
#ifdef AS_DEBUG
            else {
                printf("    nothing...\n");
            }
#endif

#if defined(AS_DEBUG) && AS_DEBUG > 1
            printf("  openset:\n");
            tmp_g_score = 0.;
            for(i = 0, curr = os_start; curr.o1 != ABNOELT; i++, curr = ASELT(curr).next) {
                printf("   ");
                for(j = 0, prev = curr; prev.o1 != ABNOELT; j++, prev = ASELT(prev).prev) {
                    printf(" [%02u%c->%02u%c]", O(prev.o1), DIR(prev.o1)?'b':'a', O(prev.o2), DIR(prev.o2)?'b':'a');
                    if(j > 100) {
                        printf("\ninfinite loop j :/\n");
                        while(1) sleep(1);
                    }
                }
                printf(", f_score=%03.2f, g_score=%03.2f%s\n", ASELT(curr).f_score, ASELT(curr).g_score, ASELT(curr).f_score<tmp_g_score?" (error)":"");
                if(!ASELT(curr).openset) {
                    printf("error, in openset but not flag set\n");
                }
                if(i > 100) {
                    printf("infinite loop i :/\n");
                    while(1) sleep(1);
                }
                tmp_g_score = ASELT(curr).f_score;
            }
            printf("  (%i elements in openset)\n", i);
#endif
        }
    }

    // no solution
    path->path_len = 0;
    path->dist = 0.;
    path->path = NULL;

#ifdef AS_STATS
    printf("%i \"current\" steps and %i \"neighbor\" steps (%uµs)\n", nb_steps_current, nb_steps_neighbor, micros() - start_us);
#endif
#ifdef AS_DEBUG
    printf("no solution\n");
#endif
}
