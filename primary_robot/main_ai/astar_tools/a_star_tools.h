#ifndef _TOOLS_H
#define _TOOLS_H

#include <stdint.h>
#include <vector>
#include "error.h"
#include "math_types.h"

#define HOLONOMIC 1 // 0 if the robot isn't an holonomic robot (for backward compatibility)

//#define AS_DEBUG 1
//#define AS_STATS
// warning, activating AS_DEBUG adds a lot of time to the time statistics (time to print the debug information)



// ==== definitions ====

/*
 *  physical obstacle 0, 1, 2, ..., N-1
 *  logical obstacle 0a, 0b, 1a, 1b, ... (notion of direction of rotation)
 *  rotation direction:
 *      a:clockwise
 *      b:counter-clockwise
 */

#define LOW_THR ((sNum_t)0.001)

#define R_SECU (5.)
#define R_ROBOT (17.5)

#if 1
#   define X_MIN (R_ROBOT)
#   define X_MAX (300. - R_ROBOT)
#   define Y_MIN (R_ROBOT)
#   define Y_MAX (200. - R_ROBOT)
#else
#   define X_MIN (0.)
#   define X_MAX (300.)
#   define Y_MIN (0.)
#   define Y_MAX (200.)
#endif

#define OUT(x, y) ((x) > X_MAX || (x) < X_MIN || (y) > Y_MAX || (y) < Y_MIN)
// get quadrant from 0 to 3 in trigonometric rotation
#define QUADRANT(x, y) ( (y) > 0?( (x) > 0?0:1 ):( (x) > 0?3:2 ) )

// ==== common types ====

// an obstacle
typedef struct {
    sPt_t c;    // center of obstacle
    sNum_t r;   // radius
    //TODO change in circle type with the new geometry tools

    uint8_t moved:4;
    uint8_t active:4;
    uint8_t state; //0 free
} sObs_t;   // sizeof(sObs_t)=16


typedef enum{
    E_OBSPAIR_UNKNOWN,
    E_OBSPAIR_0S_CONCENTRIC,
    E_OBSPAIR_1S_LINE,
    E_OBSPAIR_0S_SURROUNDS,
    E_OBSPAIR_0S_SURROUNDED,
    E_OBSPAIR_2S_INTERSECTION,
    E_OBSPAIR_2S_POINT2CIRCLE,
    E_OBSPAIR_2S_CIRCLE2POINT,
    E_OBSPAIR_4S_FULL
} _type;

// a set of common tangents between two obstacles (from o1 to o2)
typedef struct {
    sSeg_t s1;  // first external (clock wise / a)
    sSeg_t s2;  // second external (counter clock wise / b)
    sSeg_t s3;  // first internal (counter clock wise / b)
    sSeg_t s4;  // second internal (clock wise / a)

    sNum_t d; // distance between obstacles

    _type type; // type of relation between those 2 obstacles
} sTgts_t;  // sizeof(sTgts_t)=72

// an index of obstacle between 0:N-1
typedef int8_t iObs_t;
// an index of obstacle between 0:2N-1
typedef int8_t iABObs_t;
#define ABNOELT ((iABObs_t)-1)

// between 0:2N-1
#define A(i) ((iABObs_t)( ((iObs_t)(i))<<1 ))
#define B(i) ((iABObs_t)( (((iObs_t)(i))<<1)+1 ))
// direction of a number in 0:2N-1
#define DIR(i) (((iABObs_t)(i))&1)

// between 0:N-1
#define O(i) ((iObs_t)( ((iABObs_t)(i))>>1 ))

// A* node (trajectory from o1 to o2)
typedef struct {
    iABObs_t o1; // oriented object
    iABObs_t o2;
} sASNode_t;
#define ASNODE(o1, o2) ((sASNode_t){(o1), (o2)})
#define ASNOELT ASNODE(ABNOELT, ABNOELT)

typedef struct __attribute__((packed)){ // XXX this attribute reduces the size of this structure from 16 to 13bytes but ~doubles the processing time of a_star()
                                        // aselts array is reduced by 12*N² bytes, you have to choose between speed and memory...
    uint8_t active :1;
    uint8_t closedset :1;
    uint8_t openset :1;

    sNum_t f_score;
    sNum_t g_score;

    sASNode_t next; // in openset
    sASNode_t prev; // in path
} sASEl_t;

// ==== global matrices ====

extern std::vector<sObs_t> obs;                     // array of N physical obstacles (256B)
extern int N;                                       // number of physical obstacles
extern std::vector<std::vector<sTgts_t>> tgts;      // NxN tangents between physical obstacles (17kiB)
extern std::vector<std::vector<sASEl_t>> aselts;    // 2Nx2N elements (an A* node is a trajectory from an iABObs_t to another)
extern std::vector<uint8_t> obs_updated;            // array not used by A*, available for the user
#define ASELT(n) aselts[(n).o1][(n).o2]
// NxN distances between obstacles
#define DIST(i, j) (tgts[(iObs_t)(i)][(iObs_t)(j)].d)

// ==== function prototypes ====

void init_obs(const std::vector<sObs_t>& init);
void fill_tgts_lnk();

// functions on objects (0:N-1)
uint8_t o_check_segment (iObs_t o1, sSeg_t *s, iObs_t o2);
uint8_t o_check_arc     (iObs_t o1, sPt_t *p2_1, iObs_t o2, int dir, sPt_t *p2_3, iObs_t o3);
sNum_t  o_arc_len       (sPt_t *p2_1, iObs_t o2, int dir, sPt_t *p2_3);

// functions on directed objects (0:N-1)
static inline sSeg_t *tgt(iABObs_t o1, iABObs_t o2) {
    if(DIR(o1))
        return DIR(o2) ? &tgts[O(o1)][O(o2)].s2 : &tgts[O(o1)][O(o2)].s3;
    else
        return DIR(o2) ? &tgts[O(o1)][O(o2)].s4 : &tgts[O(o1)][O(o2)].s1;
}
static inline uint8_t check_arc(iABObs_t o1, iABObs_t o2, iABObs_t o3) {
    return o_check_arc(O(o1), &tgt(o1, o2)->p2, O(o2), DIR(o2), &tgt(o2, o3)->p1, O(o3));
}
static inline sNum_t arc_len(iABObs_t o1, iABObs_t o2, iABObs_t o3) {
    return o_arc_len(&tgt(o1, o2)->p2, O(o2), DIR(o2), &tgt(o2, o3)->p1);
}

#endif
