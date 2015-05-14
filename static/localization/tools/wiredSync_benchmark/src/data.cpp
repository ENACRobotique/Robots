/*
 * data.c
 *
 *  Created on: 18 avr. 2015
 *      Author: quentin
 */

/*
Real sample data captured with beacon 1, see  beacon1-2_drift_II.[csv,gnumeric] for full file.
*/

#include "data.h"

#ifdef ARCH_328P_ARDUINO
wsMeasure_t measures[187] = {
#else
wsMeasure_t measures[378] = {
#endif
    {0, 17859564},
    {1, 18858948},
    {2, 19859372},
    {3, 20858968},
    {4, 21859260},
    {5, 22859736},
    {6, 23860104},
    {7, 24859544},
    {8, 25859960},
    {9, 26860440},
    {10, 27859824},
    {11, 28860276},
    {12, 29859728},
    {13, 30860160},
    {14, 31860636},
    {15, 32861004},
    {16, 33861452},
    {17, 34860884},
    {18, 35861288},
    {19, 36860724},
    {20, 37861160},
    {21, 38860604},
    {22, 39861036},
    {23, 40860424},
    {24, 41860876},
    {25, 42861336},
    {26, 43860756},
    {27, 44861176},
    {28, 45860576},
    {29, 46861040},
    {30, 47860440},
    {31, 48860864},
    {32, 49861296},
    {33, 50860760},
    {34, 51861152},
    {35, 52860572},
    {36, 53861020},
    {37, 54860516},
    {38, 55860888},
    {39, 56860300},
    {40, 57860768},
    {41, 58861204},
    {42, 59860636},
    {43, 60861032},
    {44, 61860456},
    {45, 62860900},
    {46, 63860288},
    {47, 64860736},
    {48, 65861216},
    {49, 66860596},
    {50, 67861024},
    {51, 68860444},
    {52, 69860880},
    {53, 70860360},
    {54, 71860760},
    {55, 72860160},
    {56, 73860652},
    {57, 74861088},
    {58, 75860488},
    {59, 76860912},
    {60, 77860348},
    {61, 78860772},
    {62, 79860188},
    {63, 80860624},
    {64, 81861056},
    {65, 82860512},
    {66, 83860924},
    {67, 84860312},
    {68, 85860768},
    {69, 86860200},
    {70, 87860620},
    {71, 88860084},
    {72, 89860500},
    {73, 90860924},
    {74, 91860364},
    {75, 92860756},
    {76, 93860172},
    {77, 94860640},
    {78, 95860036},
    {79, 96860476},
    {80, 97860960},
    {81, 98860348},
    {82, 99860788},
    {83, 100860236},
    {84, 101860672},
    {85, 102860152},
    {86, 103860512},
    {87, 104859904},
    {88, 105860340},
    {89, 106860828},
    {90, 107860232},
    {91, 108860692},
    {92, 109860100},
    {93, 110860520},
    {94, 111859964},
    {95, 112860408},
    {96, 113860832},
    {97, 114860216},
    {98, 115860640},
    {99, 116860096},
    {100, 117860544},
    {101, 118859924},
    {102, 119860396},
    {103, 120859772},
    {104, 121860232},
    {105, 122860644},
    {106, 123860108},
    {107, 124860548},
    {108, 125859936},
    {109, 126860404},
    {110, 127859792},
    {111, 128860228},
    {112, 129860652},
    {113, 130860088},
    {114, 131860528},
    {115, 132859924},
    {116, 133860388},
    {117, 134859844},
    {118, 135860240},
    {119, 136859636},
    {120, 137860092},
    {121, 138860528},
    {122, 139859976},
    {123, 140860372},
    {124, 141859792},
    {125, 142860232},
    {126, 143859724},
    {127, 144860124},
    {128, 145860532},
    {129, 146859996},
    {130, 147860444},
    {131, 148859800},
    {132, 149860260},
    {133, 150859708},
    {134, 151860112},
    {135, 152859556},
    {136, 153859952},
    {137, 154860384},
    {138, 155859836},
    {139, 156860264},
    {140, 157859712},
    {141, 158860140},
    {142, 159859564},
    {143, 160859992},
    {144, 161860416},
    {145, 162859872},
    {146, 163860248},
    {147, 164859704},
    {148, 165860112},
    {149, 166859584},
    {150, 167860012},
    {151, 168859396},
    {152, 169859860},
    {153, 170860312},
    {154, 171859684},
    {155, 172860112},
    {156, 173859572},
    {157, 174859976},
    {158, 175859424},
    {159, 176859828},
    {160, 177860316},
    {161, 178859728},
    {162, 179860124},
    {163, 180859564},
    {164, 181860004},
    {165, 182859400},
    {166, 183859884},
    {167, 184859312},
    {168, 185859688},
    {169, 186860160},
    {170, 187859564},
    {171, 188860024},
    {172, 189859448},
    {173, 190859888},
    {174, 191859280},
    {175, 192859744},
    {176, 193860156},
    {177, 194859584},
    {178, 195859996},
    {179, 196859444},
    {180, 197859840},
    {181, 198859328},
    {182, 199859708},
    {183, 200859176},
    {184, 201859592},
    {185, 202860044},
#ifdef ARCH_328P_ARDUINO
    {186, 203859456}
};
#else
    {186, 203859456},
    {187, 204859912},
    {188, 205859292},
    {189, 206859732},
    {190, 207859152},
    {191, 208859592},
    {192, 209860020},
    {193, 210859436},
    {194, 211859880},
    {195, 212859328},
    {196, 213859724},
    {197, 214859260},
    {198, 215859588},
    {199, 216859004},
    {200, 217859488},
    {201, 218859912},
    {202, 219859288},
    {203, 220859732},
    {204, 221859164},
    {205, 222859604},
    {206, 223859020},
    {207, 224859492},
    {208, 225859872},
    {209, 226859356},
    {210, 227859780},
    {211, 228859188},
    {212, 229859608},
    {213, 230858996},
    {214, 231859452},
    {215, 232858976},
    {216, 233859320},
    {217, 234859780},
    {218, 235859196},
    {219, 236859600},
    {220, 237859016},
    {221, 238859460},
    {222, 239858888},
    {223, 240859300},
    {224, 241859740},
    {225, 242859212},
    {226, 243859600},
    {227, 244859028},
    {228, 245859508},
    {229, 246858912},
    {230, 247859372},
    {231, 248858764},
    {232, 249859228},
    {233, 250859636},
    {234, 251859024},
    {235, 252859516},
    {236, 253858928},
    {237, 254859344},
    {238, 255858788},
    {239, 256859192},
    {240, 257859672},
    {241, 258859092},
    {242, 259859528},
    {243, 260858956},
    {244, 261859360},
    {245, 262858804},
    {246, 263859184},
    {247, 264858596},
    {248, 265859092},
    {249, 266859528},
    {250, 267858920},
    {251, 268859356},
    {252, 269858756},
    {253, 270859252},
    {254, 271858648},
    {255, 272859084},
    {256, 273859532},
    {257, 274858948},
    {258, 275859388},
    {259, 276858780},
    {260, 277859228},
    {261, 278858708},
    {262, 279859060},
    {263, 280858544},
    {264, 281858932},
    {265, 282859380},
    {266, 283858816},
    {267, 284859232},
    {268, 285858636},
    {269, 286859060},
    {270, 287858500},
    {271, 288858920},
    {272, 289859412},
    {273, 290858804},
    {274, 291859268},
    {275, 292858676},
    {276, 293859076},
    {277, 294858496},
    {278, 295858920},
    {279, 296858348},
    {280, 297858804},
    {281, 298859276},
    {282, 299858700},
    {283, 300859084},
    {284, 301858504},
    {285, 302858948},
    {286, 303858356},
    {287, 304858836},
    {288, 305859236},
    {289, 306858696},
    {290, 307859136},
    {291, 308858548},
    {292, 309858984},
    {293, 310858408},
    {294, 311858856},
    {295, 312858260},
    {296, 313858712},
    {297, 314859120},
    {298, 315858552},
    {299, 316858968},
    {300, 317858412},
    {301, 318858844},
    {302, 319858244},
    {303, 320858688},
    {304, 321859128},
    {305, 322858540},
    {306, 323858968},
    {307, 324858396},
    {308, 325858836},
    {309, 326858268},
    {310, 327858684},
    {311, 328858096},
    {312, 329858564},
    {313, 330859000},
    {314, 331858388},
    {315, 332858852},
    {316, 333858240},
    {317, 334858684},
    {318, 335858116},
    {319, 336858548},
    {320, 337858980},
    {321, 338858388},
    {322, 339858852},
    {323, 340858248},
    {324, 341858724},
    {325, 342858124},
    {326, 343858564},
    {327, 344857980},
    {328, 345858400},
    {329, 346858860},
    {330, 347858300},
    {331, 348858752},
    {332, 349858152},
    {333, 350858572},
    {334, 351857976},
    {335, 352858428},
    {336, 353858892},
    {337, 354858276},
    {338, 355858720},
    {339, 356858172},
    {340, 357858568},
    {341, 358857984},
    {342, 359858468},
    {343, 360857832},
    {344, 361858324},
    {345, 362858764},
    {346, 363858172},
    {347, 364858612},
    {348, 365858000},
    {349, 366858456},
    {350, 367857848},
    {351, 368858336},
    {352, 369858756},
    {353, 370858172},
    {354, 371858572},
    {355, 372858020},
    {356, 373858464},
    {357, 374857900},
    {358, 375858300},
    {359, 376857700},
    {360, 377858152},
    {361, 378858628},
    {362, 379858028},
    {363, 380858432},
    {364, 381857872},
    {365, 382858316},
    {366, 383857732},
    {367, 384858184},
    {368, 385858592},
    {369, 386858012},
    {370, 387858452},
    {371, 388857908},
    {372, 389858312},
    {373, 390857736},
    {374, 391858204},
    {375, 392857632},
    {376, 393858032},
    {377, 394858452}
};
#endif
