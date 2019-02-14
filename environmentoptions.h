#ifndef ENVIRONMENTOPTIONS_H
#define ENVIRONMENTOPTIONS_H
#include "gl_const.h"

class EnvironmentOptions
{
public:
    EnvironmentOptions(bool AS, bool AD, bool CC, int MT = CN_SP_MT_EUCL); // CN_SP_MT_EUCL = 2(euclid), CN_SP_MT_EUCL = 2(euclid)
    //#define CN_SP_MT_DIAG   0
    //#define CN_SP_MT_MANH   1
    //#define CN_SP_MT_EUCL   2
    //#define CN_SP_MT_CHEB   3
    EnvironmentOptions();
    int     metrictype;     //Can be chosen Euclidean, Manhattan, Chebyshev and Diagonal distance
    bool    allowsqueeze;   //Option that allows to move throught "bottleneck"
    bool    allowdiagonal;  //Option that allows to make diagonal moves
    bool    cutcorners;     //Option that allows to make diagonal moves, when one adjacent cell is untraversable

};

#endif // ENVIRONMENTOPTIONS_H
