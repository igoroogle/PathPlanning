#include "astar.h"

Astar::Astar(double HW, bool BT)
{
    hweight = HW;
    breakingties = BT;
}

double Astar::computeHFromCellToCell(int i1, int j1, int i2, int j2, const EnvironmentOptions &options)
{
    //need to implement
    //#define CN_SP_MT_DIAG   0
    //#define CN_SP_MT_MANH   1
    //#define CN_SP_MT_EUCL   2
    //#define CN_SP_MT_CHEB   3
    int dx = abs(i1 - i2);
    int dy = abs(j1 - j2);
    if (options.metrictype == CN_SP_MT_DIAG) {
        return (dx + dy) + (CN_SQRT_TWO - 2) * std::min(dx, dy);
    } else if (options.metrictype == CN_SP_MT_MANH) {
        return dx + dy;
    } else if (options.metrictype == CN_SP_MT_EUCL) {
        return std::hypot(dx, dy);
    } else if (options.metrictype == CN_SP_MT_CHEB) {
        return (dx + dy) - std::min(dx, dy);
    }
    return 0;
}
