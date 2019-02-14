#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include "astar.h"

class Dijkstra : public Astar
{
    public:
        Dijkstra();
        double computeHFromCellToCell(int i1, int j1, int i2, int j2, const EnvironmentOptions &options) override;
};
#endif
