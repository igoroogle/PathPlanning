#include "isearch.h"

ISearch::ISearch()
{
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
}

ISearch::~ISearch(void) {}


SearchResult ISearch::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    auto start = std::chrono::system_clock::now();
    sresult.nodescreated = 0;
    sresult.numberofsteps = 0;
    /*sresult.pathfound = ;
    sresult.nodescreated =  ;
    sresult.numberofsteps = ;
    sresult.time = ;*/
    //sresult.hppath = &hppath; //Here is a constant pointer*/
    //sresult.lppath = &lppath;

    /*for (int i = 0; i < map.getMapHeight(); ++i) {
        for (int j = 0; j < map.getMapWidth(); ++j) {
            std::cout << map.getValue(i, j) << ' ';
        }
        std::cout << '\n';
    }*/

    int h = map.getMapHeight();
    int w = map.getMapWidth();
    close.assign(h, std::vector<bool>(w));
    open.assign(h, std::vector<Node*>(w));
    Node startNode(map.getStart());
    Node goalNode(map.getGoal());
    open[startNode.i][startNode.j] = new Node(startNode);
    for(;;) { //infinity circle
        Node* indPtr = open[goalNode.i][goalNode.j];
        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w; ++j) {
                ++sresult.numberofsteps;
                if ((close[i][j]) || (open[i][j] == nullptr)) {
                    continue;
                }

                if (indPtr == nullptr) {
                    indPtr = open[i][j];
                    continue;
                }


                if (indPtr->F > open[i][j]->F) {
                    indPtr = open[i][j];
                }
            }
        }

        if (indPtr == nullptr) {
            break;
        }
        close[indPtr->i][indPtr->j] = true;

        ++sresult.nodescreated;
        if (indPtr == open[goalNode.i][goalNode.j]) {
            break;
        }

        close[indPtr->i][indPtr->j] = true;

        auto listSuccessors = findSuccessors(*indPtr, map, options);
        for (auto cur : listSuccessors) {
            ++sresult.numberofsteps;
            if (close[cur.i][cur.j]) {
                continue;
            }

            if (open[cur.i][cur.j] == nullptr) {
                open[cur.i][cur.j] = new Node(cur);
                open[cur.i][cur.j]->g += open[indPtr->i][indPtr->j]->g;
                open[cur.i][cur.j]->F = open[cur.i][cur.j]->g;
                open[cur.i][cur.j]->parent = indPtr;
            } else if (open[indPtr->i][indPtr->j]->F + cur.F < open[cur.i][cur.j]->F) {
                open[cur.i][cur.j]->g = open[indPtr->i][indPtr->j]->F + cur.F;
                open[cur.i][cur.j]->F = open[cur.i][cur.j]->g;
                open[cur.i][cur.j]->parent = indPtr;
            }
        }
    }

    sresult.pathfound = close[goalNode.i][goalNode.j];
    if (sresult.pathfound)
    {
        sresult.pathlength = open[goalNode.i][goalNode.j]->F;
        makePrimaryPath(open[goalNode.i][goalNode.j]);
    } else
    {
        sresult.pathlength = 0;
    }

    auto end = std::chrono::system_clock::now();
    sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()) / 1000000000;

    if (sresult.pathfound) {
        makeSecondaryPath();
        /*for (auto cur : lppath) {
            std::cout << cur.i << ' ' << cur.j << std::endl;
        }
        std::cout << "---------\n";
        for (auto cur : hppath) {
            std::cout << cur.i << ' ' << cur.j << std::endl;
        }*/
    }

    sresult.hppath = &hppath; //Here is a constant pointer*/
    sresult.lppath = &lppath;

    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            delete(open[i][j]);
        }
    }

    return sresult;
}

std::list<Node> ISearch::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options)
{
    std::list<Node> successors;
    std::vector<int> orient = {-1, 0, 1};
    for (auto cur_i : orient) {
        for (auto cur_j : orient) {
            if ((cur_i == 0) && (cur_j == 0)) {
                continue;
            }

            if ((map.getValue(curNode.i + cur_i, curNode.j + cur_j)) != CN_GC_NOOBS) { //CN_GC_NOOBS = 0
                continue;
            }

            if ((cur_i == 0) || (cur_j == 0)) {
                successors.push_back(Node(std::make_pair(curNode.i + cur_i, curNode.j + cur_j)));
                successors.back().g = 1;
            } else {
                bool flag = false;
                if (options.allowdiagonal) {
                    if ((map.getValue(curNode.i + cur_i, curNode.j) == CN_GC_NOOBS)
                        && (map.getValue(curNode.i, curNode.j + cur_j) == CN_GC_NOOBS)) {
                        flag = true;
                    } else if (options.cutcorners) {
                        if ((map.getValue(curNode.i + cur_i, curNode.j) == CN_GC_NOOBS)
                            || (map.getValue(curNode.i, curNode.j + cur_j) == CN_GC_NOOBS)) {
                            flag = true;
                        } else if (options.allowsqueeze) {
                            flag = true;
                        }
                    }
                }

                if (flag) {
                    successors.push_back(Node(std::make_pair(curNode.i + cur_i, curNode.j + cur_j)));
                    successors.back().g = CN_SQRT_TWO;
                } else {
                    continue;
                }
            }

            successors.back().F = successors.back().g;
        }
    }

    return successors;
}


void ISearch::makePrimaryPath(Node* curNode)
{
    lppath.clear();
    auto cur = curNode;
    while (cur != nullptr) {
        lppath.push_front(*cur);
        cur = cur->parent;
    }
}

void ISearch::makeSecondaryPath()
{
    hppath.clear();
    auto cur = lppath.begin();
    hppath.push_back(*cur);
    for(;;) {
        auto firstCur = cur;
        auto secCur = ++cur;
        if (secCur == lppath.end()) {
            break;
        }
        auto thirdCur = ++cur;
        if (thirdCur == lppath.end()) {
            hppath.push_back(*secCur);
            break;
        }

        if ((thirdCur->i - secCur->i != secCur->i - firstCur->i) ||
                (thirdCur->j - secCur->j != secCur->j - firstCur->j)) {
            hppath.push_back(*secCur);
        }
        --cur;
    }
}
