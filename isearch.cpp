#include "isearch.h"

ISearch::ISearch()
{
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
}

ISearch::~ISearch(void) {}


SearchResult ISearch::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    //need to implement
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

    std::vector<std::vector<bool>> use;
    std::vector<std::vector<Node*>> d;
    int h = map.getMapHeight();
    int w = map.getMapWidth();
    use.assign(h, std::vector<bool>(w));
    d.assign(h, std::vector<Node*>(w));
    Node startNode(map.getStart());
    Node goalNode(map.getGoal());
    d[startNode.i][startNode.j] = new Node(startNode);
    for(;;) { //infinity circle
        Node* indPtr = d[goalNode.i][goalNode.j];
        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w; ++j) {
                ++sresult.numberofsteps;
                if ((use[i][j]) || (d[i][j] == nullptr)) {
                    continue;
                }

                if (indPtr == nullptr) {
                    indPtr = d[i][j];
                    continue;
                }


                if (indPtr->F > d[i][j]->F) {
                    indPtr = d[i][j];
                }
            }
        }

        if (indPtr == nullptr) {
            break;
        }
        use[indPtr->i][indPtr->j] = true;

        ++sresult.nodescreated;
        if (indPtr == d[goalNode.i][goalNode.j]) {
            break;
        }

        use[indPtr->i][indPtr->j] = true;

        auto listSuccessors = findSuccessors(*indPtr, map, options);
        for (auto cur : listSuccessors) {
            ++sresult.numberofsteps;
            if (use[cur.i][cur.j]) {
                continue;
            }

            if (d[cur.i][cur.j] == nullptr) {
                d[cur.i][cur.j] = new Node(cur);
                d[cur.i][cur.j]->g += d[indPtr->i][indPtr->j]->g;
                d[cur.i][cur.j]->F = d[cur.i][cur.j]->g;
                d[cur.i][cur.j]->parent = indPtr;
            } else if (d[indPtr->i][indPtr->j]->F + cur.F < d[cur.i][cur.j]->F) {
                d[cur.i][cur.j]->g = d[indPtr->i][indPtr->j]->F + cur.F;
                d[cur.i][cur.j]->F = d[cur.i][cur.j]->g;
                d[cur.i][cur.j]->parent = indPtr;
            }
        }
    }

    auto end = std::chrono::system_clock::now();
    sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()) / 1000000000;
    sresult.pathfound = use[goalNode.i][goalNode.j];
    hppath.clear();
    lppath.clear();
    if (sresult.pathfound)
    {
        sresult.pathlength = d[goalNode.i][goalNode.j]->F;
        lppath.push_back(*d[goalNode.i][goalNode.j]);
        auto cur = &lppath.back();
        while (cur->parent != nullptr) {
            lppath.push_back(*(cur->parent));

            cur->parent = &lppath.back();
            cur = &lppath.back();
        }
        reverse(lppath.begin(), lppath.end());
        hppath = lppath;
        /*auto cur = hppath.begin();
        auto nextCur = cur;
        ++cur;
        while (nextCur != hppath.end()) {
            nextCur->parent = &(*cur);
            nextNextCur = nextCur;
            ++nextNextCur;
            if (nextNextCur != hppath.end()) {
                auto vec = (nextCur->i - cur->i) * (nextNextCur->j - cur->j)
                    - (nextNextCur->i - cur->i) * (nextCur->j - cur->j);
                if (vec == 0) {
                    hppath.erase(nextCur);
                }
            }
            ++cur;
            nextCur = cur;
            ++nextCur;
        }*/
    } else
    {
        sresult.pathlength = 0;
    }




    sresult.hppath = &hppath; //Here is a constant pointer*/
    sresult.lppath = &lppath;

    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            delete(d[i][j]);
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

            if ((map.getValue(curNode.i + cur_i, curNode.j + cur_j)) != 0) {
                continue;
            }

            successors.push_back(Node(std::make_pair(curNode.i + cur_i, curNode.j + cur_j)));
            //successors.back.parent = *curNode
            if ((cur_i == 0) || (cur_j == 0)) {
                successors.back().g = 1;
            } else {
                successors.back().g = CN_SQRT_TWO;
            }
            successors.back().F = successors.back().g;
        }
    }

    return successors;
}

/*void ISearch::makePrimaryPath(Node curNode)
{
    //need to implement

}*/

/*void ISearch::makeSecondaryPath()
{
    //need to implement
}*/
