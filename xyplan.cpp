/*
 * RA* for (x, y) planning.
 */

#include <cmath>
#include <fstream>
#include <future>
#include <vector>
#include <queue>
#include <limits.h>
#include <chrono>

#include "args.h"
#include "log.h"
#include "utils.h"
#include "BS_thread_pool.hpp"

#define MAX_X (2048)
#define MAX_Y (2048)
#define NUM_DIRS (8)

using namespace std;
using namespace args;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::nanoseconds;

typedef std::vector<int32_t> PATH;

int32_t mapX, mapY;
int32_t robotLength = 10, robotWidth = 4;
std::vector<Rectangle*> obstacles;
double execTime;

void readMap(std::string fileName);
bool isFeasible(int32_t x, int32_t y);
bool isFree(int32_t x, int32_t y);
std::vector<int32_t> getOuterDirs(int32_t inDir);
PATH plan(int32_t startX, int32_t startY, int32_t goalX, int32_t goalY, float heuristicWeight,\
        int32_t numThreads, bool doSpeculation, uint64_t maxExps);

int main(int argc, const char **argv) {
    using args::Parser;
    using args::KVArg;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputMapArg(parser, "map", "", "Input map file");
    KVArg<int32_t> numTestsArg(parser, "num-tests", "", "Number of tests");
    KVArg<float> heuristicWeightArg(parser, "weight", "", "Heuristic weight of A*");
    KVArg<int32_t> threadsArg(parser, "threads", "", "The maximum number of threads");
    KVArg<uint64_t> maxExpsArg(parser, "max-exps", "", "Maximum number of expansions");
    FlagArg speculationArg(parser, "speculation", "", "Enable speculative parallelism");
    KVArg<std::string> outputPathArg(parser, "output", "", "Output path file");

    if (!parser.parse()) assert(false);

    assert_msg(inputMapArg.found(), "Input map file is not provided");

    std::string inputFile = inputMapArg.value();
    int32_t numTests = numTestsArg.found() ? numTestsArg.value() : 10;
    float heuristicWeight = heuristicWeightArg.found() ? heuristicWeightArg.value() : 1.0;
    int32_t numThreads = threadsArg.found() ? threadsArg.value() : 1;
    uint64_t maxExps = maxExpsArg.found() ? maxExpsArg.value() : 10'000;
    bool doSpeculation = speculationArg.found();
    std::string outputFile = outputPathArg.found() ? outputPathArg.value() : "/dev/null";

    readMap(inputFile);

    auto generateRandomValidPoint = [&]() {
        while (true) {
            int32_t x = rand() % mapX;
            int32_t y = rand() % mapY;
            if (isFeasible(x, y) && isFree(x, y)) {
                return std::make_pair(x, y);
            }
        }
    };

    for (int i = 0; i < numTests; i++) {
        auto rS = generateRandomValidPoint();
        auto rG = generateRandomValidPoint();
        info("Planning from s:(%d, %d) to g:(%d, %d)", rS.first, rS.second, rG.first, rG.second);
        plan(rS.first, rS.second, rG.first, rG.second, heuristicWeight,\
                numThreads, doSpeculation, maxExps);
    }

    std::cout << "execTime: " << execTime / numTests << std::endl;

    return 0;
}

// Could be too large to be allocated in function stack memory
int32_t gVals[MAX_Y][MAX_X];
bool visited[MAX_Y][MAX_X];
std::shared_future<bool> specFutures[MAX_Y][MAX_X];
bool specScoreboard[MAX_Y][MAX_X];
PATH plan(int32_t startX, int32_t startY, int32_t goalX, int32_t goalY, float heuristicWeight,\
        int32_t numThreads, bool doSpeculation, uint64_t maxExps) {

    assert_msg(isFeasible(startX, startY) && isFree(startX, startY),\
            "The initial state cannot be infeasible/occupied");
    assert_msg(isFeasible(goalX, goalY) && isFree(goalX, goalY),\
            "The goal state cannot be infeasible/occupied");

    struct Node {
        int32_t x, y;
        float g, f;
        Node *parent;
        int32_t dir;

        Node(int32_t _x, int32_t _y, float _g, float _f, Node *_parent, int32_t _dir) {
            x = _x;
            y = _y;
            g = _g;
            f = _f;
            parent = _parent;
            dir = _dir;
        }
    } __attribute__((aligned)); // Minimize false sharings

    struct NodeCmp {
        bool operator()(const Node* left, const Node* right) {
            return left->f > right->f;
        }
    } __attribute__((aligned));

    typedef std::priority_queue<Node*, std::vector<Node*>, NodeCmp> MIN_HEAP;

    for (int32_t i = 0; i < MAX_Y; i++) {
        for (int32_t j = 0; j < MAX_X; j++) {
            gVals[i][j] = INT_MAX;
            visited[i][j] = false;
            specScoreboard[i][j] = false;
            specFutures[i][j] = std::shared_future<bool>();
        }
    }

    auto getHeuristic = [goalX, goalY](int32_t x, int32_t y) {
       float dist = 0;
       dist += (x - goalX) * (x - goalX);
       dist += (y - goalY) * (y - goalY);
       return sqrt(dist);
    };

    auto isGoal = [goalX, goalY](int32_t x, int32_t y) {
        return ((x == goalX) && (y == goalY));
    };

    int32_t dX[NUM_DIRS] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int32_t dY[NUM_DIRS] = {-1, 0, 1, -1, 1, -1, 0, 1};
    float movementCost[NUM_DIRS] = {};
    for (auto i = 0; i < NUM_DIRS; i++) {
        movementCost[i] = std::sqrt((dX[i] * dX[i]) + (dY[i] * dY[i]));
    }

    BS::thread_pool pool(numThreads);

    MIN_HEAP openList;
    Node *startNode = new Node(startX, startY, 0, heuristicWeight * getHeuristic(startX, startY), NULL, -1);
    openList.push(startNode);
    uint64_t numExpansions = 0;
    Node *expNode = NULL;

    auto baseTime = high_resolution_clock::now();
    while (!openList.empty()) {
        expNode = openList.top();
        openList.pop();
        int32_t expX = expNode->x;
        int32_t expY = expNode->y;

        if (unlikely(visited[expY][expX])) continue;
        visited[expY][expX] = true;
        if (++numExpansions >= maxExps) break;

        if (isGoal(expX, expY)) {
            assert(numExpansions <= maxExps);

            auto t = high_resolution_clock::now();
            execTime += duration_cast<nanoseconds>(t - baseTime).count() * 1e-9;

            PATH path;
            Node *n = expNode;
            while (n) {
                path.push_back(n->dir);
                n = n->parent;
            }

            path.pop_back();
            std::reverse(path.begin(), path.end());
            return path;
        }

        int32_t outstandingThread = 0;
        std::future<bool> vFuture[NUM_DIRS];
        for (int32_t dir = 0; dir < NUM_DIRS; dir++) {
            int32_t xx = expX + dX[dir];
            int32_t yy = expY + dY[dir];
            if (visited[yy][xx]) continue;
            if (!isFeasible(xx, yy)) continue;
            if (!specScoreboard[yy][xx]) {
                vFuture[dir] = pool.submit(isFree, xx, yy);
                outstandingThread++;
            }
        }

        if (doSpeculation && outstandingThread && (expNode->parent != NULL)) {
            int32_t precedingAct = expNode->dir;
            int32_t specX = expX;
            int32_t specY = expY;
            int32_t runahead = numThreads - outstandingThread;
            while (runahead > 0) {
                specX += dX[precedingAct];
                specY += dY[precedingAct];
                if (!isFeasible(specX, specY)) break;
                for (int32_t dir : getOuterDirs(precedingAct)) {
                    int32_t newX = specX + dX[dir];
                    int32_t newY = specY + dY[dir];
                    if (!isFeasible(newX, newY) || visited[newY][newX]) continue;
                    specFutures[newY][newX] = pool.submit(isFree, newX, newY);
                    specScoreboard[newY][newX] = true;
                    if (--runahead == 0) break;
                }
            }
        }

        for (int32_t dir = 0; dir < NUM_DIRS; dir++) {
            int32_t xx = expX + dX[dir];
            int32_t yy = expY + dY[dir];
            if (visited[yy][xx]) continue;
            if (!isFeasible(xx, yy)) continue;
            bool v;
            if (!specScoreboard[yy][xx]) {
                v = vFuture[dir].get();
            } else {
                v = specFutures[yy][xx].get();
            }
            if (!v) continue;

            float g = expNode->g + movementCost[dir];
            float f = g + heuristicWeight * getHeuristic(xx, yy);
            if (g < gVals[yy][xx]) {
                gVals[yy][xx] = g;
                openList.push(new Node(xx, yy, g, f, expNode, dir));
            }
        }
    }

    auto t = high_resolution_clock::now();
    execTime += duration_cast<nanoseconds>(t - baseTime).count() * 1e-9;
    assert(numExpansions <= maxExps);

    PATH path;
    Node *n = expNode;
    while (n) {
        path.push_back(n->dir);
        n = n->parent;
    }

    path.pop_back();
    std::reverse(path.begin(), path.end());
    return path;
}

void readMap(std::string fileName) {
    std::ifstream file(fileName);
    assert(file.good());

    std::string line;
    std::size_t pos;

    std::getline(file, line);
    pos = line.find(' ');
    assert(pos != std::string::npos);
    assert(line.substr(0, pos) == "height");
    mapY = stoi(line.substr(pos + 1));

    std::getline(file, line);
    pos = line.find(' ');
    assert(pos != std::string::npos);
    assert(line.substr(0, pos) == "width");
    mapX = stoi(line.substr(pos + 1));

    std::getline(file, line);
    assert(line == "X Y Length Width");

    int x, y, length, width;
    while (std::getline(file, line)) {
        std::stringstream iss(line);
        if (!(iss >> x >> y >> length >> width)) break;
        obstacles.push_back(new Rectangle(x, y, length, width));
    }

    file.close();
}

bool isFeasible(int32_t x, int32_t y) {
    for (int32_t i = 0; i <= robotWidth; i++) {
        for (int32_t j = 0; j <= robotLength; j++) {
            int32_t xx = x + j;
            int32_t yy = y + i;

            if ((xx < 0) || (xx >= mapX) || (yy < 0) || (yy >= mapY)) {
                return false;    // Sticks out of the map
            }
        }
    }

    return true;
}

bool isFree(int32_t x, int32_t y) {
    for (size_t i = 0; i < obstacles.size(); i++) {
        Rectangle *rect = obstacles[i];
        bool c1 = (x < rect->x + rect->l);
        bool c2 = (x + robotLength > rect->x);
        bool c3 = (y < rect->y + rect->w);
        bool c4 = (y + robotWidth > rect->y);
        if (c1 && c2 && c3 && c4) {
            return false;
        }
    }
    return true;
}

std::vector<int32_t> getOuterDirs(int32_t inDir) {
    switch (inDir) {
        case 0:
            return std::vector<int32_t>{0, 1, 2, 3, 5};
        case 1:
            return std::vector<int32_t>{0, 1, 2};
        case 2:
            return std::vector<int32_t>{0, 1, 2, 4, 7};
        case 3:
            return std::vector<int32_t>{0, 3, 5};
        case 4:
            return std::vector<int32_t>{2, 4, 7};
        case 5:
            return std::vector<int32_t>{0, 3, 5, 6, 7};
        case 6:
            return std::vector<int32_t>{5, 6, 7};
        case 7:
            return std::vector<int32_t>{2, 4, 5, 6, 7};
    }
    panic("Should not reach here");
};
