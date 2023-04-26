
#include <cmath>
#include <iostream>
#include <vector>
using std::vector;
#include <algorithm>
#include <array>
#include <string>

#include "eyebot++.h"

#define WORLD_SIZE 1000
#define STARTPOS_X 100
#define STARTPOS_Y 100

#define SPEED 300

class Node
{
public:
    int id; // node index
    int x;  // x loc
    int y;  // y loc
    vector<int> neighbors;

    Node(int id_n, int x_n, int y_n, vector<int> neighbors_n)
    {
        id = id_n;
        x = x_n;
        y = y_n;
        neighbors = neighbors_n;
    }

    void addNeighbor(Node n, int neighbor)
    {
        n.neighbors.push_back(neighbor);
    }
};

class Edge
{
public:
    std::array<int, 2> neighbors;
    int distance;
    Edge(std::array<int, 2> neighbors_e, int distance_e)
    {
        neighbors = neighbors_e;
        distance = distance_e;
    }
};

void readWaypoints(vector<Node> nodes)
{
    FILE *file = fopen("way.txt", "r");
    int i = 0;
    int x, y;
    // declare neighbor nodes?
    while (fscanf(file, "%d %d ", &x, &y) > 2) // how to read more than 2 entries per row?
    {
        printf("x:%d y:%d \n", x, y);
        vector<int> neighbors;
        // push_back neighbor IDs in vector
        Node currentNode{i, x, y, neighbors};
        nodes.push_back(currentNode);
        i++;
    }
    fclose(file);
}

void driveToPoints(vector<Node> paths)
{
    // To-Do
}

int main()
{
    LCDMenu("Start", "A*", "Drive", "Exit");
    VWSetPosition(STARTPOS_X, STARTPOS_Y, 0); // eventually change x and y
    int endSim = 0;
    vector<Node> nodes;

    do
    {
        switch (KEYRead())
        {
        case KEY1:
            printf("Reading Nodes ...\n");
            
            readWaypoints(nodes);
            break;
        case KEY2:
            printf("\nCalculating shortest Path with A*\n");
            // A*
            break;
        case KEY3:
            printf("\nDriving to goal along shortest path\n");
            // driveToPoints(paths);
            break;
        case KEY4:
            endSim = 1;
            break;
        }
    } while (!endSim);
}