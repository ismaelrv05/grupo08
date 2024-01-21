#ifndef CHOCACHOCA_GRAPH_H
#define CHOCACHOCA_GRAPH_H

#include <vector>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>

class Graph
{
    using Nodes = std::vector<int>;
    using Edges = std::vector<std::pair<int,int>>;
    Nodes nodes;
    Edges edges;


public:
    Graph();
    int addNode();
    int addEdge(int n1, int n2);
    int num_nodes() const;
    void print();
};


#endif //CHOCACHOCA_GRAPH_H