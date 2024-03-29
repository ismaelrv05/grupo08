#include "graph.h"
#include <ranges>

Graph::Graph()
{
    nodes.push_back(0);
}

int Graph::addNode()
{
    nodes.push_back(nodes.size());
    return nodes.size();
}

int Graph::addEdge(int n1, int n2)
{
    qInfo() << "n1: " << n1 << "n2: " << n2;
    if( std::ranges::find(nodes, n1) != nodes.end() and
        std::ranges::find(nodes, n2) != nodes.end() and
        std::ranges::find(edges, std::make_pair(n1, n2)) == edges.end()) {
        edges.emplace_back(n1, n2);
        return 1;
    }
    else return -1;
}
void Graph::print()
{
    for (const auto &n : nodes)
    {
        std::cout<< n << " " ;
    }
    std::cout<<std::endl;
    for (const auto &e : edges)
    {
        std::cout<< e.first << " "  << e.second;
    }
    std::cout<<std::endl;
}

int Graph::num_nodes() const{
    return nodes.size();
}