#pragma once
#ifndef GRAPH_ALGORITHM_H_
#define GRAPH_ALGORITHM_H_ 1

#include <algorithm>
#include <vector>
#include <map>
#include "Graph.hh"

template <typename NodeType>
class GraphAlgorithm
{
  protected:
    Graph<NodeType> *_graph;
    std::vector<std::vector<int>> _adjacencyMatrix;
    std::map<NodeType, int> _degreeSequence;


  public:
    GraphAlgorithm() = default;
    GraphAlgorithm(const Graph<NodeType> &graph) : _graph(&graph)
    {
        _adjacencyMatrix.resize(V, std::vector<NodeType>(V, 0));
        // change the graph edges and nodes into matrix

    }



};

#endif