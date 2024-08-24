#pragma once
#ifndef GRAPH_ALGORITHM_H_
#define GRAPH_ALGORITHM_H_ 1

#include <algorithm>
#include <stack>
#include <queue>
#include <vector>
#include <map>
#include "Graph.hh"

template <typename NodeType>
class GraphAlgorithm
{
  using EdgeType = std::pair<NodeType, NodeType>;

  protected:
    Graph<NodeType> *_graph;
    std::vector<std::vector<int>> _adjacencyMatrix;
    std::map<NodeType, int> _degreeSequence;

  public:
    GraphAlgorithm() = default;
    GraphAlgorithm(const Graph<NodeType> &graph) : _graph(&graph)
    {
      _adjacencyMatrix.resize(_graph->_v, std::vector<NodeType>(_graph->_v, 0));
      for(auto edge : _graph->_edges)
      {
        int start_node = static_cast<int>(edge.first) - 1;
        int end_node = static_cast<int>(edge.second) - 1;
        if(_graph->_isDirected) 
          _adjacencyMatrix[start_node][end_node]++;
        else
        {
          _adjacencyMatrix[start_node][end_node]++;
          _adjacencyMatrix[end_node][start_node]++;
        }
      }

      /* 填充度序列的两个方式 */
      unsigned int v_square = static_cast<unsigned int>(_graph->_v * _graph->_v);
      unsigned int e = static_cast<unsigned int>(_graph->_e);
      std::vector<int> node_seq_temp(_graph->_v, 0);

      /* 度的定义式 O(E) */
      if(v_square > e)
      {
        for(auto edge : _graph->_edges)
        {
          int start_node = static_cast<int>(edge.first) - 1;
          int end_node = static_cast<int>(edge.second) - 1;
          node_seq_temp[start_node]++; node_seq_temp[end_node]++;
        }
      }
      else  /* 邻接矩阵式 O(V^2) */
      {
        for(int i = 0; i < _graph->_v; i++)
        {
          for(int j = 0; j < _graph->_v; j++)
          {
            if(i == j) node_seq_temp[i] += 2;
            else
            {
              node_seq_temp[i]++;
              node_seq_temp[j]++;
            }
          }
        }
      }

      for(int i = 0; i < _graph->_v; i++)
        _degreeSequence.insert({i, node_seq_temp[i]});
    }

    virtual ~Graph() = default;

    /* 成员算法接口 */
    void addEdge(const NodeType &start, const NodeType &end)
    {
      auto find_sta = std::find(_graph->_vertices.begin(), 
        _graph->_vertices.end(), start);
      auto find_end = std::find(_graph->_vertices.begin(), 
        _graph->_vertices.end(), end);

      if(find_sta == _graph->_vertices.end() || find_end == _graph->_vertices.end())
      {
        throw std::runtime_error("Error:Node not found!\n");
        exit(1);
      }
      
      _graph->_edges.insert({start, end});

      if(_graph->_isDirected)
        _adjacencyMatrix[start][end]++;
      else
      {
        _adjacencyMatrix[start][end]++;
        _adjacencyMatrix[end][start]++;
      }

      _degreeSequence[start]++; _degreeSequence[end]++;
    }

    void vertexDFS(const NodeType &node)
    {
      auto find_node = std::find(_graph->_vertices.begin(), 
        _graph->_vertices.end(), node);
      
      if(find_node == _graph->_vertices.end())
      {
        throw std::runtime_error("Error:Node not found!\n");
        exit(1);
      }

      std::stack<NodeType> node_stack;
      node_stack.push(node);
      std::vector<int> visited(_graph->_v, 0);

      int idx_node = static_cast<int>(node) - 1;
      visited[idx_node] = 1;

      while(!node_stack.empty())
      {
        int idx_top = static_cast<int>(node_stack.top()) - 1;
        for(int i = 0; i < _graph->_v; i++)
        {
          if(_adjacencyMatrix[idx_top][i] != 0 && visited[i] == 0)
          {
            visited[i] = 1;
            NodeType next{idx_top};
            node_stack->push(next);
            break;
          }

          if(i == _graph->_v - 1)
          {
            for(auto node : node_stack)
              std::cout << node << " ";
            std::cout << std::endl;
            node_stack.pop();
          }
        }
      }
    }

    void vertexBFS(const NodeType &node)
    {
      auto find_node = std::find(_graph->_vertices.begin(), 
        _graph->_vertices.end(), node);
      
      if(find_node == _graph->_vertices.end())
      {
        throw std::runtime_error("Error:Node not found!\n");
        exit(1);
      }

      std::queue<NodeType> node_queue;
      node_queue.push(node);

      while(!node_queue.empty())
      {
        NodeType node_current = node_queue.front();
        node_queue.pop();

        int idx_front = static_cast<int>(node_current) - 1;
        for(int i = 0; i < _graph->_v; i++)
        {
          if(_adjacencyMatrix[idx_top][i] != 0)
          {
            NodeType node_next{i};
            node_queue.push(node_next);
          }
        }

        for(auto node : node_queue)
          std::cout << node << " ";
        std::cout << std::endl;
      }
    }

    // 无向图和有向图的多态
};

#endif