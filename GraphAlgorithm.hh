#pragma once
#ifndef GRAPH_ALGORITHM_H_
#define GRAPH_ALGORITHM_H_ 1

#include <algorithm>
#include <stack>
#include <queue>
#include <vector>
#include <map>
#include <cstdlib>
#include "Graph.hh"
#include "GraphTools.hh"

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

      updateDegreeSequence();
    }

    virtual ~Graph() = default;

    /* 成员算法接口 */
    void updateDegreeSequence()
    {
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
            NodeType start{i}; NodeType end{j};
            EdgeType edge{start, end};
            auto find_edge = std::find(_graph->_edges.begin(), 
              _graph->_edges.end(), edge);

            if(find_edge != _graph->_edges.end())
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
      }

      for(int i = 0; i < _graph->_v; i++)
        _degreeSequence.insert({i, node_seq_temp[i]});
    }

    void addEdge(const NodeType &start, const NodeType &end)
    {
      auto find_sta = _graph->_vertices.find(start);
      auto find_end = _graph->_vertices.find(end);

      if(find_sta == _graph->_vertices.end() || find_end == _graph->_vertices.end())
      {
        throw std::runtime_error("Error:Node not found!\n");
        return;
      }
      
      _graph->_edges.insert({start, end});

      int start_node = static_cast<int>(edge.first) - 1;
      int end_node = static_cast<int>(edge.second) - 1;

      if(_graph->_isDirected)
        _adjacencyMatrix[start_node][end_node]++;
      else
      {
        _adjacencyMatrix[start_node][end_node]++;
        _adjacencyMatrix[end_node][start_node]++;
      }

      _degreeSequence[start_node]++; _degreeSequence[end_node]++;
    }

    void vertexDFS(const NodeType &node)
    {
      auto find_node = _graph->_vertices.find(node);
      
      if(find_node == _graph->_vertices.end())
      {
        throw std::runtime_error("Error:Node not found!\n");
        return;
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
      auto find_node = _graph->_vertices.find(node);
      
      if(find_node == _graph->_vertices.end())
      {
        throw std::runtime_error("Error:Node not found!\n");
        return;
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

    EdgeType removeEdge(const NodeType &start, const NodeType &end)
    {
      EdgeType edge{start, end};
      auto find_edge = _graph->_edges.find(edge);
      if(find_edge == _graph->_edges.end())
      {
        throw std::runtime_error("Error:Edge not found!\n");
        return;
      }

      _graph->_edges.erase(find_edge);
      return edge;
    }

    void deduplicateEdges()
    {
      for(int i = 0; i < _graph->_v; i++)
      {
        for(int j = 0; j < _graph->_v; j++)
        {
          if(_adjacencyMatrix[i][j] != 0)
            _adjacencyMatrix[i][j] = 1;
        }
      }

      if(!_graph->_isDirected)
      {
        std::unordered_set<EdgeType> edges;
        for(auto edge : _graph->_edges)
          edges.insert(edge);

        for(int i = 1; i <= _graph->_v; i++)
        {
          for(int j = 1; j <= _graph->_v; j++)
          {
            NodeType start{i}; NodeType end{j};
            EdgeType e_plus{start, end}; EdgeType e_minus{end, start};
            auto find_plus = _graph->_edges.find(e_plus);
            auto find_minus = _graph->_edges.find(e_minus);

            if(find_plus != edges.end() && find_minus != edges.end())
              edges.erase(find_minus);
          }
        }

        _graph->_edges.clear();

        for(auto edge : edges)
          _graph->_edges.insert(edge);
      }
      else
      {
        std::unordered_set<EdgeType> edges;
        for(auto edge : _graph->_edges)
          edges.insert(edge);
        
        _graph->_edges.clear();

        for(auto edge : edges)
          _graph->_edges.insert(edge);
      }

      _degreeSequence.clear();
      updateDegreeSequence();
    }

    void removeLoops()
    {
      for(int i = 0; i < _graph->_v; i++)
      {
        NodeType node{i + 1};
        EdgeType loop{node, node};
        auto find_loop = _graph->_edges.find(loop);
        if(find_loop != _graph->_edges.end())
        {
          size_t cnt = count_key_value_pairs(_graph->_edges, node, node);
          _adjacencyMatrix[i][i] = 0;
          _graph->_edges.erase(find_loop);
          _degreeSequence[loop] -= static_cast<int>(cnt) * 2;
        }
      }
    }

    bool simpleCheck() const
    {
      bool duplicatedEdge = false, loop = false;
      for(int i = 0; i < _graph->_v; i++)
      {
        for(int j = 0; j < _graph->_v; j++)
        {
          if(i == j && _adjacencyMatrix[i][j] != 0)
          {
            loop = true;
            break;
          }

          if(_adjacencyMatrix[i][j] > 1)
          {
            duplicatedEdge = true;
            break;
          }
        }

        if(deduplicateEdges || loop)
          break;
      }

      return (!loop) && (!deduplicateEdges);
    }

    bool completeCheck() const
    {
      if(!_graph->_isDirected)
        return false;

      bool complete = false;
      for(int i = 0; i < _graph->_v; i++)
      {
        for(int j = 0; j < _graph->_v; j++)
        {
          if(_adjacencyMatrix[i][j] != 1)
          {
            complete = false;
            break;
          }
        }

        if(!complete)
          break;
      }
      return complete;
    }

    bool simplify()
    {
      deduplicateEdges();
      removeLoops();
      return simpleCheck();
    }

    double density() const
    {
      double dense = (1.0 * (_graph->_e)) / ((_graph->_v) * (_graph->_v));
      if(_graph->_isDirected) 
        return dense;
      return 2.0 * dense;
    }

};

#endif