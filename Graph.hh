#pragma once
#ifndef GRAPH_H_
#define GRAPH_H_ 1

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <string>
#include <exception>
#include <memory>
#include <iostream>
#include <sstream>
#include <fstream>

class Debugger {
  public:
    bool debug;

    Debugger() {
        char ch;
        std::cout << "Open debug mode?(Y/N):" << std::endl; 
        std::cin >> ch;
        
        while(true) {
            bool keep = false;
            switch(ch) {
                case 'Y' : debug = true; break;
                case 'y' : debug = true; break;
                case 'N' : debug = false; break;
                case 'n' : debug = false; break;
                default : {
                    std::cerr << "What did you enter on your keyboard? Try again!"
                        << std::endl;
                    std::cin >> ch;
                    keep = true;
                }
            }
            
            if(!keep) break;
        }
    }

    ~Debugger() = default;
    void divide_line()
    {
        if(debug)
            std::cout << "-----------------------" << std::endl;
    }
}; 

extern Debugger debugger;

template <typename NodeType>
class Graph
{
  protected:
    int _v;
    int _e;
    std::unordered_set<NodeType> _vertices;
    std::unordered_multimap<NodeType, NodeType> _edges;
    bool _isDirected;

    void copyFromGraph(const int &v, const int &e, const std::unordered_set<NodeType> &vertices, 
        const std::unordered_multimap<NodeType, NodeType> &edges)
    {
        _isDirected = false;
        _v = v;
        _e = e;
        _vertices = vertices;
        _edges = edges;
    }

    void readFromFile(const std::string &filename)
    {
        std::ifstream input_file(filename);

        if(!input_file.is_open())
        {
            throw std::runtime_error("Error:Input file could not be opened!\n");
            exit(1);
        }

        std::string line;
        while(std::getline(input_file,line))
        {
            char ch = line[0];
            if(ch != '#' && ch != '%') break;
        }

        std::istringstream iss_numV(line);
        int numV = 0; int guardNum;
        if((iss_numV >> numV) && !(iss_numV >> guardNum)) _v = numV;
        else throw std::runtime_error
        ("Error:Syntax error of inputfile:Not correct type of numV!\n");

        while(std::getline(input_file,line))
        {
            std::istringstream iss_node(line);
            int nd_1, nd_2;
            if(iss_node >> nd_1 >> nd_2)
            {
                NodeType node_1{nd_1}, node_2{nd_2};
                _vertices.insert(node_1); _vertices.insert(node_2);
                _edges.insert(std::make_pair(node_1,node_2));
            }
            else throw std::runtime_error
            ("Error:Syntax error of inputfile:Not correct type of nodes!\n");
        }

        _e = _edges.size();
        _isDirected = false;

        input_file.close();
    }

  public:
    Graph() = default;
    Graph(const int &v, const int &e, const std::unordered_set<NodeType> &vertices, 
        const std::unordered_multimap<NodeType, NodeType> &edges) : 
        _v(v), _e(e), _vertices(vertices), _edges(edges), _isDirected(false) {}
    Graph(const std::string &filename) { readFromFile(filename); }
    Graph(const Graph<NodeType> &graph) { copyFromGraph(graph._v, graph._e, graph._vertices, graph._edges); }
    Graph(Graph<NodeType> &&graph) 
    { 
        _isDirected = std::move(graph._isDirected);
        _v = std::move(static_cast<int>(graph._v));
        _e = std::move(static_cast<int>(graph._e));
        _vertices = std::move(graph._vertices);
        _edges = std::move(graph._edges);
        _isDirected = false;
    }

    virtual ~Graph() = default;

    Graph<NodeType>& operator=(const Graph<NodeType> &graph) 
    { 
        copyFromGraph(graph._v, graph._e, graph._vertices, graph._edges); 
        return *this;
    }
    Graph<NodeType>& operator=(Graph<NodeType> &&graph) 
    { 
        _isDirected = std::move(graph._isDirected);
        _v = std::move(static_cast<int>(graph._v));
        _e = std::move(static_cast<int>(graph._e));
        _vertices = std::move(graph._vertices);
        _edges = std::move(graph._edges);

        _isDirected = false;
        return *this;
    }

    void display(const std::string &filename) const
    {
        std::ofstream output_file(filename);

        if(!output_file.is_open())
        {
            throw std::runtime_error("Error:Output file could not be opened!\n");
            exit(1);
        }

        if(!_isDirected) output_file << "graph g {\n";
        else output_file << "digraph dg {\n";

        if(!_isDirected)
        {
            for(const auto& edge : _edges)
            {
                output_file << edge.first << " -- "
                    << edge.second << ";" << "\n";
            }
        }
        else
        {
            for(const auto& edge : _edges)
            {
                output_file << edge.first << " -> "
                    << edge.second << ";" << "\n";
            }
        }
        output_file << "}\n";

        output_file.close();
    }

    const int getV() const { return _v; }
    const int getE() const { return _e; }
};


template <typename NodeType>
class DiGraph : public Graph<NodeType>
{
  public:
    DiGraph() = default;
    DiGraph(const int &v, const int &e, const std::unordered_set<NodeType> &vertices, 
        const std::unordered_multimap<NodeType, NodeType> &edges) : 
        Graph<NodeType>(v, e, vertices, edges) { Graph<NodeType>::_isDirected = true; }
    DiGraph(const std::string &filename) : Graph<NodeType>(filename) { Graph<NodeType>::_isDirected = true; }
    DiGraph(const Graph<NodeType> &graph) : Graph<NodeType>(graph) { Graph<NodeType>::_isDirected = true; }
    DiGraph(Graph<NodeType> &&graph) : Graph<NodeType>(graph) { Graph<NodeType>::_isDirected = true; }

    DiGraph<NodeType>& operator=(const Graph<NodeType> &graph) 
    { 
        Graph<NodeType>::copyFromGraph(graph._v, graph._e, graph._vertices, graph._vertices); 
        Graph<NodeType>::_isDirected = true;
        return *this;
    }
    DiGraph<NodeType>& operator=(Graph<NodeType> &&graph) 
    { 
        Graph<NodeType>::_v = std::move(static_cast<int>(graph._v));
        Graph<NodeType>::_e = std::move(static_cast<int>(graph._e));
        Graph<NodeType>::_vertices = std::move(graph._vertices);
        Graph<NodeType>::_edges = std::move(graph._edges);
        Graph<NodeType>::_isDirected = std::move(graph._isDirected);

        Graph<NodeType>::_isDirected = true;
        return *this;
    }
};

#endif