#ifndef GRAPH_TOOLS_
#define GRAPH_TOOLS_ 1

#include "Graph.hh"
#include <map>
#include <cstddef>
#include <limits>

template <typename DataType = int>
class CommonNodeType
{
  protected:
    int _code;
    DataType _data;

    void copyNode(const int &code, const DataType &data)
    {
        _code = code;
        _data = data;
    }
  public:
    CommonNodeType() = default;
    CommonNodeType(const int &code, const DataType &data) : 
        _code(code), _data(data) {}
    CommonNodeType(const CommonNodeType<DataType> &node) { copyNode(node._code, node._data); }
    CommonNodeType(CommonNodeType<DataType> &&node)
    {
        _code = std::move(static_cast<int>(node._code));
        _data = std::move(node._data);
    }

    ~CommonNodeType() = default;

    CommonNodeType<DataType>& operator=(const CommonNodeType<DataType> &node)
    {
        copyNode(node._code, node._data);
        return *this;
    }

    CommonNodeType<DataType>& operator=(CommonNodeType<DataType> &&node)
    {
        _code = std::move(static_cast<int>(node._code));
        _data = std::move(node._data);
        return *this;
    }

    operator int() const { return _code; }
    int getCode() const { return _code; }
    DataType getData() const { return _data; }
    bool operator==(const CommonNodeType &node) const {
        return (_code == node.getCode()) && (_data == node.getData());
    }
};

template <typename NodeType>
class CommonCompare
{
  public:
    bool operator()(const CommonNodeType<DataType> &a,
        const CommonNodeType<DataType> &b) const {
            return (a.getCode() > b.getCode());
        }
};

template<typename Key, typename Value>  
size_t count_key_value_pairs(const std::unordered_multimap<Key, Value> &umap, 
    const Key &key, const Value &value) 
{  
    size_t count = 0;  
    for (const auto &pair : umap.equal_range(key)) 
    {  
        if (pair.second == value)  
            ++count;   
    }  
    return count;  
}  

template <typename Key>
int min_value_pair(const std::map<Key, int> &map)
{
    int min = std::numeric_limits<int>::max();
    for(auto it : map)
    {
        if(it.second < min)
            min = it.second;
    }
    return min;
}

template <typename Key>
int max_value_pair(const std::map<Key, int> &map)
{
    int max = std::numeric_limits<int>::min();
    for(auto it : map)
    {
        if(it.second > max)
            max = it.second;
    }
    return max;
}



#endif