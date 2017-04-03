#ifndef H_GRAPH
#define H_GRAPH

#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <string>
#include <deque>
#include <map>

namespace GraphLib
{
typedef std::size_t Node;
typedef std::pair<Node, Node> Edge;

template<typename NodeType, typename EdgeType>
class Graph
{
public:
	Graph (bool directed = false);
	Graph (const std::vector<NodeType> &nodes,
	       const std::vector<std::pair<Edge, EdgeType>> &edges,
	       bool directed = false);
	Graph (const Graph<NodeType, EdgeType> &rhs);
	Graph (Graph<NodeType, EdgeType> &&rhs);
	Graph &operator= (const Graph<NodeType, EdgeType> &rhs);
	Graph &operator= (Graph<NodeType, EdgeType> && rhs);
	~Graph ();

	NodeType GetNode (Node node) const;
	void SetNode (Node node, const NodeType &data);
	EdgeType GetEdge (Edge edge) const;
	void SetEdge (Edge edge, const EdgeType &data);

	std::size_t GetNodesNum () const;
	std::size_t GetEdgesNum () const;

	template<typename ... Args>
	Node EmplaceNode (Args... args);
	Node InsertNode (const NodeType &data);

	template<typename ... Args>
	bool EmplaceEdge (Edge edge, Args... args);
	bool InsertEdge (Edge edge, const EdgeType &data);
	bool RemoveEdge (Edge edge);
	bool SearchEdge (Edge edge) const;

	void Clear ();

	template<typename NodePrinter, typename EdgePrinter>
	void PrintGraph (NodePrinter node_printer,
	                 EdgePrinter edge_printer) const;
	template<typename NodePrinter, typename EdgePrinter>
	void PrintToDot (std::string file_name,
	                 NodePrinter node_printer,
	                 EdgePrinter edge_printer) const;
private:
	bool directed_;
	std::deque<NodeType> nodes_;
	std::size_t edges_num_;
	std::deque<std::map<Node, EdgeType>> edges_;
};

template<typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::Graph (bool directed) :
	directed_ (directed),
	edges_num_ (0)
{
}

template<typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::Graph (const std::vector<NodeType> &nodes,
                                  const std::vector<std::pair<Edge, EdgeType> > &edges,
                                  bool directed) :
	directed_ (directed),
	edges_num_ (0)
{
	for (const auto &i : nodes)
		InsertNode (i);
	for (const auto &i : edges)
		InsertEdge (i.first, i.second);
}

template<typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::Graph (const Graph<NodeType, EdgeType> &rhs) :
	directed_ (rhs.directed_),
	nodes_ (rhs.nodes_),
	edges_num_ (rhs.edges_num_),
	edges_ (rhs.edges_)
{
}

template<typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::Graph (Graph<NodeType, EdgeType> &&rhs) :
	directed_ (rhs.directed_),
	nodes_ (std::move (rhs.nodes_)),
	edges_num_ (rhs.edges_num_),
	edges_ (std::move (rhs.edges_))
{
}

template<typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType> &Graph<NodeType, EdgeType>::operator= (const Graph<NodeType, EdgeType> &rhs)
{
	if (this == &rhs)
		return *this;

	directed_ = rhs.directed_;
	nodes_ = rhs.nodes_;
	edges_num_ = rhs.edges_num_;
	edges_ = rhs.edges_;

	return *this;
}

template<typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType> &Graph<NodeType, EdgeType>::operator= (Graph<NodeType, EdgeType> && rhs)
{
	if (this == &rhs)
		return *this;

	directed_ = rhs.directed_;
	nodes_ = std::move (rhs.nodes_);
	edges_num_ = rhs.edges_num_;
	edges_ = std::move (rhs.edges_);

	return *this;
}

template<typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::~Graph()
{
}

template<typename NodeType, typename EdgeType>
NodeType Graph<NodeType, EdgeType>::GetNode (Node node) const
{
	return nodes_.at (node);
}

template<typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::SetNode (Node node, const NodeType &data)
{
	nodes_.at (node) = data;
}

template<typename NodeType, typename EdgeType>
EdgeType Graph<NodeType, EdgeType>::GetEdge (Edge edge) const
{
	return edges_.at (edge.first).at (edge.second);
}

template<typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::SetEdge (Edge edge, const EdgeType &data)
{
	edges_.at (edge.first).at (edge.second) = data;
	if (!directed_)
		edges_.at (edge.second).at (edge.first) = data;
}


template<typename NodeType, typename EdgeType>
std::size_t Graph<NodeType, EdgeType>::GetNodesNum() const
{
	return nodes_.size ();
}

template<typename NodeType, typename EdgeType>
std::size_t Graph<NodeType, EdgeType>::GetEdgesNum() const
{
	return edges_num_;
}

template<typename NodeType, typename EdgeType>
template<typename ... Args>
Node Graph<NodeType, EdgeType>::EmplaceNode (Args... args)
{
	nodes_.emplace_back (std::forward<Args> (args)...);
	edges_.resize (nodes_.size);
	return nodes_.size() - 1;
}

template<typename NodeType, typename EdgeType>
Node Graph<NodeType, EdgeType>::InsertNode (const NodeType &data)
{
	nodes_.push_back (data);
	edges_.resize (nodes_.size());
	return nodes_.size() - 1;
}

template<typename NodeType, typename EdgeType>
template<typename ... Args>
bool Graph<NodeType, EdgeType>::EmplaceEdge (Edge edge, Args... args)
{
	bool result = edges_.at (edge.first ).emplace (std::piecewise_construct,
	                                               std::forward_as_tuple (edge.second),
                                                   std::forward_as_tuple (args...)).second;
	if (!directed_)
		result &= edges_.at (edge.second).emplace (std::piecewise_construct,
		                                           std::forward_as_tuple (edge.first),
		                                           std::forward_as_tuple (args...)).second;
	if (result)
		++edges_num_;
	return result;
}

template<typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::InsertEdge (Edge edge, const EdgeType &data)
{
	bool result = edges_.at (edge.first).insert (std::make_pair (edge.second, data)).second;
	if (!directed_)
		result &= edges_.at (edge.second).insert (std::make_pair (edge.first , data)).second;
	if (result)
		++edges_num_;
	return result;
}

template<typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::RemoveEdge (Edge edge)
{
	auto iter = edges_.at (edge.first).find (edge.second);
	if (iter == edges_.at (edge.first).end())
		return false;
	else
	{
		edges_.at (edge.first).erase (iter);
		if (!directed_)
			edges_.at (edge.second).erase (edges_.at (edge.second).find (edge.first));
		--edges_num_;
		return true;
	}
}

template<typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::SearchEdge (Edge edge) const
{
	if (edges_.at (edge.first).find (edge.second) == edges_.at (edge.first).end())
		return false;
	else
		return true;
}

template<typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::Clear()
{
	nodes_.clear ();
	edges_.clear ();
	edges_num_ = 0;
}

template<typename NodeType, typename EdgeType>
template<typename NodePrinter, typename EdgePrinter>
void Graph<NodeType, EdgeType>::PrintGraph(NodePrinter node_printer,
                                           EdgePrinter edge_printer) const
{
	if (directed_)
		std::cout << "Directed graph" << std::endl;
	else
		std::cout << "Nondirectional graph" << std::endl;
	std::cout << "List of nodes:" << std::endl;
	for (Node i = 0; i < nodes_.size(); ++i)
	{
		std::cout << "Node " << i + 1 << ": ";
		node_printer (std::cout, nodes_[i]);
		std::cout << std::endl;
	}
	std::cout << "List of edges:" << std::endl;
	for (Node i = 0; i < nodes_.size(); ++i)
		for (const auto &j : edges_[i])
			if (directed_ || i < j.first)
			{
				std::cout << i + 1 << ( (directed_) ? " -> " : " -- ") << j.first + 1
				          << " : ";
  				edge_printer (std::cout, j.second);
				std::cout << std::endl;
			}
}

template<typename NodeType, typename EdgeType>
template<typename NodePrinter, typename EdgePrinter>
void Graph<NodeType, EdgeType>::PrintToDot (std::string file_name,
                                            NodePrinter node_printer,
                                            EdgePrinter edge_printer) const
{
	std::ofstream file (file_name);
	file << ( (directed_) ? "digraph" : "graph") << std::endl;
	file << "{" << std::endl;
	for (Node i = 0; i < nodes_.size(); ++i)
	{
		file << i << ' ';
		node_printer (file, nodes_[i]);
		file << std::endl;
	}
	for (Node i = 0; i < nodes_.size(); ++i)
		for (const auto &j : edges_[i])
			if (directed_ || i < j.first)
			{
				file << i << ( (directed_) ? " -> " : " -- ") << j.first << ' ';
				edge_printer (file, j.second);
	      		file << std::endl;
			}
	file << "}" << std::endl;
}
}

#endif
