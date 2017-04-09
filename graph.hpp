#ifndef H_GRAPH
#define H_GRAPH

#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <string>
#include <queue>
#include <deque>
#include <list>
#include <map>

namespace GraphLib
{
// interface
typedef std::size_t Node;
typedef std::pair<Node, Node> Edge;

template<typename NodeType, typename EdgeType>
class Graph
{
public:
	typedef std::map<Node, typename std::list<EdgeType>::iterator> AdjacencyList;
	Graph (bool directed = false);
	Graph (const std::vector<NodeType>& nodes,
	       const std::vector<std::pair<Edge, EdgeType>>& edges,
	       bool directed = false);
	Graph (const Graph<NodeType, EdgeType>& rhs);
	Graph (Graph<NodeType, EdgeType>&& rhs);
	Graph &operator= (const Graph<NodeType, EdgeType>& rhs);
	Graph &operator= (Graph<NodeType, EdgeType>&& rhs);
	~Graph ();

	NodeType& NodeData (Node node);
	const NodeType& NodeData (Node node) const;
	EdgeType& EdgeData (Edge edge);
	const EdgeType& EdgeData (Edge edge) const;
	const AdjacencyList& IncidentEdges (Node node) const;

	std::size_t GetNodesNum () const;
	std::size_t GetEdgesNum () const;

	template<typename ... Args>
	Node EmplaceNode (Args... args);
	Node InsertNode (const NodeType& data);

	template<typename ... Args>
	bool EmplaceEdge (Edge edge, Args... args);
	bool InsertEdge (Edge edge, const EdgeType& data);
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
	std::list<EdgeType> edges_;
	std::deque<AdjacencyList> adjacency_list_;
};

enum class Color : char
{
	nocolor,
	color_1,
	color_2
};

template<typename Graph>
bool PaintGraph (Graph& graph, typename Graph::Edge& cycle);
template<typename Graph>
void PrintCycle (const Graph& graph, const typename Graph::Edge& cycle);

// implementation
template<typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::Graph (bool directed) :
	directed_ (directed)
{
}

template<typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::Graph (const std::vector<NodeType>& nodes,
                                  const std::vector<std::pair<Edge, EdgeType>>& edges,
                                  bool directed) :
	directed_ (directed)
{
	for (const auto &i : nodes)
		InsertNode (i);
	for (const auto &i : edges)
		InsertEdge (i.first, i.second);
}

template<typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::Graph (const Graph<NodeType, EdgeType>& rhs) :
	directed_ (rhs.directed_),
	nodes_ (rhs.nodes_),
	edges_ (rhs.edges_),
	adjacency_list_ (rhs.adjacency_list_)
{
}

template<typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::Graph (Graph<NodeType, EdgeType>&& rhs) :
	directed_ (rhs.directed_),
	nodes_ (std::move (rhs.nodes_)),
	edges_ (std::move (rhs.edges_)),
	adjacency_list_ (std::move (rhs.adjacency_list_))
{
}

template<typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType> &Graph<NodeType, EdgeType>::operator= (const Graph<NodeType, EdgeType>& rhs)
{
	if (this == &rhs)
		return *this;

	directed_ = rhs.directed_;
	nodes_ = rhs.nodes_;
	edges_ = rhs.edges_;
	adjacency_list_ = rhs.adjacency_list_;

	return *this;
}

template<typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType> &Graph<NodeType, EdgeType>::operator= (Graph<NodeType, EdgeType>&& rhs)
{
	if (this == &rhs)
		return *this;

	directed_ = rhs.directed_;
	nodes_ = std::move (rhs.nodes_);
	edges_ = std::move (rhs.edges_);
	adjacency_list_ = std::move (rhs.adjacency_list_);

	return *this;
}

template<typename NodeType, typename EdgeType>
Graph<NodeType, EdgeType>::~Graph()
{
}

template<typename NodeType, typename EdgeType>
NodeType& Graph<NodeType, EdgeType>::NodeData(Node node)
{
	return nodes_.at(node);
}

template<typename NodeType, typename EdgeType>
const NodeType& Graph<NodeType, EdgeType>::NodeData(Node node) const
{
	return nodes_.at(node);
}

template<typename NodeType, typename EdgeType>
EdgeType& Graph<NodeType, EdgeType>::EdgeData(Edge edge)
{
	return *adjacency_list_.at(edge.first).at(edge.second);
}

template<typename NodeType, typename EdgeType>
const EdgeType& Graph<NodeType, EdgeType>::EdgeData(Edge edge) const
{
	return *adjacency_list_.at(edge.first).at(edge.second);
}

template<typename NodeType, typename EdgeType>
const typename Graph<NodeType, EdgeType>::AdjacencyList& Graph<NodeType, EdgeType>::IncidentEdges(Node node) const
{
	return adjacency_list_.at(node);
}



template<typename NodeType, typename EdgeType>
std::size_t Graph<NodeType, EdgeType>::GetNodesNum() const
{
	return nodes_.size ();
}

template<typename NodeType, typename EdgeType>
std::size_t Graph<NodeType, EdgeType>::GetEdgesNum() const
{
	return edges_.size ();
}

template<typename NodeType, typename EdgeType>
template<typename ... Args>
Node Graph<NodeType, EdgeType>::EmplaceNode (Args... args)
{
	nodes_.emplace_back (std::forward<Args> (args)...);
	adjacency_list_.resize (nodes_.size);
	return nodes_.size() - 1;
}

template<typename NodeType, typename EdgeType>
Node Graph<NodeType, EdgeType>::InsertNode (const NodeType &data)
{
	nodes_.push_back (data);
	adjacency_list_.resize (nodes_.size());
	return nodes_.size() - 1;
}

template<typename NodeType, typename EdgeType>
template<typename ... Args>
bool Graph<NodeType, EdgeType>::EmplaceEdge (Edge edge, Args... args)
{
	bool result = adjacency_list_.at (edge.first).insert (std::make_pair (edge.second, edges_.end())).second;
	if (!directed_)
		result &= adjacency_list_.at (edge.second).insert (std::make_pair (edge.first, edges_.end())).second;
	if (result)
		edges_.emplace_back(std::forward<Args>(args)...);
	return result;
}

template<typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::InsertEdge (Edge edge, const EdgeType &data)
{
	bool result = adjacency_list_.at (edge.first).insert (std::make_pair (edge.second, edges_.end())).second;
	if (!directed_)
		result &= adjacency_list_.at (edge.second).insert (std::make_pair (edge.first, edges_.end())).second;
	if (result)
		edges_.push_back(data);
	return result;
}

template<typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::RemoveEdge (Edge edge)
{
	auto iter = adjacency_list_.at (edge.first).find (edge.second);
	if (iter == adjacency_list_.at (edge.first).end())
		return false;
	else
	{
		edges_.erase(*iter.second);
		adjacency_list_.at (edge.first).erase (iter);
		if (!directed_)
			adjacency_list_.at (edge.second).erase (adjacency_list_.at (edge.second).find (edge.first));
		return true;
	}
}

template<typename NodeType, typename EdgeType>
bool Graph<NodeType, EdgeType>::SearchEdge (Edge edge) const
{
	if (adjacency_list_.at (edge.first).find (edge.second) == adjacency_list_.at (edge.first).end())
		return false;
	else
		return true;
}

template<typename NodeType, typename EdgeType>
void Graph<NodeType, EdgeType>::Clear()
{
	nodes_.clear ();
	edges_.clear ();
	adjacency_list_.clear ();
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
		for (const auto &j : adjacency_list_[i])
			if (directed_ || i < j.first)
			{
				std::cout << i + 1 << ( (directed_) ? " -> " : " -- ") << j.first + 1
				          << " : ";
  				edge_printer (std::cout, *j.second);
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
		for (const auto &j : adjacency_list_[i])
			if (directed_ || i < j.first)
			{
				file << i << ( (directed_) ? " -> " : " -- ") << j.first << ' ';
				edge_printer (file, *j.second);
				file << std::endl;
			}
	file << "}" << std::endl;
}

namespace
{
template<typename Graph>
bool FindNocolorNode (const Graph& graph, Node& node)
{
	for (Node i = 0; i < graph.GetNodesNum(); ++i)
		if (graph.NodeData (i).color == Color::nocolor)
		{
			node = i;
			return true;
		}
			return false;
}

Color InverseColor (Color color)
{
	return (color == Color::color_1) ? Color::color_2 :
	       (color == Color::color_2) ? Color::color_1 : Color::nocolor;
}
}

template<typename Graph>
bool PaintGraph (Graph& graph, Edge& cycle)
{
	Node cur_node;
	while (FindNocolorNode (graph, cur_node))
	{
		// BFS
		std::queue<Node> cur_nodes;
		cur_nodes.push (cur_node);
		graph.NodeData (cur_node).color = Color::color_1;
		graph.NodeData (cur_node).prev = cur_node;
		while (!cur_nodes.empty())
		{
			cur_node = cur_nodes.front();
			cur_nodes.pop();
			for (const auto &edge : graph.IncidentEdges (cur_node))
			{
				Node node = edge.first;
				if (graph.NodeData (node).color == Color::nocolor)
				{
					graph.NodeData (node).color = InverseColor (graph.NodeData (cur_node).color);
					graph.NodeData (node).prev = cur_node;
					cur_nodes.push (node);
				}
				else if (graph.NodeData (node).color != InverseColor (graph.NodeData (cur_node).color))
				{
					cycle = std::make_pair (cur_node, node);
					return false;
				}
			}
		}
	}
	return true;
}

template <typename Graph>
void PrintCycle (const Graph& graph, const Edge& cycle)
{
	std::cout << "Nodes in cycle:" << std::endl;

	Node cur_node = cycle.first;
	while (graph.NodeData (cur_node).prev != cur_node)
	{
	while (graph.NodeData (cur_node).prev != cur_node)
	{
		std::cout << cur_node << ' ';
		cur_node = graph.NodeData (cur_node).prev;
	}
	std::cout << cur_node << std::endl;
}
}
}

#endif
