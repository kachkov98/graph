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
		Graph (const std::vector<NodeType>& nodes,
		       const std::vector<std::pair<Edge, EdgeType>>& edges,
		       bool directed = false);
		Graph (const Graph<NodeType, EdgeType>& rhs);
		Graph (Graph<NodeType, EdgeType>&& rhs);
		Graph& operator= (const Graph<NodeType, EdgeType>& rhs);
		Graph& operator= (Graph<NodeType, EdgeType>&& rhs);
		~Graph ();

		NodeType& NodeData (Node node);
		EdgeType& EdgeData (Edge edge);

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

		void PrintGraph () const;
		void PrintToDot (std::string file_name) const;
	private:
		bool directed_;
		std::deque<NodeType> nodes_;
		std::size_t edges_num_;
		std::deque<std::map<Node, EdgeType>> edges_;
	};

	Edge OrderedEdge (Edge edge)
	{
		return (edge.first <= edge.second) ? edge : Edge(edge.second, edge.first);
	}

	template<typename NodeType, typename EdgeType>
	Graph<NodeType, EdgeType>::Graph(bool directed):
		directed_(directed),
		edges_num_(0)
	{
	}

	template<typename NodeType, typename EdgeType>
	Graph<NodeType, EdgeType>::Graph(const std::vector<NodeType>& nodes,
	                                 const std::vector<std::pair<Edge, EdgeType> >& edges,
	                                 bool directed):
		directed_(directed),
		edges_num_(0)
	{
		for (const auto& i: nodes)
			InsertNode (i);
		for (const auto& i: edges)
			InsertEdge (i.first, i.second);
	}

	template<typename NodeType, typename EdgeType>
	Graph<NodeType, EdgeType>::Graph(const Graph<NodeType, EdgeType>& rhs):
		directed_(rhs.directed_),
		nodes_(rhs.nodes_),
		edges_num_(rhs.edges_num_),
		edges_(rhs.edges_)
	{
	}

	template<typename NodeType, typename EdgeType>
	Graph<NodeType, EdgeType>::Graph(Graph<NodeType, EdgeType>&& rhs):
		directed_(rhs.directed_),
		nodes_(std::move(rhs.nodes_)),
		edges_num_(rhs.edges_num_),
		edges_(std::move(rhs.edges_))
	{
	}

	template<typename NodeType, typename EdgeType>
	Graph<NodeType, EdgeType>& Graph<NodeType, EdgeType>::operator=(const Graph<NodeType, EdgeType>& rhs)
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
	Graph<NodeType, EdgeType>& Graph<NodeType, EdgeType>::operator=(Graph<NodeType, EdgeType>&& rhs)
	{
		if (this == &rhs)
			return *this;

		directed_ = rhs.directed_;
		nodes_ = std::move(rhs.nodes_);
		edges_num_ = rhs.edges_num_;
		edges_ = std::move(rhs.edges_);

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
	EdgeType& Graph<NodeType, EdgeType>::EdgeData(Edge edge)
	{
		if (directed_)
			edge = OrderedEdge(edge);
		return edges_.at(edge.first).at(edge.second);
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
	Node Graph<NodeType, EdgeType>::EmplaceNode(Args... args)
	{
		nodes_.emplace_back(std::forward<Args>(args)...);
		edges_.resize(nodes_.size);
		return nodes_.size() - 1;
	}

	template<typename NodeType, typename EdgeType>
	Node Graph<NodeType, EdgeType>::InsertNode(const NodeType& data)
	{
		nodes_.push_back(data);
		edges_.resize(nodes_.size());
		return nodes_.size() - 1;
	}

	template<typename NodeType, typename EdgeType>
	template<typename ... Args>
	bool Graph<NodeType, EdgeType>::EmplaceEdge(Edge edge, Args... args)
	{
		if (!directed_)
			edge = OrderedEdge(edge);
		bool result = edges_.at(edge.first).emplace(std::piecewise_construct,
		                                            std::forward_as_tuple(edge.second),
		                                            std::forward_as_tuple(args...)).second;
		if (result)
			++edges_num_;
		return result;
	}

	template<typename NodeType, typename EdgeType>
	bool Graph<NodeType, EdgeType>::InsertEdge(Edge edge, const EdgeType& data)
	{
		if (!directed_)
			edge = OrderedEdge(edge);
		bool result =  edges_.at(edge.first).insert(std::pair<Node, EdgeType>(edge.second, data)).second;
		if (result)
			++edges_num_;
		return result;
	}

	template<typename NodeType, typename EdgeType>
	bool Graph<NodeType, EdgeType>::RemoveEdge(Edge edge)
	{
		if (!directed_)
			edge = OrderedEdge(edge);
		auto iter = edges_.at(edge.first).find(edge.second);
		if (iter == edges_.at(edge.first).end())
			return false;
		else
		{
			edges_.at(edge.first).erase(iter);
			--edges_num_;
			return true;
		}
	}

	template<typename NodeType, typename EdgeType>
	bool Graph<NodeType, EdgeType>::SearchEdge(Edge edge) const
	{
		if (!directed_)
			edge = OrderedEdge(edge);
		if (edges_.at(edge.first).find(edge.second) == edges_.at(edge.first).end())
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
	void Graph<NodeType, EdgeType>::PrintGraph() const
	{
		if (directed_)
			std::cout << "Directed graph" << std::endl;
		else
			std::cout << "Nondirectional graph" << std::endl;
		std::cout << "List of nodes:" << std::endl;
		for (Node i = 0; i < nodes_.size(); ++i)
			std::cout << "Node " << i+1 << ": " << nodes_[i] << std::endl;
		std::cout << "List of edges:" << std::endl;
		for (Node i = 0; i < nodes_.size(); ++i)
			for (const auto& j: edges_[i])
				std::cout << i+1 << ((directed_) ? " -> " : " -- ") << j.first+1
				          << " : " << j.second << std::endl;
	}

	template<typename NodeType, typename EdgeType>
	void Graph<NodeType, EdgeType>::PrintToDot(std::string file_name) const
	{
		std::ofstream file(file_name);
		file << ((directed_) ? "digraph" : "graph") << std::endl;
		file << "{" << std::endl;
		for (Node i = 0; i < nodes_.size(); ++i)
			file << i << " [label=\"" << nodes_[i] << "\"]" << std::endl;
		for (Node i = 0; i < nodes_.size(); ++i)
			for (const auto& j: edges_[i])
				file << i << ((directed_) ? " -> " : " -- ") << j.first
				     << " [label=\"" << j.second << "\"]" << std::endl;
		file << "}" << std::endl;
	}
}

#endif
