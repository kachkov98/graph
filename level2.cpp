#include <iostream>
#include <random>
#include "graph.hpp"

struct NodeData
{
	GraphLib::Node prev;
	GraphLib::Color color;
};

typedef GraphLib::Graph<NodeData, char> Graph;
typedef std::uniform_int_distribution<GraphLib::Node> random_node;

Graph GenerateCompleteGraph (std::size_t nodes_num, bool directed)
{
	Graph graph (directed);

	for (std::size_t i = 0; i < nodes_num; ++i)
		graph.InsertNode (NodeData());

	for (GraphLib::Node i = 0; i < nodes_num - 1; ++i)
		for (GraphLib::Node j = i + 1; j < nodes_num; ++j)
			graph.InsertEdge (GraphLib::Edge (i, j), 0);

	return graph;
}

Graph GeneratePathGraph (std::size_t nodes_num, bool directed)
{
	Graph graph (directed);

	for (std::size_t i = 0; i < nodes_num; ++i)
		graph.InsertNode (NodeData());

	for (GraphLib::Node i = 0; i < nodes_num - 1; ++i)
		graph.InsertEdge (GraphLib::Edge (i, i + 1), 0);

	return graph;
}

Graph GenerateCycleGraph (std::size_t nodes_num, bool directed)
{
	Graph graph (directed);

	for (std::size_t i = 0; i < nodes_num; ++i)
		graph.InsertNode (NodeData());

	for (GraphLib::Node i = 0; i < nodes_num; ++i)
		graph.InsertEdge (GraphLib::Edge (i, (i + 1) % nodes_num), 0);

	return graph;
}

Graph GenerateBipartiteGraph (std::size_t nodes_num, bool directed)
{
	Graph graph (directed);

	for (std::size_t i = 0; i < nodes_num; ++i)
		graph.InsertNode (NodeData());

	std::random_device generator;
	for (GraphLib::Node node1 = 0; node1 < nodes_num / 2; ++node1)
	{
		std::size_t n = std::uniform_int_distribution<std::size_t> (1, nodes_num / 2) (generator);
		for (std::size_t j = 0; j < n; ++j)
		{
			GraphLib::Node node2 = random_node (nodes_num / 2, nodes_num - 1) (generator);
			graph.InsertEdge (GraphLib::Edge (node1, node2), 0);
		}
	}

	return graph;
}

void ProcessTest (Graph& graph, std::string name)
{
	auto node_printer = [] (auto& stream, auto node)
	{
		switch (node.color)
		{
			case GraphLib::Color::nocolor:
				stream << "[color=grey]";
				break;
			case GraphLib::Color::color_1:
				stream << "[color=red]";
				break;
			case GraphLib::Color::color_2:
				stream << "[color=blue]";
				break;
		}
	};

	auto edge_printer = [] (auto& stream, auto edge)
	{};

	GraphLib::Edge cycle_edge;
	std::cout << name << " graph" << std::endl;

	// BFS test
	std::cout << "BFS method: " << std::endl;
	if (GraphLib::BFSPaint (graph, cycle_edge))
		std::cout << "painting is successfull" << std::endl;
	else
	{
		std::cout << "painting is not possible" << std::endl;
		GraphLib::BFSPrintCycle (graph, cycle_edge);
	};
	graph.PrintToDot (name + "-bfs.dot", node_printer, edge_printer);

	// clear painting after previous algorithm
	for (GraphLib::Node i = 0; i < graph.GetNodesNum(); ++i)
		graph.NodeData(i) = NodeData();

	// DFS test
	std::cout << "DFS method: " << std::endl;
	if (GraphLib::DFSPaint (graph, cycle_edge))
		std::cout << "painting is successfull" << std::endl;
	else
	{
		std::cout << "painting is not possible" << std::endl;
		GraphLib::DFSPrintCycle (graph, cycle_edge);
	}
	graph.PrintToDot (name + "-dfs.dot", node_printer, edge_printer);
}

int main ()
{
	Graph complete = GenerateCompleteGraph (5, false);
	Graph path = GeneratePathGraph (5, false);
	Graph cycle = GenerateCycleGraph (5, false);

	std::vector<std::pair<GraphLib::Edge, char>> edges =
	{
		std::pair<GraphLib::Edge, char> (GraphLib::Edge (0, 1), 0),
		std::pair<GraphLib::Edge, char> (GraphLib::Edge (0, 2), 0),
		std::pair<GraphLib::Edge, char> (GraphLib::Edge (1, 3), 0),
		std::pair<GraphLib::Edge, char> (GraphLib::Edge (2, 3), 0),
		std::pair<GraphLib::Edge, char> (GraphLib::Edge (3, 4), 0),
		std::pair<GraphLib::Edge, char> (GraphLib::Edge (3, 5), 0),
		std::pair<GraphLib::Edge, char> (GraphLib::Edge (4, 6), 0),
		std::pair<GraphLib::Edge, char> (GraphLib::Edge (5, 6), 0)
	};
	Graph custom (std::vector<NodeData> (7), edges);
	Graph bigraph = GenerateBipartiteGraph (8, false);
	std::random_device generator;
	GraphLib::Node node1 = random_node (0, bigraph.GetNodesNum() / 2 - 1) (generator);
	GraphLib::Node node2 = random_node (node1 + 1, bigraph.GetNodesNum() / 2) (generator);
	bigraph.InsertEdge (GraphLib::Edge (node1, node2), 0);

	ProcessTest (complete, "complete");
	ProcessTest (path, "path");
	ProcessTest (cycle, "cycle");
	ProcessTest (custom, "custom");
	ProcessTest (bigraph, "bigraph");

	return 0;
}
