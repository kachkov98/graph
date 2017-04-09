#include <iostream>
#include "graph.hpp"

struct NodeData
{
	GraphLib::Node prev;
	GraphLib::Color color;
};

typedef GraphLib::Graph<NodeData, char> Graph;

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

	std::cout << "Complete graph: ";
	if (PaintGraph (complete, cycle_edge))
	{
		std::cout << "painting is successfull" << std::endl;
		complete.PrintToDot ("complete.dot", node_printer, edge_printer);
	}
	else
	{
		std::cout << "painting is not possible" << std::endl;
		PrintCycle (complete, cycle_edge);
	}

	std::cout << "Path graph: ";
	if (PaintGraph (path, cycle_edge))
	{
		std::cout << "painting is successfull" << std::endl;
		path.PrintToDot ("path.dot", node_printer, edge_printer);
	}
	else
		PrintCycle (path, cycle_edge);

	std::cout << "Cycle graph: ";
	if (PaintGraph (cycle, cycle_edge))
	{
		std::cout << "painting is successfull" << std::endl;
		cycle.PrintToDot ("cycle.dot", node_printer, edge_printer);
	}
	else
	{
		std::cout << "painting is not possible" << std::endl;
		PrintCycle (cycle, cycle_edge);
	}

	std::cout << "Custom graph: ";
	if (PaintGraph (custom, cycle_edge))
	{
		std::cout << "painting is successfull" << std::endl;
		custom.PrintToDot ("custom.dot", node_printer, edge_printer);
	}
	else
	{
		std::cout << "painting is not possible" << std::endl;
		PrintCycle (custom, cycle_edge);
	}

	return 0;
}
