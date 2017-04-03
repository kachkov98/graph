#include <iostream>
#include "graph.hpp"

enum class Color : char
{
	nocolor,
    color_1,
    color_2
};

typedef GraphLib::Graph<Color, char> Graph;

Graph GenerateCompleteGraph (std::size_t nodes_num, bool directed)
{
	Graph graph (directed);

	for (GraphLib::Node i = 0; i < nodes_num; ++i)
		graph.InsertNode (Color::nocolor);

	for (GraphLib::Node i = 0; i < nodes_num - 1; ++i)
		for (GraphLib::Node j = i + 1; j < nodes_num; ++j)
			graph.InsertEdge (GraphLib::Edge (i, j), 0);

	return graph;
}

Graph GeneratePathGraph (std::size_t nodes_num, bool directed)
{
	Graph graph (directed);

	for (GraphLib::Node i = 0; i < nodes_num; ++i)
		graph.InsertNode (Color::nocolor);

	for (GraphLib::Node i = 0; i < nodes_num - 1; ++i)
		graph.InsertEdge (GraphLib::Edge (i, i + 1), 0);

	return graph;
}

Graph GenerateCycleGraph (std::size_t nodes_num, bool directed)
{
	Graph graph (directed);

	for (GraphLib::Node i = 0; i < nodes_num; ++i)
		graph.InsertNode (Color::nocolor);

	for (GraphLib::Node i = 0; i < nodes_num; ++i)
		graph.InsertEdge (GraphLib::Edge (i, (i + 1) % nodes_num), 0);

	return graph;
}

int main ()
{
	Graph complete = GenerateCompleteGraph (5, false);
	Graph path = GeneratePathGraph (5, false);
	Graph cycle = GenerateCycleGraph (5, false);
	auto node_printer = [](auto& stream, auto node)
	{
		switch (node)
		{
			case Color::nocolor: stream << "[color=grey]"; break;
			case Color::color_1: stream << "[color=red]";  break;
			case Color::color_2: stream << "[color=blue]"; break;
		}
	};
	auto edge_printer = [](auto& stream, auto edge)
	{};
	complete.PrintToDot ("complete.dot", node_printer, edge_printer);
	path.PrintToDot ("path.dot", node_printer, edge_printer);
	cycle.PrintToDot ("cycle.dot", node_printer, edge_printer);
	return 0;
}
