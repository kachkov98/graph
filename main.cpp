#include <iostream>
#include "graph.hpp"

int main ()
{
	std::vector<int> nodes = {1, 2, 3, 4, 5};
	std::vector<std::pair<GraphLib::Edge, int>> edges =
	{
		std::pair<GraphLib::Edge, int>(GraphLib::Edge(0, 1), 1),
		std::pair<GraphLib::Edge, int>(GraphLib::Edge(1, 2), 2),
		std::pair<GraphLib::Edge, int>(GraphLib::Edge(1, 3), 3),
		std::pair<GraphLib::Edge, int>(GraphLib::Edge(2, 4), 1),
		std::pair<GraphLib::Edge, int>(GraphLib::Edge(3, 4), 2),
		std::pair<GraphLib::Edge, int>(GraphLib::Edge(2, 3), 3)
	};
	GraphLib::Graph<int, int> a(nodes, edges, false);
	GraphLib::Graph<int, int> b(nodes, edges, true);
	a.RemoveEdge(GraphLib::Edge(2, 3));
	b.InsertEdge(GraphLib::Edge(1, 0), 1);
	a.PrintGraph();
	b.PrintGraph();
	a.PrintToDot("a.dot");
	b.PrintToDot("b.dot");
	return 0;
}
