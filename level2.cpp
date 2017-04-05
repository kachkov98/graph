#include <iostream>
#include <queue>
#include "graph.hpp"

enum class Color : char
{
    nocolor,
    color_1,
    color_2
};

struct NodeData
{
	GraphLib::Node prev = 0;
	Color color = Color::nocolor;
	NodeData (GraphLib::Node prev_ = 0, Color color_ = Color::nocolor) :
		prev (prev_),
		color (color_)
	{}
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

bool FindNocolorNode (const Graph &graph, GraphLib::Node &node)
{
	for (GraphLib::Node i = 0; i < graph.GetNodesNum(); ++i)
		if (graph.GetNode (i).color == Color::nocolor)
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

bool PaintGraph (Graph &graph, GraphLib::Edge &cycle)
{
	GraphLib::Node cur_node;
	while (FindNocolorNode (graph, cur_node))
	{
		// BFS
		std::queue<GraphLib::Node> cur_nodes;
		cur_nodes.push (cur_node);
		graph.SetNode (cur_node, NodeData (cur_node, Color::color_1));
		while (!cur_nodes.empty())
		{
			cur_node = cur_nodes.front();
			cur_nodes.pop();
			for (const auto &edge : graph.IncidentEdges (cur_node))
			{
				GraphLib::Node node = edge.first;
				if (graph.GetNode (node).color == Color::nocolor)
				{
					graph.SetNode (node, NodeData (cur_node, InverseColor (graph.GetNode (cur_node).color)));
					cur_nodes.push (node);
				}
				else if (graph.GetNode (node).color != InverseColor (graph.GetNode (cur_node).color))
				{
					cycle = std::make_pair (cur_node, node);
					return false;
				}
			}
		}
	}
	return true;
}

void PrintCycle (const Graph &graph, const GraphLib::Edge &cycle)
{
	std::cout << "Nodes in cycle:" << std::endl;

	GraphLib::Node cur_node = cycle.first;
	while (graph.GetNode (cur_node).prev != cur_node)
	{
		std::cout << cur_node << ' ';
		cur_node = graph.GetNode (cur_node).prev;
	}

	cur_node = cycle.second;
	while (graph.GetNode (cur_node).prev != cur_node)
	{
		std::cout << cur_node << ' ';
		cur_node = graph.GetNode (cur_node).prev;
	}
	std::cout << cur_node << std::endl;
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
			case Color::nocolor:
				stream << "[color=grey]";
				break;
			case Color::color_1:
				stream << "[color=red]";
				break;
			case Color::color_2:
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
