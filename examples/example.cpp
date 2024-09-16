#include <iostream>

#include "../src/graph.h"
#include "../src/graph_algorithms.h"


void print_vertices(const ns::graph::Graph<int, double>& graph)
{
    std::cout << "vertices:" << std::endl;

    for (const auto vid : graph.vertices())
    {
        std::cout << vid << ", " << std::flush;
    }

    std::cout << std::endl;
}


void print_edges(const ns::graph::Graph<int, double>& graph)
{
    std::cout << "edges:" << std::endl;
    for(const auto [v1, v2, distance] : graph.edges())
    {
        std::cout << "(" << v1 << ", " << v2 << "): " << distance << std::endl;
    }
}


void print_path(const ns::graph::Graph<int, double>& graph, const std::vector<int>& path)
{
    double path_length = 0;

    for (int i = 0; i < static_cast<int>(path.size()) - 1; i++)
    {
        std::cout << path[i] << ", ";
        path_length += graph.edge(path[i], path[i+1]);
    }

    std::cout << path[path.size() - 1] << std::endl;
    std::cout << "distance: " << path_length << std::endl << std::endl;
}


int main()
{
    ns::graph::Graph<int, double> graph {};

    for (int i = 0; i < 6; i++)
    {
        graph.add_vertex(i);
    }

    using Edge = std::tuple<int, int, double>;
    std::vector<Edge> edges {
        Edge(0, 5, 14),
        Edge(0, 2, 9),
        Edge(0, 1, 7),
        Edge(1, 2, 10),
        Edge(1, 3, 15),
        Edge(2, 5, 2),
        Edge(2, 3, 11),
        Edge(3, 4, 6),
        Edge(4, 5, 9)
    };

    for (const auto& [u, v, distance] : edges)
    {
        graph.add_edge(u, v, distance);
        graph.add_edge(v, u, distance);
    }

    print_vertices(graph);
    std::cout << std::endl;

    print_edges(graph);
    std::cout << std::endl;

    const std::vector<int> shortest_path = ns::graph::shortest_path(graph, 0, 4);
    const std::vector<int> solution_path {0, 2, 5, 4};
    
    std::cout << "path:" << std::endl;
    print_path(graph, shortest_path);

    std::cout << "solution:" << std::endl;
    print_path(graph, solution_path);

    return 0;
}