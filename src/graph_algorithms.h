/*
Copyright 2024 Gordon Sheffield

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef NS_GRAPH_ALGORITHMS_H
#define NS_GRAPH_ALGORITHMS_H

#include <vector>
#include <limits>
#include <queue>
#include <algorithm>

#include "graph.h"


namespace ns::graph
{
    /**
    * @brief Finds the shortest path between any 2 vertices in the graph.
    *
    * Internally uses Dijkstra's algorithm and hence runs in $ O(|E| * |V| \log |V|) $ time.
    *
    * @returns the shortest path between start_vertex and end_vertex. If a path doesn't exist, then an empty path is returned instead.
    */
    template<typename V>
    std::vector<int> shortest_path(const Graph<V, double>& graph, const int start_vertex, const int end_vertex)
    {
        struct HeapKeyValue
        {
            int vertex_id;
            double accumulated_distance;

            HeapKeyValue(const int vertex_id, const double accumulated_distance)
                : vertex_id(vertex_id), accumulated_distance(accumulated_distance) {}
        };

        constexpr auto compare_less_than = [](const HeapKeyValue& h1, const HeapKeyValue& h2) -> bool
        {
            return h1.accumulated_distance > h2.accumulated_distance;
        };

        constexpr double MAX_DISTANCE = std::numeric_limits<double>::infinity();
        constexpr double MIN_DISTANCE = 0;

        std::priority_queue<HeapKeyValue, std::vector<HeapKeyValue>, decltype(compare_less_than)> min_heap(compare_less_than);
        std::vector<double> vertex_distance = graph.template vertex_id_mapping<double>(MAX_DISTANCE);
        std::vector<int> visited_vertex_map = graph.template vertex_id_mapping<int>();

        // and kick-start the process by saying we've reached the start_vertex with 0 accumulated distance
        min_heap.emplace(start_vertex, MIN_DISTANCE);
        vertex_distance[start_vertex] = MIN_DISTANCE;

        while ( ! min_heap.empty())
        {
            const auto [vid, distance] = min_heap.top();
            min_heap.pop();

            const bool invalid_heap_entry = (distance != vertex_distance[vid]);
            if (invalid_heap_entry) { continue; }

            if (vid == end_vertex) { break; }

            for (const auto [neighbor_id, neighbor_distance] : graph.neighbors(vid))
            {
                const double accumulated_distance = neighbor_distance + distance;

                const bool update_distance = (accumulated_distance < vertex_distance[neighbor_id]);
                if ( ! update_distance) { continue; }

                visited_vertex_map[neighbor_id] = vid;
                vertex_distance[neighbor_id] = accumulated_distance;

                min_heap.emplace(neighbor_id, accumulated_distance);
            }
        }

        const bool end_vertex_never_reached = (vertex_distance[end_vertex] == MAX_DISTANCE);
        if (end_vertex_never_reached)
        {
            return {};
        }

        std::vector<int> path {};
        int current_vertex_id = end_vertex;

        while (current_vertex_id != start_vertex)
        {
            path.push_back(current_vertex_id);
            current_vertex_id = visited_vertex_map[current_vertex_id];
        }

        path.push_back(start_vertex);
        std::reverse(path.begin(), path.end());

        return path;
    }

}

#endif // NS_GRAPH_ALGORITHMS_H