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

#ifndef NS_GRAPH_H
#define NS_GRAPH_H

#include <vector>
#include <utility>
#include <stdexcept>
#include <algorithm>
#include <optional>

#include "graph_util.h"


namespace ns::graph
{
    /**
    * @brief A directed graph with loops. Data is associated with each vertex and edge.
    */
    template<typename V, typename E>
    class Graph
    {
        // constants
        public:
            static constexpr int kInvalidVertexId = -1;

        // data types
        private:
            struct EdgeMetadata
            {
                int neighbor_id;
                E data;

                EdgeMetadata(const int neighbor_id_, const E& data_) :
                    neighbor_id(neighbor_id_), data(data_)
                { }

                EdgeMetadata(const int neighbor_id_, E&& data_) :
                    neighbor_id(neighbor_id_), data(std::move(data_))
                { }
            };

            struct VertexMetadata
            {
                V data;
                std::vector<EdgeMetadata> edges;

                VertexMetadata(const V& data_, std::vector<EdgeMetadata>&& edges_) :
                    data(data_), edges(std::move(edges_))
                { }
            };

        // data types
        public:
            class VertexIterator
            {
                public:
                    VertexIterator(const Graph& graph, const std::vector<bool>::const_iterator& cursor) :
                        id_to_valid_begin_(graph.id_to_valid_.begin()),
                        id_to_valid_end_(graph.id_to_valid_.end()),
                        cursor_(cursor)
                    { }

                    bool operator==(const VertexIterator& rhs) const { return (cursor_ == rhs.cursor_); }
                    bool operator !=(const VertexIterator& rhs) const { return (cursor_ != rhs.cursor_); }
                    int operator*() const { return (cursor_ - id_to_valid_begin_); }

                    VertexIterator& operator++()
                    {
                        do { ++cursor_; } while ( ! *cursor_ && cursor_ != id_to_valid_end_);
                        return *this;
                    }

                private:
                    const std::vector<bool>::const_iterator id_to_valid_begin_;
                    const std::vector<bool>::const_iterator id_to_valid_end_;
                    std::vector<bool>::const_iterator cursor_;
            };

            class NeighborConstIterator
            {
                public:
                    explicit NeighborConstIterator(typename std::vector<EdgeMetadata>::const_iterator iterator) : iterator_(iterator) { }

                    NeighborInfoConst<E> operator*() const { return {iterator_->neighbor_id, iterator_->data}; }
                    bool operator==(const NeighborConstIterator& rhs) const { return iterator_ == rhs.iterator_; }
                    bool operator!=(const NeighborConstIterator& rhs) const { return iterator_ != rhs.iterator_; }
                    void operator++() { ++iterator_; }

                private:
                    typename std::vector<EdgeMetadata>::const_iterator iterator_;
            };

            class NeighborIterator
            {
                public:
                    explicit NeighborIterator(typename std::vector<EdgeMetadata>::const_iterator iterator) : iterator_(iterator) { }

                    NeighborInfo<E> operator*() const { return {iterator_->neighbor_id, iterator_->data}; }
                    bool operator==(const NeighborIterator& rhs) const { return iterator_ == rhs.iterator_; }
                    bool operator!=(const NeighborIterator& rhs) const { return iterator_ != rhs.iterator_; }
                    void operator++() { ++iterator_; }

                private:
                    typename std::vector<EdgeMetadata>::const_iterator iterator_;
            };

            class EdgeConstIterator
            {
                public:
                    explicit EdgeConstIterator(const Graph& graph, const std::vector<bool>::const_iterator& cursor) :
                        graph_(graph),
                        vertex_iterator_(graph, cursor),
                        vertex_iterator_end_(graph, graph.id_to_valid_.end())
                    {
                        if (vertex_iterator_ == vertex_iterator_end_) { return; }

                        const int vid = *vertex_iterator_;
                        const VertexMetadata& vertex = graph.id_to_vertex_[vid];

                        neighbor_iterator_     = vertex.edges.begin();
                        neighbor_iterator_end_ = vertex.edges.end();
                    }

                    EdgeInfoConst<E> operator*() const
                    {
                        const EdgeMetadata& edge = *neighbor_iterator_;

                        return {
                            (*vertex_iterator_),
                            edge.neighbor_id,
                            edge.data
                        };
                    }

                    bool operator!=(const EdgeConstIterator& rhs) const { return (neighbor_iterator_ != rhs.neighbor_iterator_); }
                    bool operator==(const EdgeConstIterator& rhs) const { return (neighbor_iterator_ == rhs.neighbor_iterator_); }

                    void operator++()
                    {
                        if (++neighbor_iterator_ != neighbor_iterator_end_) { return; }

                        while ( ++vertex_iterator_ != vertex_iterator_end_ ) {
                            const int vid = *vertex_iterator_;
                            const VertexMetadata& vertex = graph_.id_to_vertex_[vid];

                            neighbor_iterator_ = vertex.edges.begin();
                            neighbor_iterator_end_ = vertex.edges.end();

                            if (neighbor_iterator_ != neighbor_iterator_end_) { return; }
                        }

                        // ran out of vertices and neighbors, set neighbor iterators to the end sentinel
                        neighbor_iterator_ = {};
                        neighbor_iterator_end_ = {};
                    }

                private:
                    const Graph& graph_;
                    VertexIterator vertex_iterator_;
                    const VertexIterator vertex_iterator_end_;
                    typename std::vector<EdgeMetadata>::const_iterator neighbor_iterator_;
                    typename std::vector<EdgeMetadata>::const_iterator neighbor_iterator_end_;
            };

            class EdgeIterator
            {
                public:
                    explicit EdgeIterator(Graph& graph, const std::vector<bool>::const_iterator& cursor) :
                        graph_(graph),
                        vertex_iterator_(graph, cursor),
                        vertex_iterator_end_(graph, graph.id_to_valid_.end())
                    {
                        if (vertex_iterator_ == vertex_iterator_end_) { return; }

                        const int vid = *vertex_iterator_;
                        VertexMetadata& vertex = graph.id_to_vertex_[vid];

                        neighbor_iterator_     = vertex.edges.begin();
                        neighbor_iterator_end_ = vertex.edges.end();
                    }

                    EdgeInfo<E> operator*() const
                    {
                        EdgeMetadata& edge = *neighbor_iterator_;

                        return {
                            (*vertex_iterator_),
                            edge.neighbor_id,
                            edge.data
                        };
                    }

                    bool operator!=(const EdgeIterator& rhs) const { return (neighbor_iterator_ != rhs.neighbor_iterator_); }
                    bool operator==(const EdgeIterator& rhs) const { return (neighbor_iterator_ == rhs.neighbor_iterator_); }

                    void operator++()
                    {
                        if (++neighbor_iterator_ != neighbor_iterator_end_) { return; }

                        while ( ++vertex_iterator_ != vertex_iterator_end_ ) {
                            const int vid = *vertex_iterator_;
                            VertexMetadata& vertex = graph_.id_to_vertex_[vid];

                            neighbor_iterator_ = vertex.edges.begin();
                            neighbor_iterator_end_ = vertex.edges.end();

                            if (neighbor_iterator_ != neighbor_iterator_end_) { return; }
                        }

                        // ran out of vertices and neighbors, set neighbor iterators to the end sentinel
                        neighbor_iterator_ = {};
                        neighbor_iterator_end_ = {};
                    }

                private:
                    Graph& graph_;
                    VertexIterator vertex_iterator_;
                    const VertexIterator vertex_iterator_end_;
                    typename std::vector<EdgeMetadata>::iterator neighbor_iterator_;
                    typename std::vector<EdgeMetadata>::iterator neighbor_iterator_end_;
            };

        // constructors
        public:

            /**
            * Constructs an empty graph with no vertices and no edges.
            */
            Graph() = default;

            /**
            * Copies the given to graph to this graph. All vertex and edge ids are preserved.
            */
            Graph(const Graph& graph) = default;
            Graph& operator=(const Graph& graph) = default;

            /**
            * Destructively moves the given graph into this graph.
            */
            Graph(Graph&& graph) = default;
            Graph& operator=(Graph&& graph) = default;

        // methods
        public:

            /**
            * Adds the given vertex data to the graph and returns a vertex id handle
            *
            * @returns vertex_id handle to the vertex data
            */
            int add_vertex(const V& v)
            {
                if ( ! available_ids_.empty())
                {
                    const int id = available_ids_.back();
                    available_ids_.pop_back();

                    id_to_vertex_[id].data = v;
                    id_to_valid_[id] = true;

                    return id;
                }

                const int vertex_id = (id_to_valid_.size());

                id_to_valid_.push_back(true);
                id_to_vertex_.emplace_back(v, std::vector<EdgeMetadata>());

                return vertex_id;
            }

            /**
            * Adds the given vertex data to the graph and returns a vertex id handle
            *
            * @returns vertex_id associated with the added vertex
            */
            int add_vertex(V&& v)
            {
                if ( ! available_ids_.empty())
                {
                    const int id = available_ids_.back();
                    available_ids_.pop_back();

                    id_to_vertex_[id].data = v;
                    id_to_valid_[id] = true;

                    return id;
                }

                const int id = (id_to_valid_.size());

                id_to_valid_.push_back(true);
                id_to_vertex_.emplace_back(std::move(v), std::vector<EdgeMetadata>());

                return id;
            }

            /**
            * Deletes the given vertex from the graph, including all edges the vertex is connected to.
            */
            void del_vertex(int vid)
            {
                available_ids_.push_back(vid);

                id_to_vertex_[vid] = {};
                id_to_valid_[vid] = false;
            }

            /**
            * @returns the vertex data associated with the vertex id vid
            */
            [[nodiscard]] V pop_vertex(int vid)
            {
                const V vertex_data = id_to_vertex_[vid].data;
                del_vertex(vid);

                return vertex_data;
            }

            /**
            * @returns a reference to the vertex data associated with the vertex id vid. The reference is invalidated upon any addition/removal of a vertex.
            */
            [[nodiscard]] V& vertex(int vid)
            {
                VertexMetadata& metadata = id_to_vertex_[vid];
                return metadata.data;
            }

            /**
            * @param vid the vertex id handle of the vertex data
            *
            * @returns a reference to the vertex data associated with the vertex id vid. The reference is invalidated upon any addition/removal of a vertex.
            */
            [[nodiscard]] const V& vertex(int vid) const
            {
                const VertexMetadata& metadata = id_to_vertex_[vid];
                return metadata.data;
            }

            /**
            * @returns a container of vertex ids; invalidated upon any vertex additions/removals
            */
            [[nodiscard]] Span<VertexIterator> vertices() const
            {
                return {
                    VertexIterator(*this, id_to_valid_.begin()),
                    VertexIterator(*this, id_to_valid_.end())
                };
            }

            /**
             * @returns the number of vertices
             */
            [[nodiscard]] int num_vertices() const
            {
                return (id_to_vertex_.size() - available_ids_.size());
            }

            /**
            * Adds the edges going from v1 to v2 with edge data e.
            */
            void add_edge(int v1, int v2, const E& e)
            {
                VertexMetadata& metadata = id_to_vertex_[v1];
                metadata.edges.emplace_back(v2, e);
            }

            /**
            * Adds the edges going from v1 to v2 with edge data e.
            */
            void add_edge(int v1, int v2, E&& e)
            {
                VertexMetadata& metadata = id_to_vertex_[v1];
                metadata.edges.emplace_back(v2, std::move(e));
            }

            /**
            * Deletes the edges going from v1 to v2.
            */
            void del_edge(int v1, int v2)
            {
                std::vector<EdgeMetadata>& edges = id_to_vertex_[v1].edges;

                auto is_connected_to_v2 = [v2](const EdgeMetadata& metadata) -> bool
                {
                    return (metadata.neighbor_id == v2);
                };

                auto edge_metadata_iter = std::find_if(edges.begin(), edges.end(), is_connected_to_v2);

                if (edge_metadata_iter == edges.end())
                {
                    throw std::runtime_error("ns::graph::Graph::del_edge(int, int) : invalid edge !");
                }

                edges.erase(edge_metadata_iter);
            }

            /**
            * @returns the edge data associated with the edge from v1 to v2
            */
            [[nodiscard]] E& edge(int v1, int v2)
            {
                VertexMetadata& metadata = id_to_vertex_[v1];

                for (auto& [neighbor_id, data] : metadata.edges)
                {
                    if (neighbor_id == v2)
                    {
                        return data;
                    }
                }

                throw std::runtime_error("ns::graph::Graph::edge(int, int) : invalid edge !");
            }

            /**
            * @returns the edge data associated with the edge from v1 to v2
            */
            [[nodiscard]] const E& edge(int v1, int v2) const
            {
                const VertexMetadata& metadata = id_to_vertex_[v1];

                for (const auto& [neighbor_id, data] : metadata.edges)
                {
                    if (neighbor_id == v2)
                    {
                        return data;
                    }
                }

                throw std::runtime_error("ns::graph::Graph::edge(int, int) const : invalid edge !");
            }

            /**
             * @returns the edge data associated with the edge from v1 to v2; if no edge exists, returns an empty optional
             */
            [[nodiscard]] std::optional<E&> edge_optional(int v1, int v2)
            {
                VertexMetadata& metadata = id_to_vertex_[v1];

                for (auto& [neighbor_id, data] : metadata.edges)
                {
                    if (neighbor_id == v2)
                    {
                        return data;
                    }
                }

                return {};
            }

            /**
            * @returns the edge data associated with the edge from v1 to v2
            */
            [[nodiscard]] std::optional<const E&> edge_optional(int v1, int v2) const
            {
                const VertexMetadata& metadata = id_to_vertex_[v1];

                for (const auto& [neighbor_id, data] : metadata.edges)
                {
                    if (neighbor_id == v2)
                    {
                        return data;
                    }
                }

                return {};
            }

            /**
            * @returns a container of neighbor ids; invalidated upon vertex or edge additions/removals
            */
            [[nodiscard]] Span<NeighborConstIterator> neighbors(int vid) const
            {
                const VertexMetadata& metadata = id_to_vertex_[vid];

                return {
                    NeighborConstIterator(metadata.edges.begin()),
                    NeighborConstIterator(metadata.edges.end())
                };
            }

            /**
            * @returns a container of neighbor ids; invalidated upon vertex or edge additions/removals
            */
            [[nodiscard]] Span<NeighborIterator> neighbors(int vid)
            {
                const VertexMetadata& metadata = id_to_vertex_[vid];

                return {
                    NeighborIterator(metadata.edges.begin()),
                    NeighborIterator(metadata.edges.end())
                };
            }

            /**
            * @returns a container of edge ids; invalidated upon edge additions/removals
            */
            [[nodiscard]] Span<EdgeConstIterator> edges() const
            {
                return {
                    EdgeConstIterator(*this, id_to_valid_.begin()),
                    EdgeConstIterator(*this, id_to_valid_.end())
                };
            }

            /**
            * @returns a container of edge ids; invalidated upon edge additions/removals
            */
            [[nodiscard]] Span<EdgeIterator> edges()
            {
                return {
                    EdgeIterator(*this, id_to_valid_.begin()),
                    EdgeIterator(*this, id_to_valid_.end())
                };
            }

            /**
            * @returns the degree of the vertex
            */
            [[nodiscard]] int degree(int vid) const
            {
                return id_to_vertex_[vid].edges.size();
            }

            /**
            * @returns an efficient representation of a mapping of vertex ids to the given type T.
            */
            template<typename T>
            [[nodiscard]] std::vector<T> vertex_id_mapping() const
            {
                return std::vector<T>(id_to_vertex_.size());
            }

            /**
            * @returns an efficient representation of a mapping of vertex ids to the given type T.
            */
            template<typename T>
            [[nodiscard]] std::vector<T> vertex_id_mapping(const T& value) const
            {
                return std::vector<T>(id_to_vertex_.size(), value);
            }

            /**
            * @brief Shrinks the data structure capacity to exactly match the number of vertices.
            *
            * This will invalidate all previous associations of vertices with their vertex id.
            */
            void shrink()
            {
                if (available_ids_.empty()) { return; }

                const int old_num_vertices = num_vertices();

                std::vector<VertexMetadata> new_id_to_vertex(old_num_vertices);
                std::vector<int> new_id_to_old(old_num_vertices);

                int new_vid = 0;
                for (auto old_vid : vertices())
                {
                    new_id_to_vertex[new_vid] = std::move(id_to_vertex_[old_vid]);
                    new_id_to_old[new_vid] = old_vid;

                    ++new_vid;
                }

                for (VertexMetadata& vertex_metadata : new_id_to_vertex)
                {
                    for (auto& [neighbor_id, _] : vertex_metadata.edges)
                    {
                        neighbor_id = new_id_to_old[neighbor_id];
                    }
                }

                id_to_vertex_ = std::move(new_id_to_vertex);
                id_to_valid_ = std::vector<bool>(old_num_vertices, true);
                available_ids_ = {};
            }

            /**
            * @brief Shrinks the data structure capacity to exactly match the number of vertices.
            *
            * This will invalidate all previous associations of vertices with their vertex id.
            *
            * @returns a mapping associating old vertex ids to the new ones
            */
            [[nodiscard]] std::vector<int> shrink_return_mapping()
            {
                std::vector<int> mapping = vertex_id_mapping<int>();

                int new_id = 0;
                for (const int old_id : vertices())
                {
                    mapping[old_id] = new_id++;
                }

                shrink();

                return mapping;
            }

        private:
            std::vector<VertexMetadata> id_to_vertex_;
            std::vector<bool> id_to_valid_;
            std::vector<int> available_ids_;
    };
}

#endif // NS_GRAPH_H