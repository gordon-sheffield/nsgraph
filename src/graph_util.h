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

#ifndef NS_GRAPH_UTIL_H
#define NS_GRAPH_UTIL_H


namespace ns::graph
{
    template <typename Iterator>
    class Span
    {
        public:
            Span(const Iterator& begin, const Iterator& end) : begin_(begin), end_(end) { }
            Span(Span& range_iterator) = default;
            Span(Span&& range_iterator) = default;

            Iterator begin() const { return begin_; }
            Iterator end() const { return end_; }
        private:
            Iterator begin_;
            Iterator end_;
    };

    template <typename E>
    struct EdgeInfoConst
    {
        int v1;
        int v2;
        const E& data;
    };

    template <typename E>
    struct EdgeInfo
    {
        int v1;
        int v2;
        E& data;
    };

    template <typename E>
    struct NeighborInfoConst
    {
        int neighbor_id;
        const E& data;
    };

    template <typename E>
    struct NeighborInfo
    {
        int neighbor_id;
        E& data;
    };
}

#endif //NS_GRAPH_UTIL_H
