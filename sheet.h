#pragma once

#include "cell.h"
#include "common.h"

#include <functional>
#include <unordered_map>
#include <iterator>

namespace graph {

	namespace ranges {

		template <typename It>
		class Range {
		public:
			using ValueType = typename std::iterator_traits<It>::value_type;

			Range(It begin, It end)
				: begin_(begin)
				, end_(end) {
			}
			It begin() const {
				return begin_;
			}
			It end() const {
				return end_;
			}

		private:
			It begin_;
			It end_;
		};

		template <typename C>
		auto AsRange(const C& container) {
			return Range{ container.begin(), container.end() };
		}

	}  // namespace ranges

	using VertexId = Position;

	struct Edge {
		VertexId from;
		VertexId to;
		bool operator==(const Edge& other) const {
			return from == other.from && to == other.to;
		}
	};

	class GraphHasher {
		size_t operator()(const Position& pos) const {
			return std::hash<int>()(pos.row) + std::hash<int>()(pos.col) * 37;
		}

		size_t operator()(const Edge& edge) const {
			return this->operator()(edge.from) + this->operator()(edge.to) * 43;
		}

		size_t operator()(const Edge* item) const {
			return this->operator()(*item);
		}
	};

	using IncidenceList = std::unordered_set<const Edge*, GraphHasher>;
	using IncidentEdgesRange = ranges::Range<typename IncidenceList::const_iterator>;
	using IncidentEdges = std::unordered_map<VertexId, IncidenceList, GraphHasher>;
	using EdgeSetContainer = std::unordered_set<Edge, GraphHasher>;

	class AbstractGraph {
	public:
		virtual bool AddEdge(Edge edge) = 0;
		virtual bool HasEdge(const Edge& edge) const = 0;

		virtual size_t GetVertexCount() const = 0;
		virtual size_t GetEdgeCount() const = 0;
		virtual IncidentEdgesRange GetIncidentEdges(VertexId vertex) const = 0;

		virtual bool EraseEdge(const Edge& edge) = 0;
		virtual bool EraseVertex(const VertexId& vertex_id) = 0;

		virtual void Traversal(const VertexId& vertex_id, std::function<bool(const Edge*)> action) const = 0;
		virtual bool DetectCircularDependency(const VertexId& from, const std::vector<VertexId>& to_refs) const = 0;

	protected:
		virtual size_t AddEdgesImpl(EdgeSetContainer::iterator begin, EdgeSetContainer::iterator end) = 0;
	};

	class DirectedGraph : AbstractGraph {
		friend DependencyGraph;
	public:
		DirectedGraph(EdgeSetContainer&& edges, IncidentEdges&& incidence_lists);

		template <typename It, std::enable_if_t<std::is_same_v<typename std::iterator_traits<It>::value_type, Edge>, bool> = true>
		size_t AddEdges(It begin, It end);
		bool AddEdge(Edge edge) override;
		bool HasEdge(const Edge& edge) const override;

		size_t GetVertexCount() const override;
		size_t GetEdgeCount() const override;
		IncidentEdgesRange GetIncidentEdges(VertexId vertex) const override;

		virtual bool EraseEdge(const Edge& edge) override;
		virtual bool EraseVertex(const VertexId& vertex_id) override;

		void Traversal(const VertexId& vertex_id, std::function<bool(const Edge*)> action) const override;
		bool DetectCircularDependency(const VertexId& from, const std::vector<VertexId>& to_refs) const override;

	protected:
		size_t AddEdgesImpl(EdgeSetContainer::iterator begin, EdgeSetContainer::iterator end) override;

		EdgeSetContainer edges_;
		IncidentEdges incidence_lists_;
	};

	class DependencyGraph : AbstractGraph {
	public:
		enum class Direction { forward, backward };

		DependencyGraph();
		DependencyGraph(DirectedGraph forward_graph, DirectedGraph backward_graph);
		~DependencyGraph();

		template <typename It, std::enable_if_t<std::is_same_v<typename std::iterator_traits<It>::value_type, Edge>, bool> = true>
		size_t AddEdges(It begin, It end);
		bool AddEdge(Edge edge) override;
		bool HasEdge(const Edge& edge) const override;

		size_t GetVertexCount() const override;
		size_t GetEdgeCount() const override;
		IncidentEdgesRange GetIncidentEdges(VertexId vertex) const override;

		bool EraseEdge(const Edge& edge) override;
		bool EraseVertex(const VertexId& vertex_id) override;

		void Traversal(const VertexId& vertex_id, std::function<bool(const Edge*)> action, Direction direction = Direction::forward) const;
		void Traversal(const VertexId& vertex_id, std::function<bool(const Edge*)> action) const override;
		bool DetectCircularDependency(const VertexId& from, const std::vector<VertexId>& to_refs) const override;

	private:
		size_t AddEdgesImpl(EdgeSetContainer::iterator begin, EdgeSetContainer::iterator end) override;

		DirectedGraph forward_graph_;
		DirectedGraph backward_graph_;
	};
}

class Sheet : public SheetInterface {
public:
	~Sheet();

	void SetCell(Position pos, std::string text) override;

	const CellInterface* GetCell(Position pos) const override;
	CellInterface* GetCell(Position pos) override;

	void ClearCell(Position pos) override;

	Size GetPrintableSize() const override;

	void PrintValues(std::ostream& output) const override;
	void PrintTexts(std::ostream& output) const override;

	const graph::DependencyGraph& GetGraph() const;

private:
	void InvalidateCache_(const Position& pos);

	std::unordered_map<int, std::unordered_map<int, std::unique_ptr<Cell>>> sheet_;
	graph::DependencyGraph graph_;
};