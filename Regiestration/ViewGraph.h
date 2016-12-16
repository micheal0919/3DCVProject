#ifndef __VIEW_GRAPH_H__
#define __VIEW_GRAPH_H__

#include <unordered_set>
#include <unordered_map>

#include "Types.h"

class CViewGraph
{
public:
	CViewGraph();
	~CViewGraph();

	// hash fucntion for unordered_map
	struct ViewIdPair_hash
	{
		std::size_t operator() (const ViewIdPair& p) const
		{
			auto h1 = std::hash<ViewId>{}(p.first);
			auto h2 = std::hash<ViewId>{}(p.second);

			return h1 ^ h2;
		}

	};

	// Number of views in the graph.
	int NumViews() const;

	// Number of undirected edges in the graph.
	int NumEdges() const;

	bool HasView(const ViewId view_id) const;

	bool HasEdge(const ViewId view_id_1, const ViewId view_id_2) const;

	// Returns a set of the ViewIds contained in the view graph.
	std::unordered_set<ViewId> ViewIds() const;

	// Removes the view from the view graph and removes all edges connected to the
	// view. Returns true on success and false if the view did not exist in the
	// view graph.
	bool RemoveView(const ViewId view_id);

	// Adds an edge between the two views with the edge value of
	// two_view_info. New vertices are added to the graph if they did not already
	// exist. If an edge already existed between the two views then the edge value
	// is updated.
	void AddEdge(const ViewId view_id_1, const ViewId view_id_2,
		const TwoViewInfo& two_view_info);

	// Removes the edge from the view graph. Returns true if the edge is removed
	// and false if the edge did not exist.
	bool RemoveEdge(const ViewId view_id_1, const ViewId view_id_2);

	// Returns the neighbor view ids for a given view, or nullptr if the view does
	// not exist.
	const std::unordered_set<ViewId>* GetNeighborIdsForView(
		const ViewId view_id) const;

	// Returns the edge value or NULL if it does not exist.
	const TwoViewInfo* GetEdge(const ViewId view_id_1,
		const ViewId view_id_2) const;

	TwoViewInfo* GetMutableEdge(const ViewId view_id_1, const ViewId view_id_2);

	// Returns a map of all edges. Each edge is found exactly once in the map and
	// is indexed by the ViewIdPair (view id 1, view id 2) such that view id 1 <
	// view id 2.
	const std::unordered_map<ViewIdPair, TwoViewInfo, ViewIdPair_hash>& GetAllEdges() const;

private:
	// The underlying adjacency map. ViewIds are the vertices which are mapped to
	// a collection of its neighbors and the edges themselves are stored
	// separately.
	std::unordered_map<ViewId, std::unordered_set<ViewId> > m_vertices;
	std::unordered_map<ViewIdPair, TwoViewInfo, ViewIdPair_hash> m_edges;
};

#endif // __VIEW_GRAPH_H_
