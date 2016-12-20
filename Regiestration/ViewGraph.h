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

	int NumViews() const;
	int NumEdges() const;
	bool HasView(const ViewId view_id) const;
	bool HasEdge(const ViewId view_id_1, const ViewId view_id_2) const;
	std::unordered_set<ViewId> ViewIds() const;
	bool RemoveView(const ViewId view_id);

	void AddEdge(const ViewId view_id_1, const ViewId view_id_2,
		const TwoViewInfo& two_view_info);
	bool RemoveEdge(const ViewId view_id_1, const ViewId view_id_2);
	const std::unordered_set<ViewId>* GetNeighborIdsForView(
		const ViewId view_id) const;

	const TwoViewInfo* GetEdge(const ViewId view_id_1,
		const ViewId view_id_2) const;

	TwoViewInfo* GetMutableEdge(const ViewId view_id_1, const ViewId view_id_2);

	const std::unordered_map<ViewIdPair, TwoViewInfo, ViewIdPair_hash>& GetAllEdges() const;

private:
	std::unordered_map<ViewId, std::unordered_set<ViewId> > m_vertices;
	std::unordered_map<ViewIdPair, TwoViewInfo, ViewIdPair_hash> m_edges;
};

#endif // __VIEW_GRAPH_H_
