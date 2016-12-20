#include "ViewGraph.h"

#include "MapUtils.h"

CViewGraph::CViewGraph()
{
}


CViewGraph::~CViewGraph()
{
}

int CViewGraph::NumViews() const 
{ 
	return m_vertices.size(); 
}

int CViewGraph::NumEdges() const 
{
	return m_edges.size();
}

std::unordered_set<ViewId> CViewGraph::ViewIds() const 
{
	std::unordered_set<ViewId> view_ids;
	view_ids.reserve(m_vertices.size());
	for (const auto& vertex : m_vertices) {
		view_ids.insert(vertex.first);
	}
	return view_ids;
}

bool CViewGraph::HasView(const ViewId view_id) const 
{
	return ContainsKey(m_vertices, view_id);
}

bool CViewGraph::HasEdge(const ViewId view_id_1, const ViewId view_id_2) const 
{
	const ViewIdPair view_id_pair = (view_id_1 < view_id_2)
		? ViewIdPair(view_id_1, view_id_2)
		: ViewIdPair(view_id_2, view_id_1);
	return ContainsKey(m_edges, view_id_pair);
}

bool CViewGraph::RemoveView(const ViewId view_id) 
{
	const auto* neighbor_ids = FindOrNull(m_vertices, view_id);
	if (neighbor_ids == nullptr) {
		return false;
	}

	for (const ViewId neighbor_id : *neighbor_ids) {
		m_vertices[neighbor_id].erase(view_id);
		const ViewIdPair view_id_pair = (view_id < neighbor_id)
			? ViewIdPair(view_id, neighbor_id)
			: ViewIdPair(neighbor_id, view_id);
		m_edges.erase(view_id_pair);
	}

	m_vertices.erase(view_id);
	return true;
}

void CViewGraph::AddEdge(const ViewId view_id_1, const ViewId view_id_2,
	const TwoViewInfo& two_view_info) 
{
	if (view_id_1 == view_id_2) {
		DLOG(WARNING) << "Cannot add an edge from view id " << view_id_1
			<< " to itself!";
		return;
	}

	const ViewIdPair view_id_pair = (view_id_1 < view_id_2)
		? ViewIdPair(view_id_1, view_id_2)
		: ViewIdPair(view_id_2, view_id_1);

	DLOG_IF(WARNING, ContainsKey(m_edges, view_id_pair))
		<< "An edge already exists between view " << view_id_1 << " and view "
		<< view_id_2;

	m_vertices[view_id_1].insert(view_id_2);
	m_vertices[view_id_2].insert(view_id_1);
	m_edges[view_id_pair] = two_view_info;
}

bool CViewGraph::RemoveEdge(const ViewId view_id_1, const ViewId view_id_2) 
{
	const ViewIdPair view_id_pair = (view_id_1 < view_id_2)
		? ViewIdPair(view_id_1, view_id_2)
		: ViewIdPair(view_id_2, view_id_1);
	if (!ContainsKey(m_edges, view_id_pair)) {
		return false;
	}

	if (m_vertices[view_id_1].erase(view_id_2) == 0 ||
		m_vertices[view_id_2].erase(view_id_1) == 0 ||
		m_edges.erase(view_id_pair) == 0) {
		return false;
	}

	return true;
}

const std::unordered_set<ViewId>* CViewGraph::GetNeighborIdsForView(
	const ViewId view_id) const 
{
	return FindOrNull(m_vertices, view_id);
}

const TwoViewInfo* CViewGraph::GetEdge(const ViewId view_id_1,
	const ViewId view_id_2) const 
{
	const ViewIdPair view_id_pair = (view_id_1 < view_id_2)
		? ViewIdPair(view_id_1, view_id_2)
		: ViewIdPair(view_id_2, view_id_1);
	return FindOrNull(m_edges, view_id_pair);
}

TwoViewInfo* CViewGraph::GetMutableEdge(const ViewId view_id_1,
	const ViewId view_id_2) 
{
	const ViewIdPair view_id_pair = (view_id_1 < view_id_2)
		? ViewIdPair(view_id_1, view_id_2)
		: ViewIdPair(view_id_2, view_id_1);
	return FindOrNull(m_edges, view_id_pair);
}

const std::unordered_map<ViewIdPair, TwoViewInfo, CViewGraph::ViewIdPair_hash>& CViewGraph::GetAllEdges() const
{
	return m_edges;
}
