#ifndef __CONNECTED_COMPONENTS_H__
#define __CONNECTED_COMPONENTS_H__

#include <stdint.h>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include <glog/logging.h>

#include "MapUtils.h"

// A connected components algorithm based on the union-find structure. Connected
// components from a graph are needed for estimating poses from a view graph and
// for generating tracks from image correspondences.
//
// This particular implementation can utilize an upper limit on the size of a
// connected component. This is useful when generating tracks in SfM since large
// tracks are increasingly likely to have outliers.
//
// NOTE: The template parameter T must be a type compatible with
// numeric_limits<T> e.g. int, uint16_t, etc.
template <typename T>
class CConnectedComponents {
public:
	// The Root struct is used to store the connected component that each node is
	// a part of. Each node is mapped to a Root and all nodes that map to a root
	// with the same ID are part of the same connected component.
	struct Root {
		Root(const T& id, const int size) : id(id), size(size) {}
		T id;
		int size;
	};

	CConnectedComponents()
		: m_max_connected_component_size(std::numeric_limits<T>::max()) {}

	// Specify the maximum connected component size.
	explicit CConnectedComponents(const int max_size)
		: m_max_connected_component_size(max_size) {
		CHECK_GT(m_max_connected_component_size, 0);
	}

	// Adds an edge connecting the two nodes to the connected component graph. The
	// edge is inserted as a new connected component if the edge is not present in
	// the graph. The edge is added to the current connected component if at least
	// one of the nodes already exists in the graph, and connected components are
	// merged if appropriate. If adding the edge to the graph creates a connected
	// component larger than the maximum allowable size then we simply create a
	// new connected component.
	void AddEdge(const T& node1, const T& node2) {
		Root* root1 = FindOrInsert(node1);
		Root* root2 = FindOrInsert(node2);

		// If the nodes are already part of the same connected component then do
		// nothing. If merging the connected components will create a connected
		// component larger than the max size then do nothing.
		if (root1->id == root2->id ||
			root1->size + root2->size > m_max_connected_component_size) {
			return;
		}

		// Union the two connected components. Balance the tree better by attaching
		// the smaller tree to the larger one.
		if (root1->size < root2->size) {
			root2->size += root1->size;
			*root1 = *root2;
		}
		else {
			root1->size += root2->size;
			*root2 = *root1;
		}
	}

	// Computes the connected components and returns the disjointed sets.
	void Extract(
		std::unordered_map<T, std::unordered_set<T> >* connected_components) {
		CHECK_NOTNULL(connected_components)->clear();

		for (const auto& node : m_disjoint_set) {
			const Root* root = FindRoot(node.first);
			(*connected_components)[root->id].insert(node.first);
		}
	}

	// Returns true if both nodes are in the same connected component and false
	// otherwise.
	bool NodesInSameConnectedComponent(const T& node1, const T& node2) {
		if (!ContainsKey(m_disjoint_set, node1) ||
			!ContainsKey(m_disjoint_set, node2)) {
			return false;
		}

		const Root* root1 = FindRoot(node1);
		const Root* root2 = FindRoot(node2);
		return root1->id == root2->id;
	}

private:
	// Attempts to find the root of the tree, or otherwise inserts the node.
	Root* FindOrInsert(const T& node) {
		const Root* parent = FindOrNull(m_disjoint_set, node);
		// If we cannot find the node in the disjoint set list, insert it.
		if (parent == nullptr) {
			InsertOrDie(&m_disjoint_set, node, Root(node, 1));
			return FindOrNull(m_disjoint_set, node);
		}

		return FindRoot(node);
	}

	// Perform a recursive search to find the root of the node. We flatten the
	// tree structure as we proceed so that finding the root is always a few
	// (hopefully one) steps away.
	Root* FindRoot(const T& node) {
		Root* parent = CHECK_NOTNULL(FindOrNull(m_disjoint_set, node));

		// If this node is a root, return the node itself.
		if (node == parent->id) {
			return parent;
		}

		// Otherwise, recusively search for the root.
		Root* root = FindRoot(parent->id);
		*parent = *root;
		return root;
	}

private:
	uint64_t m_max_connected_component_size;

	// Each node is mapped to a Root node. If the node is equal to the root id
	// then the node is a root and the size of the root is the size of the
	// connected component.
	std::unordered_map<T, Root> m_disjoint_set;
//	std::map<T, Root> m_disjoint_set;
};


#endif  // __CONNECTED_COMPONENTS_H__
