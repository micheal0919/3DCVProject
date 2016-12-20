#ifndef __CONNECTED_COMPONENTS_H__
#define __CONNECTED_COMPONENTS_H__

#include <stdint.h>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include <glog/logging.h>

#include "MapUtils.h"

template <typename T>
class CConnectedComponents {
public:
	struct Root {
		Root(const T& id, const int size) : id(id), size(size) {}
		T id;
		int size;
	};

	CConnectedComponents()
		: m_max_connected_component_size(std::numeric_limits<T>::max()) {}

	explicit CConnectedComponents(const int max_size)
		: m_max_connected_component_size(max_size) {
		CHECK_GT(m_max_connected_component_size, 0);
	}

	void AddEdge(const T& node1, const T& node2) {
		Root* root1 = FindOrInsert(node1);
		Root* root2 = FindOrInsert(node2);

		if (root1->id == root2->id ||
			root1->size + root2->size > m_max_connected_component_size) {
			return;
		}

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
	Root* FindOrInsert(const T& node) {
		const Root* parent = FindOrNull(m_disjoint_set, node);
		if (parent == nullptr) {
			InsertOrDie(&m_disjoint_set, node, Root(node, 1));
			return FindOrNull(m_disjoint_set, node);
		}

		return FindRoot(node);
	}

	Root* FindRoot(const T& node) {
		Root* parent = CHECK_NOTNULL(FindOrNull(m_disjoint_set, node));
		if (node == parent->id) {
			return parent;
		}
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
};


#endif  // __CONNECTED_COMPONENTS_H__
