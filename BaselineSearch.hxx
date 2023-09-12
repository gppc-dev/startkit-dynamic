#ifndef OPT_GPPC_BASELINE_SEARCH_HXX
#define OPT_GPPC_BASELINE_SEARCH_HXX

#include <vector>
#include <utility>
#include <limits>
#include <queue>
#include <cstdint>

namespace baseline
{

using std::uint32_t;
using std::size_t;

constexpr uint32_t COST_0 = 1'000;
constexpr uint32_t COST_1 = 1'414;
using Point = std::pair<int,int>;
struct Node
{
	static constexpr uint32_t INV = std::numeric_limits<uint32_t>::max();
	// Node() noexcept : pred(INV), cost(INV)
	// { }
	uint32_t pred;
	uint32_t cost;
	bool open() const noexcept { return cost != INV; }
};
struct Grid
{


	uint32_t width;
	uint32_t height;
	const std::vector<bool>* cells;
	std::vector<Node> nodes;
	size_t size() const noexcept { return cells->size(); }
};

void dijkstra(Grid& grid, uint32_t origin)
{
	struct Qcmp
	{
		bool operator()(int a, int b) const {
			return grid->nodes[b].cost > grid->nodes[a].cost;
		}
		const Grid* grid;
	};
	std::priority_queue<uint32_t, std::vector<uint32_t>, Qcmp> Q(Qcmp{&grid});
	Q.push(origin);
	grid.nodes.assign(grid.size(), Node{Node::INV, Node::INV});
	grid.nodes[origin].cost = 0;
	while (!Q.empty()) {

	}
}

} // namespace baseline

#endif
