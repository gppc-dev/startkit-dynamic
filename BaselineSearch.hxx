#ifndef OPT_GPPC_BASELINE_SEARCH_HXX
#define OPT_GPPC_BASELINE_SEARCH_HXX

#include <vector>
#include <utility>
#include <limits>
#include <queue>
#include <stack>
#include <array>
#include <memory_resource>
#include <algorithm>
#include <cstdint>
#include <cassert>
#include "Entry.h"

namespace baseline
{

using std::uint32_t;
using std::size_t;

constexpr uint32_t COST_0 = 1000; // 1.0
constexpr uint32_t COST_1 = 1414; // 1.414
using Point = std::pair<int,int>;
struct Node
{
	static constexpr uint32_t INV = std::numeric_limits<uint32_t>::max();
	static constexpr uint32_t FLOOD_FILL = INV-1;
	static constexpr uint32_t NO_PRED = INV-2;
	// Node() noexcept : pred(INV), cost(INV)
	// { }
	uint32_t pred;
	uint32_t cost;
};
struct Grid
{
	size_t size() const noexcept { return cells_size; }
	uint32_t pack(Point p) const noexcept
	{
		assert(static_cast<uint32_t>(p.first) < width && static_cast<uint32_t>(p.second) < height);
		return static_cast<uint32_t>(p.second) * width + static_cast<uint32_t>(p.first);
	}
	Point unpack(uint32_t p) const noexcept
	{
		assert(width != 0 && p < size());
		return Point(static_cast<int>(p % width), static_cast<int>(p / width));
	}
	bool get_unbound(uint32_t p) const noexcept
	{
		return gppc_patch_get(cells, static_cast<int>(p));
	}
	bool get(uint32_t p) const noexcept
	{
		return p < size() && gppc_patch_get(cells, static_cast<int>(p));
	}
	bool get(Point p) const noexcept
	{
		return static_cast<uint32_t>(p.first) < width
		    && static_cast<uint32_t>(p.second) < height
			&& gppc_patch_get_xy(cells, p.first, p.second);
	}

	Grid(gppc_patch map) :
		 width(static_cast<uint32_t>(map.width))
		,height(static_cast<uint32_t>(map.height))
		,cells(map)
		,cells_size(width * height)
	{ }

	uint32_t width;
	uint32_t height;
	gppc_patch cells;
	uint32_t cells_size;
	std::vector<Node> nodes;
};

void path_to_root(const Grid& grid, Point start, std::vector<Point>& out);
void setup_grid(Grid& grid);

struct SpanningTreeSearch : Grid
{
	SpanningTreeSearch(gppc_patch map) : Grid(map)
	{
		update_grid();
	}
	void update_grid()
	{
		setup_grid(*this);
	}
	std::array<std::vector<gppc_point>, 2> path_parts;
	const std::vector<gppc_point>& get_path() const noexcept { return path_parts[0]; }
	// bool search found a path
	bool search(Point s, Point g)
	{
		auto&& push_back = [](std::vector<gppc_point>& path, Point p) {
			path.push_back(gppc_point{static_cast<uint16_t>(p.first), static_cast<uint16_t>(p.second)});
		};
		std::array<uint32_t, 2> nodeid{{pack(s), pack(g)}};
		if (nodes[nodeid[0]].pred == Node::INV || nodes[nodeid[1]].pred == Node::INV)
			return false;
		if (nodeid[0] == nodeid[1]) {
			// zero path case
			path_parts[0].assign({gppc_point{(uint16_t)s.first, (uint16_t)s.second}, 
				gppc_point{(uint16_t)g.first, (uint16_t)g.second}});
			return true;
		}
		path_parts[0].clear(); path_parts[1].clear();
		while (true) {
			int progressId = 0;
			auto c0 = nodes[nodeid[0]].cost, c1 = nodes[nodeid[1]].cost; 
			if (c0 == c1) {
				// same dist, check if same root
				if (nodeid[0] == nodeid[1]) {
					push_back(path_parts[0], unpack(nodeid[static_cast<uint32_t>(progressId)]));
					break; // found least common ancestor
				}
				if (c0 == 0)
					return false; // tree root's are different, no path
			} else if (c1 > c0) {
				progressId = 1; // nodeid[1] is longer thus process it first
			}
			push_back(path_parts[progressId], unpack(nodeid[progressId]));
			nodeid[progressId] = nodes[nodeid[progressId]].pred;
		}
		path_parts[0].insert(path_parts[0].end(), path_parts[1].rbegin(), path_parts[1].rend());
		return true;
	}
};

void flood_fill(Grid& grid, std::vector<Point>& out, uint32_t origin)
{
	assert(origin < grid.nodes.size() && grid.nodes[origin].pred == Node::INV);
	out.clear();
	std::stack<Point, std::vector<Point>> Q;
	auto&& push_queue = [&grid,&Q] (Point p, int dx, int dy) {
		p.first += dx; p.second += dy;
		if (grid.get(p)) {
			auto& node = grid.nodes[grid.pack(p)];
			if (node.pred == Node::INV) {
				node.pred = Node::FLOOD_FILL;
				Q.push(p);
			}
		}
	};
	grid.nodes[origin].pred = Node::FLOOD_FILL;
	Q.push(grid.unpack(origin));
	while (!Q.empty())
	{
		Point p = Q.top(); Q.pop();
		out.push_back(p);
		push_queue(p, 1, 0);
		push_queue(p, -1, 0);
		push_queue(p, 0, 1);
		push_queue(p, 0, -1);
	}
}

enum class Compass : uint32_t
{
	     //222111000
	N =  0b000000010,
	E =  0b000100000,
	S =  0b010000000,
	W =  0b000001000,
	NE = 0b000000100 | N | E,
	NW = 0b000000001 | N | W,
	SE = 0b100000000 | S | E,
	SW = 0b001000000 | S | W,
};
void dijkstra(Grid& grid, uint32_t origin)
{
	// first = dist, second = node-id
	using node_type = std::pair<uint32_t,uint32_t>;
	std::priority_queue<node_type, std::vector<node_type>, std::greater<node_type>> Q;
	auto try_push = [&grid,&Q](uint32_t node, int dx, int dy, uint32_t cost) {
		uint32_t newNode = static_cast<uint32_t>( static_cast<int>(node) + dy * grid.width + dx );
		Node& N = grid.nodes[newNode];
		if (cost < N.cost) {
			N.pred = node;
			N.cost = cost;
			Q.emplace(cost, newNode);
		}
	};
	Q.emplace(0, origin);
	grid.nodes[origin].cost = 0;
	grid.nodes[origin].pred = Node::NO_PRED;
	while (!Q.empty()) {
		auto node_value = Q.top(); Q.pop();
		auto cost = node_value.first;
		auto node = node_value.second; 
		if (cost != grid.nodes[node].cost)
			continue; // skip
		Point p = grid.unpack(node);
		// push successors
		uint32_t mask = 0;
		for (int i = 0, dy = -1; dy < 2; dy++)
		for (int dx = -1; dx < 2; dx++) {
			mask |= static_cast<uint32_t>(grid.get( Point(p.first + dx, p.second + dy) )) << i++;
		}
		mask = ~mask; // 1 = non-trav, 0 = trav

		// 012
		// 345
		// 678
		// N
		if ( (mask & static_cast<uint32_t>(Compass::N)) == 0 ) try_push(node, 0, -1, cost + COST_0);
		// E
		if ( (mask & static_cast<uint32_t>(Compass::E)) == 0 ) try_push(node, 1, 0, cost + COST_0);
		// S
		if ( (mask & static_cast<uint32_t>(Compass::S)) == 0 ) try_push(node, 0, 1, cost + COST_0);
		// W
		if ( (mask & static_cast<uint32_t>(Compass::W)) == 0 ) try_push(node, -1, 0, cost + COST_0);
		// NE
		if ( (mask & static_cast<uint32_t>(Compass::NE)) == 0 ) try_push(node, 1, -1, cost + COST_1);
		// NW
		if ( (mask & static_cast<uint32_t>(Compass::NW)) == 0 ) try_push(node, -1, -1, cost + COST_1);
		// SE
		if ( (mask & static_cast<uint32_t>(Compass::SE)) == 0 ) try_push(node, 1, 1, cost + COST_1);
		// SW
		if ( (mask & static_cast<uint32_t>(Compass::SW)) == 0 ) try_push(node, -1, 1, cost + COST_1);
	}
}

void setup_grid(Grid& grid)
{
	grid.nodes.assign(grid.size(), Node{Node::INV, Node::INV});
	std::vector<Point> cluster;
	struct Dist {
		bool operator()(Point q, Point p) const noexcept {
			return dist(q, centre) < dist(p, centre);
		}
		static int dist(Point q, Point p) noexcept {
			return std::abs(q.first - p.first) + std::abs(q.second - p.second);
		}
		Point centre;
	};
	for (uint32_t i = 0, ie = grid.size(); i < ie; ++i) {
		if (grid.get_unbound(i) && grid.nodes[i].pred == Node::INV) {
			// new cluster
			flood_fill(grid, cluster, i);
			assert(!cluster.empty());
			std::uint64_t sumx = 0, sumy = 0;
			for (Point p : cluster) {
				sumx += p.first; sumy += p.second;
			}
			Point cluster_centre(static_cast<int>(sumx / cluster.size()), static_cast<int>(sumy / cluster.size()));
			uint32_t cluster_id = grid.pack( *std::min_element(cluster.begin(), cluster.end(), Dist{cluster_centre}) );
			dijkstra(grid, cluster_id);
			assert(std::all_of(cluster.begin(), cluster.end(), [&grid] (Point q) { return grid.nodes.at(grid.pack(q)).pred != Node::FLOOD_FILL; }));
		}
	}
}

} // namespace baseline

#endif
