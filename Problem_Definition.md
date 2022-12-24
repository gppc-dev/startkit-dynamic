# Problem statement
Your task is to find a path on a given graph, and the solution is evaluated based on optimality, time performance and space cost.

## Definition: Graph
The input graph is an 8-connected grid map, i.e. each cell (`c`) has 8 adjacent neighbors:

<img src="./figs/neigb-1.png" width="128" height="128">

the distance to cardinal neighbors (`N,S,W,E`) is 1, the distance to diagonal neighbors (`NE,NW,SW,SE`) is `1.4141` (approximated `sqrt(2)`).

Each cell is either traversable or obstacle, **corner cutting is not allowed**:

<img src="./figs/cornercut-1.png" width="128" height="128">

## Definition: Path

For a query `(s, t)`, a valid path is a sequence of nodes `p=(s,v1,...vn,t)`, any adjacent nodes `(a, b)`on the path must be a **valid segment**, i.e. all moves from `a` to `b` must be in same direction, for example:

<img src="./figs/exp-1.png" width="512" height="256">

* from `a` to `b` needs 3 diagonal moves (`Northeast`), so `(a, b)` is a valid segment;
* from `a` to `c` needs 1 diagonal move (`Northeast`) and 2 cardinal moves (`East`), so `(a, c)` is not a valid segment;
* from `c` to `d`, the first diagonal move from `c` is forbidden due to the **no corner-cutting** rule, so `(c, d)` is not a valid segment;

**When start and target are same node, the path must be empty, the length must be `0`.**
