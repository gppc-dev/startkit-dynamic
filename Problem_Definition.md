# Problem statement
Your task is to find a path on a given graph, and the solution is evaluated based on optimality, time performance and space cost.

## Definition: Graph
The input graph is an 8-connected grid map, i.e. each cell (`c`) has 8 adjacent neighbors:

```
123
8c4
765
```
the distance to cardinal neighbors (`2,4,6,8`) is 1, the distance to diagonal neighbors (`1,3,5,7`) is `1.4141` (approximated `sqrt(2)`).

Each cell is either traversable or obstacle, **corner cutting is not allowed** , for example:
```
c..
.b.
a#.
```
`#` is an obstacle, `a,b,c` and `.` are traversable cells; `c` to `b` is a valid diagonal move while `a` to `b` is not.

## Definition: Path

For a query `(s, t)`, a valid path is a sequence of nodes `p=(s,v1,...vn,t)`, any adjacent nodes `(a, b)`on the path must be a **valid segment**, i.e. all moves from `a` to `b` must be in same direction, for example:

```
  01234567
0 ...b.d..
1 ........
2 ...c#...
3 a.......
```

* from `a` to `b` needs 3 diagonal moves (`Northeast`), so `(a, b)` is a valid segment;
* from `a` to `c` needs 1 diagonal move (`Northeast`) and 2 cardinal moves (`East`), so `(a, c)` is not a valid segment;
* from `c` to `d`, the first diagonal move from `c` is forbidden due to the **no corner-cutting** rule, so `(c, d)` is not a valid segment;

**When start and target are same node, the path must be empty, the length must be `0`.**
