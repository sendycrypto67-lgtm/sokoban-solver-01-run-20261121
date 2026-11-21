# Build log — sokoban-solver-01-run-20261121

## What was built

`main.ts`: a push-optimal Sokoban solver, written from scratch (Node built-ins only,
erasable TypeScript so Node 22 strips types natively).

- Parses each level, finds the hallway row (the row holding `@`), and splits the
  remaining floor into connected components: one room per component, door square
  included (handles a door square that starts out holding a box).
- Solves every room independently with push-space A*: states are player position
  plus sorted box positions, expanded by pushes with player reachability by BFS
  inside each state. Admissible heuristic: minimum-cost box-to-goal matching over
  precomputed BFS distances ignoring other boxes (exact assignment for 3-4 boxes).
- Deadlock pruning uses only provable rules: box in a corner of two true walls
  (not on a goal); box trapped on a wall segment sealed at both ends with no goal
  on it; 2x2 block of boxes/walls with no goal inside. Hallway floor is never
  treated as a wall, so a box on the door square is never falsely pruned.
- A room is a goal only when all its boxes are on goals AND the player can walk
  back to the door square. Rooms are joined by hallway walks; room order does not
  affect the push count.
- Budget by work, not clock: A* cap 2,000,000 expansions per room, then greedy
  best-first, then Dijkstra, each with a fixed cap. Whole-level A* fallback if
  room detection ever fails to cover the level. Deterministic: no clock, no
  randomness, no environment-dependent output; output files byte-identical
  across runs.

## Test results

### Practice set (kit checker + official scorer `score.ts`)

| instance | boxes | rooms | P* | solver pushes | checker |
|----------|-------|-------|----|---------------|---------|
| i001 | 3 | 1 | 13 | 13 | solved, 1000 pts |
| i002 | 4 | 1 | 16 | 16 | solved, 1000 pts |
| i003 | 8 | 2 | 29 | 29 | solved, 1000 pts |
| i004 | 12 | 3 | 47 | 47 | solved, 1000 pts |
| i005 | 16 | 4 | 60 | 60 | solved, 1000 pts |

Official scorer: `{"score":1000000,"passed":1,"answered":5,"instances":5}`.
Total wall time for the 5 practice instances: ~0.65 s (limit 60 s).

### Generated levels (own seeded generator: hallway + 1-4 rooms in 6x6 slots,
boxes on goals then 30-80 reverse pulls; rooms kept only when an independent
push-space BFS optimum is >= 12 pushes for 3 boxes / >= 14 for 4 boxes,
mirroring the real generator's keep rule)

- Easy batch (no minimum): 30/30 solved and replay-verified.
- Hard batch: 30/30 solved, every push count exactly equal to the independent
  Dijkstra optimum recorded at generation time (P* 12-18 per 1-room level,
  28-34 for 2 rooms, 43-46 for 3 rooms, 60-64 for 4 rooms; total 990 pushes).
  Spot-checked with a separate Dijkstra script: g001=14, g013=34, g025=64.
  All 30 solved in ~2.4 s total.
- Shapes covered: 1 room of 3, 1 room of 4, 2 rooms, 3 rooms, 4 rooms (6 each).

### Edge cases (hand-built, each replay-verified and cross-checked against an
independent exhaustive push-space BFS with no heuristic and no pruning)

| case | description | solver pushes | Dijkstra optimum |
|------|-------------|---------------|------------------|
| e1 | narrow room, 1-wide corridors | 7 | 7 |
| e2 | box starts on the door square | 8 | 8 |
| e3 | box starts in a corner on a goal; dead corner nearby | 4 | 4 |
| e4 | push order required (box A on its goal strands box B) | 4 | 4 |
| e5 | box must be pushed away from its goal first | 5 | 5 |
| e6 | box already on a goal at the start | 4 | 4 |
| e7 | goal tucked in a pocket reachable only from one side | 8 | 8 |

7/7 solved, all optimal. Three of the first hand-built drafts (e1, e5, e7) turned
out to be genuinely unsolvable by construction (frozen boxes / unreachable
stances, confirmed by the independent BFS exploring only 1-45 states); they
were redesigned until solvable, not dropped, and the solver correctly returned
an empty (valid, scored-0) answer for the unsolvable drafts instead of an
illegal solution.

### Determinism

Practice set run twice: 5 solution files byte-identical. Hard generated set run
twice: byte-identical. No `Date`, `Math.random`, or output-affecting environment
reads in `main.ts` (env vars only select input/output directories).

### Self-review (done before finishing)

Re-read `main.ts` end to end. Fixed during review: a 2x2 deadlock check that
could wrap around row ends (now computed from coordinates); a wall-segment
check that treated the board edge as a leak instead of sealed (now sealed,
still sound); a stale comment. Re-ran all tests after the fixes: practice
5/5 optimal, hard generated 30/30 optimal, edge 7/7 optimal.

## Known limitations

- The solver relies on the level structure (hallway + independent rooms). If a
  scored level broke that structure, the whole-level A* fallback would try to
  solve it directly; for 16 boxes that search is far too large to finish, so
  such a level would likely go unanswered rather than solved. This structure is
  guaranteed by the competition generator, so the fallback is only a safety net.
- Greedy/Dijkstra fallbacks are complete but not push-optimal; they only run if
  A* exhausts a 2,000,000-expansion cap, which never happened in any test.
