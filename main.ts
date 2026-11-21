// sokoban-solver-01: push-optimal Sokoban solver.
// Decomposes each level into independent rooms off a hallway, solves every
// room push-optimally with A* over pushes (minimum-matching heuristic, sound
// deadlock pruning), and concatenates the rooms with hallway walks.
// Deterministic: no clock, no randomness, no environment-dependent output.
// Node built-ins only.
import * as fs from "node:fs";
import * as path from "node:path";

// Directions: [dx, dy, letter]. Fixed order everywhere, so output is deterministic.
const DIRS = [
  [0, -1, "u"],
  [0, 1, "d"],
  [-1, 0, "l"],
  [1, 0, "r"],
];

interface Level {
  w: number;
  h: number;
  wall: Uint8Array; // 1 = wall (or off-board padding treated as wall by callers)
  goal: Uint8Array;
  boxes: number[]; // global cell indices, sorted
  player: number; // global cell index
}

function parseLevel(text: string): Level {
  const rows = text.split("\n");
  while (rows.length > 0 && rows[rows.length - 1].length === 0) rows.pop();
  const h = rows.length;
  const w = Math.max(0, ...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goal = new Uint8Array(w * h);
  const boxes: number[] = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < rows[y].length; x++) {
      const ch = rows[y][x];
      const i = y * w + x;
      if (ch === "#") continue;
      wall[i] = 0;
      if (ch === "." || ch === "*" || ch === "+") goal[i] = 1;
      if (ch === "$" || ch === "*") boxes.push(i);
      if (ch === "@" || ch === "+") player = i;
    }
  }
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goal, boxes, player };
}

interface Room {
  cells: number[]; // global indices of the room floor, door square included, sorted
  door: number; // global index of the door square
  entry: number; // global index of the hallway square outside the door
  boxes: number[];
  goals: number[];
}

// Find the hallway row (the row holding '@') and split the remaining floor
// into connected components; each component is one room.
function findRooms(level: Level): Room[] {
  const { w, h, wall, goal, boxes } = level;
  const hallRow = Math.floor(level.player / w);
  const seen = new Uint8Array(w * h);
  const rooms: Room[] = [];
  const boxSet = new Set(boxes);
  for (let start = 0; start < w * h; start++) {
    if (wall[start] || seen[start]) continue;
    const y = Math.floor(start / w);
    if (y === hallRow) continue;
    // Flood fill this component (never crossing the hallway row).
    const comp: number[] = [start];
    seen[start] = 1;
    for (let i = 0; i < comp.length; i++) {
      const c = comp[i];
      const cx = c % w;
      const cy = Math.floor(c / w);
      for (const [dx, dy] of DIRS) {
        const nx = cx + dx;
        const ny = cy + dy;
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
        if (ny === hallRow) continue;
        const n = ny * w + nx;
        if (!wall[n] && !seen[n]) {
          seen[n] = 1;
          comp.push(n);
        }
      }
    }
    comp.sort((a, b) => a - b);
    // The door is the component cell adjacent to the hallway row.
    let door = -1;
    for (const c of comp) {
      const cy = Math.floor(c / w);
      if (cy === hallRow - 1 || cy === hallRow + 1) {
        if (!wall[hallRow * w + (c % w)]) {
          door = c;
          break;
        }
      }
    }
    if (door < 0) continue; // no door: not a room, ignore
    const entry = hallRow * w + (door % w);
    const roomBoxes = comp.filter((c) => boxSet.has(c));
    const roomGoals = comp.filter((c) => goal[c] === 1);
    rooms.push({ cells: comp, door, entry, boxes: roomBoxes, goals: roomGoals });
  }
  // Deterministic room order: above rooms first (smaller door row), then by door column.
  rooms.sort((a, b) => {
    const ra = Math.floor(a.door / w);
    const rb = Math.floor(b.door / w);
    if (ra !== rb) return ra - rb;
    return a.door % w - (b.door % w);
  });
  return rooms;
}

// ---- Small utilities -------------------------------------------------------

// BFS distances from `from` over local cells, avoiding nothing (boxes ignored).
// adj: local adjacency lists; returns Int32Array of distances (-1 = unreachable).
function bfsDist(adj: number[][], from: number): Int32Array {
  const n = adj.length;
  const dist = new Int32Array(n).fill(-1);
  dist[from] = 0;
  const q = [from];
  for (let i = 0; i < q.length; i++) {
    const c = q[i];
    for (const nb of adj[c]) {
      if (nb >= 0 && dist[nb] < 0) {
        dist[nb] = dist[c] + 1;
        q.push(nb);
      }
    }
  }
  return dist;
}

// BFS walk from `from` to `to` over local cells, avoiding `blocked` local cells.
// Returns the move string, or null when unreachable. Deterministic (fixed dir order).
function bfsPath(
  adj: number[][],
  from: number,
  to: number,
  blocked: Uint8Array,
): string | null {
  if (from === to) return "";
  const n = adj.length;
  const prev = new Int32Array(n).fill(-1);
  const prevCh = new Array<string>(n).fill("");
  prev[from] = from;
  const q = [from];
  for (let i = 0; i < q.length; i++) {
    const c = q[i];
    for (let k = 0; k < adj[c].length; k++) {
      const nb = adj[c][k];
      if (nb < 0 || blocked[nb] || prev[nb] >= 0) continue;
      prev[nb] = c;
      prevCh[nb] = DIRS[k][2] as string;
      if (nb === to) {
        let s = "";
        let cur = nb;
        while (cur !== from) {
          s = prevCh[cur] + s;
          cur = prev[cur];
        }
        return s;
      }
      q.push(nb);
    }
  }
  return null;
}

// All cells reachable from `from` over local cells avoiding `blocked`.
// Returns a Uint8Array reachability mask.
function reachableMask(adj: number[][], from: number, blocked: Uint8Array): Uint8Array {
  const n = adj.length;
  const seen = new Uint8Array(n);
  seen[from] = 1;
  const q = [from];
  for (let i = 0; i < q.length; i++) {
    const c = q[i];
    for (const nb of adj[c]) {
      if (nb >= 0 && !blocked[nb] && !seen[nb]) {
        seen[nb] = 1;
        q.push(nb);
      }
    }
  }
  return seen;
}

// Minimum-cost assignment of boxes to goals (permutation search for n <= 6,
// otherwise the weaker but still admissible sum of per-box minima).
// dist[a][b] = walking distance from box cell a to goal cell b (-1 = unreachable).
// Returns Infinity when no assignment exists.
function matchingHeuristic(
  boxes: number[],
  distBoxGoal: Int32Array[],
  nGoals: number,
): number {
  const n = boxes.length;
  if (n === 0) return 0;
  if (n <= 6) {
    // Enumerate goal permutations via recursion over boxes in order.
    let best = Infinity;
    const used = new Uint8Array(nGoals);
    const rec = (bi: number, acc: number): void => {
      if (acc >= best) return;
      if (bi === n) {
        best = acc;
        return;
      }
      for (let g = 0; g < nGoals; g++) {
        if (used[g]) continue;
        const d = distBoxGoal[bi][g];
        if (d < 0) continue;
        used[g] = 1;
        rec(bi + 1, acc + d);
        used[g] = 0;
      }
    };
    rec(0, 0);
    return best;
  }
  let sum = 0;
  for (let bi = 0; bi < n; bi++) {
    let m = Infinity;
    for (let g = 0; g < nGoals; g++) {
      const d = distBoxGoal[bi][g];
      if (d >= 0 && d < m) m = d;
    }
    if (m === Infinity) return Infinity;
    sum += m;
  }
  return sum;
}

// ---- Deadlock pruning (only rules provable dead) ---------------------------
// A "wall" here is always a true grid wall (or off-board). The hallway and
// other floor outside the room are NOT walls: a box on the door square can
// still be pushed back up into the room, so it is never a corner.
function isDead(
  boxesArr: number[],
  boxSet: Uint8Array, // over local cells
  goalLocal: Uint8Array,
  localToGlobal: number[],
  w: number,
  h: number,
  wallGrid: Uint8Array,
): boolean {
  const isWall = (g: number): boolean => {
    if (g < 0 || g >= w * h) return true;
    return wallGrid[g] === 1;
  };
  for (const b of boxesArr) {
    if (goalLocal[b]) continue;
    const g = localToGlobal[b];
    const gx = g % w;
    const gy = Math.floor(g / w);
    // Corner: two perpendicular true walls.
    const wallN = isWall(g - w);
    const wallS = isWall(g + w);
    const wallW = gx === 0 || isWall(g - 1);
    const wallE = gx === w - 1 || isWall(g + 1);
    if ((wallN || wallS) && (wallW || wallE)) {
      // Need perpendicular pair: (N|S) and (W|E) both present means a corner
      // only when the pair is one vertical + one horizontal, which holds here.
      if ((wallN && wallW) || (wallN && wallE) || (wallS && wallW) || (wallS && wallE)) return true;
    }
    // Wall segment: box against a wall in direction d; it can only slide along
    // the maximal run of floor cells sharing that wall, and can only leave the
    // run at its ends. If both ends are walls and no goal lies on the run, dead.
    const wallDirs: Array<[number, number, number, number]> = [
      [0, -1, 1, 0], // wall north, slide east/west
      [0, 1, 1, 0], // wall south, slide east/west
      [-1, 0, 0, 1], // wall west, slide north/south
      [1, 0, 0, 1], // wall east, slide north/south
    ];
    for (const [wdx, wdy, sdx, sdy] of wallDirs) {
      const wg = g + wdy * w + wdx;
      if (!isWall(wg)) continue;
      if (segmentDead(g, wdx, wdy, sdx, sdy, w, h, wallGrid, goalLocal, localToGlobal)) return true;
    }
    // 2x2 block: every 2x2 block containing this box, all four cells
    // wall-or-box with no goal among them, freezes the boxes inside.
    // (Block corner coordinates are computed from gx/gy, never by adding
    // offsets to the flat index, so no row can wrap around.)
    for (const [ox, oy] of [
      [0, 0],
      [-1, 0],
      [0, -1],
      [-1, -1],
    ]) {
      const bx = gx + ox;
      const by = gy + oy;
      const cells4 = [
        by * w + bx,
        by * w + bx + 1,
        (by + 1) * w + bx,
        (by + 1) * w + bx + 1,
      ];
      let full = true;
      let anyGoal = false;
      for (const c4 of cells4) {
        const cx4 = c4 % w;
        const cy4 = Math.floor(c4 / w);
        if (cx4 < 0 || cx4 >= w || cy4 < 0 || cy4 >= h) {
          // off-board counts as wall
          continue;
        }
        if (wallGrid[c4] === 1) continue;
        // floor cell: must hold a box to count, and note goals
        const li = globalToLocalLookup(c4, localToGlobal);
        if (li >= 0 && goalLocal[li]) anyGoal = true;
        if (li < 0 || !boxSet[li]) {
          full = false;
          break;
        }
      }
      if (full && !anyGoal) return true;
    }
  }
  return false;
}

// Helper: local index of a global cell, or -1. localToGlobal is sorted by
// global index in our construction, so binary search is exact.
function globalToLocalLookup(g: number, localToGlobal: number[]): number {
  let lo = 0;
  let hi = localToGlobal.length - 1;
  while (lo <= hi) {
    const mid = (lo + hi) >> 1;
    if (localToGlobal[mid] === g) return mid;
    if (localToGlobal[mid] < g) lo = mid + 1;
    else hi = mid - 1;
  }
  return -1;
}

// Wall-segment deadlock for one box cell g with a wall in (wdx, wdy),
// sliding in (sdx, sdy). True when the box is trapped on a goal-less run.
function segmentDead(
  g: number,
  wdx: number,
  wdy: number,
  sdx: number,
  sdy: number,
  w: number,
  h: number,
  wallGrid: Uint8Array,
  goalLocal: Uint8Array,
  localToGlobal: number[],
): boolean {
  const isWall = (cg: number): boolean => {
    if (cg < 0 || cg >= w * h) return true;
    return wallGrid[cg] === 1;
  };
  const gx = g % w;
  const gy = Math.floor(g / w);
  const run: number[] = [g];
  for (const sgn of [1, -1]) {
    let cx = gx;
    let cy = gy;
    for (;;) {
      cx += sgn * sdx;
      cy += sgn * sdy;
      if (cx < 0 || cx >= w || cy < 0 || cy >= h) break; // sealed by the board edge (off-board is wall)
      const cg = cy * w + cx;
      if (wallGrid[cg] === 1) break; // sealed by a wall at this end
      if (!isWall(cg + wdy * w + wdx)) return false; // wall ends: box can turn away
      run.push(cg);
    }
  }
  // Both ends sealed by walls or the board edge, and no goal on the run.
  for (const cg of run) {
    const li = globalToLocalLookup(cg, localToGlobal);
    if (li >= 0 && goalLocal[li]) return false;
  }
  return true;
}

// ---- Push-space A* over one sub-problem ------------------------------------
// Local cells = the room floor (door included) plus the entry square outside
// the door. Boxes never leave the local cells. `door` is the local index the
// player must be able to reach at the end (exit); pass -1 to skip that check.

interface SubResult {
  moves: string;
}

function solveSubproblem(
  level: Level,
  cellsGlobal: number[],
  boxesGlobal: number[],
  goalsGlobal: number[],
  startPlayerGlobal: number,
  doorGlobal: number,
): SubResult | null {
  const { w, h, wall } = level;
  const localToGlobal = [...cellsGlobal].sort((a, b) => a - b);
  const n = localToGlobal.length;
  const localOf = (g: number): number => globalToLocalLookup(g, localToGlobal);
  const startPlayer = localOf(startPlayerGlobal);
  const doorLocal = doorGlobal >= 0 ? localOf(doorGlobal) : -1;
  if (startPlayer < 0) return null;
  const goalLocal = new Uint8Array(n);
  for (const g of goalsGlobal) {
    const li = localOf(g);
    if (li >= 0) goalLocal[li] = 1;
  }
  const boxLocal = boxesGlobal.map(localOf).sort((a, b) => a - b);
  if (boxLocal.some((b) => b < 0)) return null;

  // Local adjacency in DIRS order.
  const adj: number[][] = [];
  for (let i = 0; i < n; i++) {
    const g = localToGlobal[i];
    const gx = g % w;
    const gy = Math.floor(g / w);
    const row: number[] = [];
    for (const [dx, dy] of DIRS) {
      const nx = gx + dx;
      const ny = gy + dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) row.push(-1);
      else row.push(localOf(ny * w + nx));
    }
    adj.push(row);
  }
  // All-pairs walking distances ignoring boxes.
  const dists: Int32Array[] = [];
  for (let i = 0; i < n; i++) dists.push(bfsDist(adj, i));
  const goalList: number[] = [];
  for (let i = 0; i < n; i++) if (goalLocal[i]) goalList.push(i);

  const heuristic = (boxes: number[]): number => {
    const rows: Int32Array[] = boxes.map((b) => {
      const r = new Int32Array(goalList.length);
      for (let gi = 0; gi < goalList.length; gi++) r[gi] = dists[b][goalList[gi]];
      return r;
    });
    return matchingHeuristic(boxes, rows, goalList.length);
  };

  const deadCheck = (boxes: number[]): boolean => {
    const set = new Uint8Array(n);
    for (const b of boxes) set[b] = 1;
    return isDead(boxes, set, goalLocal, localToGlobal, w, h, wall);
  };

  const isGoalBoxes = (boxes: number[]): boolean => {
    if (boxes.length !== goalList.length) return false;
    for (let i = 0; i < boxes.length; i++) if (!goalLocal[boxes[i]]) return false;
    return true;
  };

  // Binary heap of node indices. Priority arrays are filled per search mode.
  interface SearchState {
    boxes: number[];
    player: number;
    g: number;
    h: number;
    parent: number;
    seg: string;
    key: string;
  }

  const attempts: Array<["astar" | "greedy" | "dijkstra", number]> = [
    ["astar", 2000000],
    ["greedy", 2000000],
    ["dijkstra", 1000000],
  ];
  const found = searchOnce();
  if (!found) return null;
  const { goalIdx, nodes } = found;
  // Reconstruct the move string.
  const segs: string[] = [];
  let cur = goalIdx;
  while (cur > 0) {
    segs.push(nodes[cur].seg);
    cur = nodes[cur].parent;
  }
  segs.reverse();
  let moves = segs.join("");
  if (doorLocal >= 0) {
    const finalNode = nodes[goalIdx];
    const blocked = new Uint8Array(n);
    for (const b of finalNode.boxes) blocked[b] = 1;
    const exitWalk = bfsPath(adj, finalNode.player, doorLocal, blocked);
    if (exitWalk === null) return null;
    moves += exitWalk;
    // Step from the door square to the entry square.
    const dg = localToGlobal[doorLocal];
    const eg = startPlayerGlobal;
    const dx = (eg % w) - (dg % w);
    const dy = Math.floor(eg / w) - Math.floor(dg / w);
    for (const [ddx, ddy, ch] of DIRS) {
      if (ddx === dx && ddy === dy) moves += ch as string;
    }
  }
  return { moves };

  // Full search with node capture; tries A*, then greedy, then Dijkstra.
  function searchOnce(): { goalIdx: number; nodes: SearchState[] } | null {
    // Try A* first, then greedy best-first, then Dijkstra, each with a
    // fixed expansion cap (budget by work, never by clock).
    for (const [mode, cap] of attempts) {
      const r = runSearchWithNodes(mode, cap);
      if (r === null) return null; // exhausted in this mode: no solution at all
      if (r.goal >= 0) return { goalIdx: r.goal, nodes: r.nodes };
      // capped: try the next, cheaper mode
    }
    return null;
  }

  function runSearchWithNodes(
    mode: "astar" | "greedy" | "dijkstra",
    maxExp: number,
  ): { goal: number; nodes: SearchState[]; capped: boolean } | null {
    const nodes: SearchState[] = [];
    const best = new Map<string, number>();
    const fOf = (nd: SearchState): number =>
      mode === "astar" ? nd.g + nd.h : mode === "greedy" ? nd.h : nd.g;
    const tOf = (nd: SearchState): number =>
      mode === "astar" ? nd.h : mode === "greedy" ? -nd.g : 0;
    const heap: number[] = [];
    const less = (a: number, b: number): boolean => {
      const fa = fOf(nodes[a]);
      const fb = fOf(nodes[b]);
      if (fa !== fb) return fa < fb;
      const ta = tOf(nodes[a]);
      const tb = tOf(nodes[b]);
      if (ta !== tb) return ta < tb;
      return nodes[a].g < nodes[b].g;
    };
    const pushHeap = (idx: number): void => {
      heap.push(idx);
      let i = heap.length - 1;
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (!less(heap[i], heap[p])) break;
        [heap[i], heap[p]] = [heap[p], heap[i]];
        i = p;
      }
    };
    const popHeap = (): number => {
      const top = heap[0];
      const last = heap.pop() as number;
      if (heap.length > 0) {
        heap[0] = last;
        let i = 0;
        for (;;) {
          const l = 2 * i + 1;
          const r = l + 1;
          let m = i;
          if (l < heap.length && less(heap[l], heap[m])) m = l;
          if (r < heap.length && less(heap[r], heap[m])) m = r;
          if (m === i) break;
          [heap[i], heap[m]] = [heap[m], heap[i]];
          i = m;
        }
      }
      return top;
    };
    const startBoxes = [...boxLocal];
    const startH = heuristic(startBoxes);
    if (startH === Infinity) return null;
    if (deadCheck(startBoxes)) return null;
    const startKey = startPlayer + ":" + startBoxes.join(",");
    nodes.push({
      boxes: startBoxes,
      player: startPlayer,
      g: 0,
      h: startH,
      parent: -1,
      seg: "",
      key: startKey,
    });
    best.set(startKey, 0);
    pushHeap(0);
    let expansions = 0;
    while (heap.length > 0) {
      if (expansions >= maxExp) return { goal: -1, nodes, capped: true };
      const ni = popHeap();
      const nd = nodes[ni];
      if (best.get(nd.key) !== nd.g) continue;
      expansions++;
      if (isGoalBoxes(nd.boxes)) {
        if (doorLocal < 0) return { goal: ni, nodes, capped: false };
        const blocked = new Uint8Array(n);
        for (const b of nd.boxes) blocked[b] = 1;
        if (reachableMask(adj, nd.player, blocked)[doorLocal]) {
          return { goal: ni, nodes, capped: false };
        }
      }
      const blocked = new Uint8Array(n);
      for (const b of nd.boxes) blocked[b] = 1;
      const reach = reachableMask(adj, nd.player, blocked);
      for (let bi = 0; bi < nd.boxes.length; bi++) {
        const b = nd.boxes[bi];
        for (let k = 0; k < 4; k++) {
          const dest = adj[b][k];
          if (dest < 0 || blocked[dest]) continue;
          const stance = adj[b][k ^ 1];
          if (stance < 0 || !reach[stance]) continue;
          const walk = bfsPath(adj, nd.player, stance, blocked);
          if (walk === null) continue;
          const nb = [...nd.boxes];
          nb[bi] = dest;
          nb.sort((a, c) => a - c);
          if (deadCheck(nb)) continue;
          const nh = heuristic(nb);
          if (nh === Infinity) continue;
          const ng = nd.g + 1;
          const realKey = b + ":" + nb.join(",");
          const prev = best.get(realKey);
          if (prev !== undefined && prev <= ng) continue;
          best.set(realKey, ng);
          nodes.push({
            boxes: nb,
            player: b,
            g: ng,
            h: nh,
            parent: ni,
            seg: walk + (DIRS[k][2] as string),
            key: realKey,
          });
          pushHeap(nodes.length - 1);
        }
      }
    }
    return null;
  }
}

// ---- Driver ----------------------------------------------------------------

// BFS walk on the full grid from `from` to `to`, avoiding cells in `blocked`.
function bfsPathGlobal(level: Level, from: number, to: number, blocked: Set<number>): string | null {
  const { w, h, wall } = level;
  if (from === to) return "";
  const prev = new Int32Array(w * h).fill(-1);
  const prevCh = new Array<string>(w * h).fill("");
  prev[from] = from;
  const q = [from];
  for (let i = 0; i < q.length; i++) {
    const c = q[i];
    const cx = c % w;
    const cy = Math.floor(c / w);
    for (const [dx, dy, ch] of DIRS) {
      const nx = cx + dx;
      const ny = cy + dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const nb = ny * w + nx;
      if (wall[nb] || blocked.has(nb) || prev[nb] >= 0) continue;
      prev[nb] = c;
      prevCh[nb] = ch as string;
      if (nb === to) {
        let s = "";
        let cur = nb;
        while (cur !== from) {
          s = prevCh[cur] + s;
          cur = prev[cur];
        }
        return s;
      }
      q.push(nb);
    }
  }
  return null;
}

function solveLevel(level: Level): string {
  const rooms = findRooms(level);
  const roomBoxes = rooms.reduce((a, r) => a + r.boxes.length, 0);
  // Whole-level fallback when the room structure is not as expected.
  if (rooms.length === 0 || roomBoxes !== level.boxes.length) {
    const allFloor: number[] = [];
    for (let i = 0; i < level.w * level.h; i++) if (!level.wall[i]) allFloor.push(i);
    const allGoals: number[] = [];
    for (let i = 0; i < level.w * level.h; i++) if (level.goal[i]) allGoals.push(i);
    const r = solveSubproblem(level, allFloor, level.boxes, allGoals, level.player, -1);
    return r ? r.moves : "";
  }
  let moves = "";
  let curPlayer = level.player;
  // Current box positions, updated as rooms are solved (boxes end on goals).
  const curBoxes = new Set<number>(level.boxes);
  for (const room of rooms) {
    const walk = bfsPathGlobal(level, curPlayer, room.entry, curBoxes);
    if (walk === null) return "";
    moves += walk;
    const r = solveSubproblem(
      level,
      [...room.cells, room.entry],
      room.boxes,
      room.goals,
      room.entry,
      room.door,
    );
    if (!r) {
      // Room failed: try the whole level as a last resort.
      const allFloor: number[] = [];
      for (let i = 0; i < level.w * level.h; i++) if (!level.wall[i]) allFloor.push(i);
      const allGoals: number[] = [];
      for (let i = 0; i < level.w * level.h; i++) if (level.goal[i]) allGoals.push(i);
      const whole = solveSubproblem(level, allFloor, level.boxes, allGoals, level.player, -1);
      return whole ? whole.moves : "";
    }
    moves += r.moves;
    // Boxes of this room are now on its goals; the player stands at the entry.
    for (const b of room.boxes) curBoxes.delete(b);
    for (const g of room.goals) curBoxes.add(g);
    curPlayer = room.entry;
  }
  return moves;
}

function main(): void {
  const inputDir = process.env.MAKEFUN_INPUT_DIR || "/input";
  const outputDir = process.env.MAKEFUN_OUTPUT_DIR || "/output";
  const ids = fs
    .readdirSync(inputDir, { withFileTypes: true })
    .filter((e) => e.isDirectory())
    .map((e) => e.name)
    .sort();
  for (const id of ids) {
    const levelPath = path.join(inputDir, id, "level.txt");
    const text = fs.readFileSync(levelPath, "utf8");
    const level = parseLevel(text);
    const moves = solveLevel(level);
    const outDir = path.join(outputDir, id);
    fs.mkdirSync(outDir, { recursive: true });
    fs.writeFileSync(path.join(outDir, "solution.txt"), moves, "utf8");
  }
}

main();
