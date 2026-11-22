// Room-wise push-optimal Sokoban solver.
//
// Reads /input/<id>/level.txt for every instance id found under /input
// (or $MAKEFUN_INPUT_DIR when the harness sets it; pass inputDir
// outputDir as argv for local testing) and writes
// /output/<id>/solution.txt (or $MAKEFUN_OUTPUT_DIR), the player's moves
// in LURD.
//
// Why rooms: a generated level is a hallway with one to four rooms off it,
// each room holding 3-4 boxes and as many goals, opening onto the hallway
// through one door. A box pushed through a door into the hallway can never
// be pushed back, so every box stays in its room and the level's
// push-optimal count is the sum of its rooms' optima. Each room is solved
// independently with push-space A* and an admissible heuristic, which makes
// the first solution found push-optimal for that room.
//
// Determinism: fixed iteration orders everywhere, fixed node budgets counted
// in expanded search nodes, never the wall clock, no randomness, no
// environment reads. Two runs on the same input produce byte-identical
// outputs.
//
// Usage: node main.ts [inputDir] [outputDir]
//   (grader runs: node /bot/main.ts, i.e. /input and /output)

import { mkdirSync, readFileSync, readdirSync, writeFileSync } from "node:fs";
import { join } from "node:path";

// ---------------------------------------------------------------------------
// Small utilities
// ---------------------------------------------------------------------------

const DIRS = [
  [0, -1], // u
  [0, 1], // d
  [-1, 0], // l
  [1, 0], // r
];
const DIRCH = ["u", "d", "l", "r"];

// Fixed work budgets (expanded search nodes), never time-based.
const ROOM_ASTAR_BUDGET = 200000;
const ROOM_GREEDY_BUDGET = 120000;
const GLOBAL_BUDGET = 1200000;

let expansionsTotal = 0;

function assert(cond, msg) {
  if (!cond) throw new Error(msg);
}

// ---------------------------------------------------------------------------
// Level parsing (XSB)
// ---------------------------------------------------------------------------

type Level = {
  w: number;
  h: number;
  wall: Uint8Array; // 1 = wall (or outside the board)
  goal: Uint8Array; // 1 = goal
  boxes: number[]; // sorted global cell indices
  player: number; // global cell index
};

function parseLevel(text: string): Level {
  const rows = text.split("\n").filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goal = new Uint8Array(w * h);
  const boxes: number[] = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    const row = rows[y];
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === "#") continue;
      wall[i] = 0;
      if (ch === "." || ch === "*" || ch === "+") goal[i] = 1;
      if (ch === "$" || ch === "*") boxes.push(i);
      if (ch === "@" || ch === "+") player = i;
    }
  }
  assert(player >= 0, "level has no player");
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goal, boxes, player };
}

// ---------------------------------------------------------------------------
// Room decomposition
// ---------------------------------------------------------------------------

type Room = {
  door: number; // global cell: the single cell joining room and hallway
  entry: number; // global cell: hallway neighbour of the door (player enters here)
  floor: number[]; // global cells where this room's boxes may be (room + door)
  playerFloor: number[]; // floor + entry (where the player may walk)
  boxes: number[]; // global cells of the room's boxes
  goals: number[]; // global cells of the room's goals
};

// Connected components of the floor graph after removing one cell.
// Returns an array of components, each a list of global cell indices.
function componentsWithout(lv: Level, removed: number): number[][] {
  const { w, h, wall } = lv;
  const seen = new Uint8Array(w * h);
  seen[removed] = 1;
  const comps: number[][] = [];
  for (let s = 0; s < w * h; s++) {
    if (wall[s] || seen[s]) continue;
    const comp: number[] = [];
    const stack = [s];
    seen[s] = 1;
    while (stack.length > 0) {
      const c = stack.pop() as number;
      comp.push(c);
      const x = c % w;
      const y = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = x + DIRS[d][0];
        const ny = y + DIRS[d][1];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const n = ny * w + nx;
        if (wall[n] || seen[n]) continue;
        seen[n] = 1;
        stack.push(n);
      }
    }
    comps.push(comp);
  }
  return comps;
}

function decompose(lv: Level): Room[] | null {
  const { w, wall, goal, boxes, player } = lv;
  const hallwayRow = (player / w) | 0;
  const inHallway = (c: number) => ((c / w) | 0) === hallwayRow && !wall[c];
  const boxSet = new Set(boxes);
  const rooms: Room[] = [];
  const seenRoomKeys = new Set<string>();
  const usedBoxes = new Set<number>();
  const usedGoals = new Set<number>();

  // Candidate doors: floor cells off the hallway row, adjacent to the hallway.
  for (let c = 0; c < w * lv.h; c++) {
    if (wall[c] || inHallway(c)) continue;
    let touchesHallway = false;
    let entry = -1;
    const x = c % w;
    const y = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DIRS[d][0];
      const ny = y + DIRS[d][1];
      if (nx < 0 || ny < 0 || nx >= w || ny >= lv.h) continue;
      const n = ny * w + nx;
      if (inHallway(n)) {
        touchesHallway = true;
        entry = n;
        break;
      }
    }
    if (!touchesHallway) continue;
    for (const comp of componentsWithout(lv, c)) {
      if (comp.includes(player)) continue;
      const compSet = new Set(comp);
      const rBoxes = comp.filter((cell) => boxSet.has(cell));
      const rGoals = comp.filter((cell) => goal[cell] === 1);
      if (boxSet.has(c)) rBoxes.push(c);
      if (goal[c] === 1) rGoals.push(c);
      if (rBoxes.length === 0 || rBoxes.length !== rGoals.length) continue;
      if (rBoxes.length < 3 || rBoxes.length > 4) continue;
      const key = [...comp].sort((a, b) => a - b).join(",");
      if (seenRoomKeys.has(key)) continue;
      seenRoomKeys.add(key);
      const floor = [...comp, c];
      const playerFloor = [...floor, entry];
      rooms.push({ door: c, entry, floor, playerFloor, boxes: rBoxes, goals: rGoals });
      for (const b of rBoxes) usedBoxes.add(b);
      for (const g of rGoals) usedGoals.add(g);
    }
  }

  // The decomposition is only trusted when it accounts for every box and
  // every goal exactly once. Otherwise the caller falls back to solving
  // the whole level as one room.
  if (rooms.length === 0) return null;
  if (usedBoxes.size !== boxes.length) return null;
  if (usedGoals.size !== goal.filter((g) => g === 1).length) return null;
  for (const r of rooms) {
    for (const b of r.boxes) {
      let n = 0;
      for (const r2 of rooms) if (r2.boxes.includes(b)) n++;
      if (n !== 1) return null;
    }
  }
  return rooms;
}

// ---------------------------------------------------------------------------
// Push-space A* for one room
// ---------------------------------------------------------------------------

type Push = { box: number; dir: number }; // box = global cell, dir = 0..3 (u,d,l,r)

type RoomSolve = {
  pushes: Push[];
  endPlayer: number; // global cell where the player ends up
  endBoxes: number[]; // sorted global cells of the boxes at the end
};

// Binary min-heap of node indices, ordered by (f, h, seq).
class Heap {
  a: number[] = [];
  f: number[];
  h: number[];
  seq: number[];
  constructor(f: number[], h: number[], seq: number[]) {
    this.f = f;
    this.h = h;
    this.seq = seq;
  }
  get size() {
    return this.a.length;
  }
  private less(i: number, j: number): boolean {
    const x = this.a[i];
    const y = this.a[j];
    if (this.f[x] !== this.f[y]) return this.f[x] < this.f[y];
    if (this.h[x] !== this.h[y]) return this.h[x] < this.h[y];
    return this.seq[x] < this.seq[y];
  }
  push(v: number) {
    const a = this.a;
    a.push(v);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!this.less(i, p)) break;
      const t = a[i];
      a[i] = a[p];
      a[p] = t;
      i = p;
    }
  }
  pop(): number {
    const a = this.a;
    const top = a[0];
    const last = a.pop() as number;
    if (a.length > 0) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1;
        const r = l + 1;
        let m = i;
        if (l < a.length && this.less(l, m)) m = l;
        if (r < a.length && this.less(r, m)) m = r;
        if (m === i) break;
        const t = a[i];
        a[i] = a[m];
        a[m] = t;
        i = m;
      }
    }
    return top;
  }
}

function solveRoom(lv: Level, room: Room, budget: number, greedy: boolean): RoomSolve | null {
  const { w, goal } = lv;
  const floor = room.floor;
  const F = floor.length;
  // Local indexing: room floor cells 0..F-1. The entry cell (the hallway
  // neighbour of the door, where the player starts) gets local index F,
  // unless it is already a floor cell (whole-level fallback), in which
  // case it keeps its floor index.
  const toLocal = new Map<number, number>();
  floor.forEach((cell, i) => toLocal.set(cell, i));
  const entryLocal = toLocal.has(room.entry) ? (toLocal.get(room.entry) as number) : F;
  const floorSet = new Set<number>();
  for (let i = 0; i < F; i++) floorSet.add(i);
  const playerSet = new Set<number>(floorSet);
  playerSet.add(entryLocal);
  const cellOf = (li: number): number => (li < F ? floor[li] : room.entry);
  const isBoxCell = (li: number): boolean => floorSet.has(li);

  // Neighbours in local indices (-1 when off the room). The entry cell is
  // a neighbour exactly when it is the hallway neighbour of a floor cell.
  const neigh: number[][] = [];
  for (let li = 0; li <= F; li++) {
    const g = cellOf(li);
    const x = g % w;
    const y = (g / w) | 0;
    const row: number[] = [];
    for (let d = 0; d < 4; d++) {
      const nx = x + DIRS[d][0];
      const ny = y + DIRS[d][1];
      if (nx < 0 || ny < 0 || nx >= w || ny >= lv.h) {
        row.push(-1);
        continue;
      }
      const n = ny * w + nx;
      const ln = toLocal.get(n);
      if (ln !== undefined) row.push(ln);
      else if (n === room.entry) row.push(entryLocal);
      else row.push(-1);
    }
    neigh.push(row);
  }

  const goalLocal = new Set<number>(room.goals.map((g) => toLocal.get(g) as number));
  const isGoal = (li: number): boolean => goalLocal.has(li);

  // All-pairs cell distances on the room floor (BFS from each goal).
  const dist: number[][] = [];
  for (const g of room.goals) {
    const gi = toLocal.get(g) as number;
    const d = new Array<number>(F + 1).fill(Infinity);
    d[gi] = 0;
    const q = [gi];
    for (let qi = 0; qi < q.length; qi++) {
      const c = q[qi];
      for (let dir = 0; dir < 4; dir++) {
        const n = neigh[c][dir];
        if (n < 0 || !isBoxCell(n) || d[n] !== Infinity) continue;
        d[n] = d[c] + 1;
        q.push(n);
      }
    }
    dist.push(d);
  }
  const minDist = new Array<number>(F + 1).fill(Infinity);
  for (let li = 0; li < F; li++) {
    for (let gi = 0; gi < dist.length; gi++) minDist[li] = Math.min(minDist[li], dist[gi][li]);
  }

  // Dead cells: reverse pull BFS from the goals. A cell is live when a box
  // there can still reach a goal by pulls; a box pushed onto a non-live
  // cell can never reach a goal, so such pushes are pruned. (The BFS
  // ignores other boxes and the player's exact reachability, which only
  // enlarges the live set, so pruning is sound.)
  const live = new Array<boolean>(F + 1).fill(false);
  {
    const q: number[] = [];
    for (const g of room.goals) {
      const gi = toLocal.get(g) as number;
      if (!live[gi]) {
        live[gi] = true;
        q.push(gi);
      }
    }
    for (let qi = 0; qi < q.length; qi++) {
      const b = q[qi];
      // A cell c next to b is live when a box on c can be pushed onto b:
      // the player must be able to stand on the far side of c, at
      // 2c - b (the cell beyond c away from b). (Pushes run forward, pulls
      // run backward; the stand cell is beyond the pulled box, not
      // beyond the live cell.)
      for (let d = 0; d < 4; d++) {
        const c = neigh[b][d ^ 1];
        if (c < 0 || !isBoxCell(c) || live[c]) continue;
        const s = neigh[c][d ^ 1];
        if (s < 0) continue; // player cannot stand there at all
        live[c] = true;
        q.push(c);
      }
    }
  }

  const nBoxes = room.boxes.length;
  const boxStart = room.boxes.map((b) => toLocal.get(b) as number).sort((a, b) => a - b);

  // Flood fill of the player's reachable region from a start cell, over
  // player-floor cells that hold no box. Returns the region as a boolean
  // array plus its smallest local index (canonical representative).
  function region(startLocal: number, boxAt: Uint8Array): { inRegion: Uint8Array; rep: number } {
    const inRegion = new Uint8Array(F + 1);
    const q = [startLocal];
    inRegion[startLocal] = 1;
    let rep = startLocal;
    for (let qi = 0; qi < q.length; qi++) {
      const c = q[qi];
      if (c < rep) rep = c;
      for (let d = 0; d < 4; d++) {
        const n = neigh[c][d];
        if (n < 0 || inRegion[n] || boxAt[n]) continue;
        inRegion[n] = 1;
        q.push(n);
      }
    }
    return { inRegion, rep };
  }

  // Admissible heuristic. For a few boxes: minimum-cost matching of boxes
  // to goals over cell distances (each push moves one box one cell, so a
  // box needs at least its cell distance in pushes; pushes for distinct
  // boxes are disjoint). Brute-force permutations: at most 6! = 720.
  // For many boxes (whole-level fallback): the sum of per-box minima,
  // which is a lower bound on the matching and hence on the true cost.
  function heuristic(boxesLocal: number[]): number {
    if (nBoxes <= 6) {
      let best = Infinity;
      const used = new Array<boolean>(nBoxes).fill(false);
      const rec = (k: number, sum: number) => {
        if (sum >= best) return;
        if (k === nBoxes) {
          best = sum;
          return;
        }
        for (let gi = 0; gi < nBoxes; gi++) {
          if (used[gi]) continue;
          const d = dist[gi][boxesLocal[k]];
          if (d === Infinity) continue;
          used[gi] = true;
          rec(k + 1, sum + d);
          used[gi] = false;
        }
      };
      rec(0, 0);
      return best;
    }
    let sum = 0;
    for (const b of boxesLocal) {
      if (minDist[b] === Infinity) return Infinity;
      sum += minDist[b];
    }
    return sum;
  }

  function isSolved(boxesLocal: number[]): boolean {
    for (const b of boxesLocal) if (!isGoal(b)) return false;
    return true;
  }

  // 2x2 frozen-block check on a box set (local indices). Any 2x2 square
  // of wall-or-box cells holding a box off its goal is a deadlock: the
  // boxes in it can never move again (each box's exits are blocked by the
  // square, and the player cannot stand inside it to push one out).
  // Only the squares touching a box off its goal can newly freeze.
  function hasFrozenBlock(boxesLocal: number[]): boolean {
    const boxAt = new Uint8Array(F + 1);
    for (const b of boxesLocal) boxAt[b] = 1;
    const seen = new Set<string>();
    for (const b of boxesLocal) {
      if (isGoal(b)) continue;
      const g = cellOf(b);
      const x = g % w;
      const y = (g / w) | 0;
      for (const [ox, oy] of [
        [0, 0],
        [-1, 0],
        [0, -1],
        [-1, -1],
      ]) {
        const cx = x + ox;
        const cy = y + oy;
        if (cx < 0 || cy < 0 || cx + 1 >= w || cy + 1 >= lv.h) continue;
        const key = cx + "," + cy;
        if (seen.has(key)) continue;
        seen.add(key);
        const cells = [cy * w + cx, cy * w + cx + 1, (cy + 1) * w + cx, (cy + 1) * w + cx + 1];
        let blocked = 0;
        let badBox = false;
        for (const gc of cells) {
          const li = toLocal.get(gc);
          if (li === undefined || !isBoxCell(li)) {
            blocked++; // wall, or the entry cell (boxes can never be there)
            continue;
          }
          if (boxAt[li]) {
            blocked++;
            if (!isGoal(li)) badBox = true;
          }
        }
        if (blocked === 4 && badBox) return true;
      }
    }
    return false;
  }

  // State key: packed box cells + player region representative. Numeric
  // packing only fits a few boxes; many boxes use a string key instead.
  const usePack = nBoxes <= 4;
  function packBoxes(boxesLocal: number[]): number {
    let p = 0;
    for (let i = nBoxes - 1; i >= 0; i--) p = p * F + boxesLocal[i];
    return p;
  }
  function stateKey(boxesLocal: number[], rep: number): number | string {
    if (usePack) return packBoxes(boxesLocal) * (F + 1) + rep;
    return boxesLocal.join(",") + "|" + rep;
  }

  // A* (or greedy best-first when greedy=true) over push states.
  const par: number[] = [];
  const parBox: number[] = []; // local box cell the push moved FROM
  const parDir: number[] = [];
  const gScore: number[] = [];
  const hScore: number[] = [];
  const fScore: number[] = [];
  const seqNo: number[] = [];
  const stateBoxes: number[][] = [];
  const statePlayer: number[] = [];
  const stateRep: number[] = [];
  let seq = 0;

  const startBoxAt = new Uint8Array(F + 1);
  for (const b of boxStart) startBoxAt[b] = 1;
  const startRegion = region(entryLocal, startBoxAt);
  const h0 = heuristic(boxStart);
  if (h0 === Infinity) return null;
  const startKey = stateKey(boxStart, startRegion.rep);
  const visited = new Map<number | string, number>();
  visited.set(startKey, 0);
  par.push(-1);
  parBox.push(-1);
  parDir.push(-1);
  gScore.push(0);
  hScore.push(h0);
  fScore.push(greedy ? h0 : h0);
  seqNo.push(seq++);
  stateBoxes.push([...boxStart]);
  statePlayer.push(entryLocal);
  stateRep.push(startRegion.rep);
  const heap = new Heap(fScore, hScore, seqNo);
  heap.push(0);

  let expansions = 0;
  let goalNode = -1;
  while (heap.size > 0) {
    if (expansions >= budget || expansionsTotal >= GLOBAL_BUDGET) break;
    const node = heap.pop();
    const boxesLocal = stateBoxes[node];
    const g = gScore[node];
    // The heap may hold a stale entry superseded by a better path.
    const keyHere = stateKey(boxesLocal, stateRep[node]);
    if ((visited.get(keyHere) as number) < g) continue;
    expansions++;
    expansionsTotal++;
    if (isSolved(boxesLocal)) {
      goalNode = node;
      break;
    }
    const boxAt = new Uint8Array(F + 1);
    for (const b of boxesLocal) boxAt[b] = 1;
    const reg = region(statePlayer[node], boxAt);
    for (let bi = 0; bi < nBoxes; bi++) {
      const b = boxesLocal[bi];
      for (let d = 0; d < 4; d++) {
        const dest = neigh[b][d];
        if (dest < 0 || !isBoxCell(dest) || boxAt[dest]) continue;
        const stand = neigh[b][d ^ 1]; // player stands opposite the push direction
        if (stand < 0 || !reg.inRegion[stand]) continue;
        if (!live[dest]) continue; // box would land on a dead cell
        // New box set.
        const nb = [...boxesLocal];
        nb[bi] = dest;
        nb.sort((a, b2) => a - b2);
        if (hasFrozenBlock(nb)) continue;
        const nh = heuristic(nb);
        if (nh === Infinity) continue;
        const nPlayer = b; // player steps into the box's old cell
        const nBoxAt = new Uint8Array(F + 1);
        for (const x of nb) nBoxAt[x] = 1;
        const nReg = region(nPlayer, nBoxAt);
        const nKey = stateKey(nb, nReg.rep);
        const ng = g + 1;
        const prev = visited.get(nKey);
        if (prev !== undefined && prev <= ng) continue;
        visited.set(nKey, ng);
        const idx = par.length;
        par.push(node);
        parBox.push(b);
        parDir.push(d);
        gScore.push(ng);
        hScore.push(nh);
        fScore.push(greedy ? nh : ng + nh);
        seqNo.push(seq++);
        stateBoxes.push(nb);
        statePlayer.push(nPlayer);
        stateRep.push(nReg.rep);
        heap.push(idx);
      }
    }
  }

  if (goalNode < 0) return null;
  // Reconstruct the push list.
  const pushes: Push[] = [];
  let cur = goalNode;
  while (par[cur] >= 0) {
    pushes.push({ box: cellOf(parBox[cur]), dir: parDir[cur] });
    cur = par[cur];
  }
  pushes.reverse();
  const endBoxes = stateBoxes[goalNode].map((li) => cellOf(li)).sort((a, b) => a - b);
  return { pushes, endPlayer: cellOf(statePlayer[goalNode]), endBoxes };
}

// ---------------------------------------------------------------------------
// Expanding pushes to a full LURD walk
// ---------------------------------------------------------------------------

// BFS shortest walk for the player over floor cells (walls and boxes are
// obstacles), fixed neighbour order u,d,l,r. Returns the move letters, or
// null when the target is unreachable.
function routePlayer(lv: Level, boxes: Set<number>, from: number, to: number): string | null {
  if (from === to) return "";
  const { w, h, wall } = lv;
  const prev = new Int32Array(w * h).fill(-1);
  const prevDir = new Int8Array(w * h).fill(-1);
  const q = [from];
  prev[from] = from;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi];
    const x = c % w;
    const y = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DIRS[d][0];
      const ny = y + DIRS[d][1];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (wall[n] || boxes.has(n) || prev[n] !== -1) continue;
      prev[n] = c;
      prevDir[n] = d;
      if (n === to) {
        // Reconstruct.
        let s = "";
        let cur = n;
        const rev: string[] = [];
        while (cur !== from) {
          rev.push(DIRCH[prevDir[cur]]);
          cur = prev[cur];
        }
        rev.reverse();
        s = rev.join("");
        return s;
      }
      q.push(n);
    }
  }
  return null;
}

// Solve one level: decompose into rooms, solve each, expand to LURD.
// Returns the move string ("" when nothing could be solved — still valid).
function solveLevel(lv: Level): string {
  const rooms = decompose(lv);
  let solves: { room: Room; solve: RoomSolve }[] = [];

  if (rooms !== null) {
    for (const room of rooms) {
      let s = solveRoom(lv, room, ROOM_ASTAR_BUDGET, false);
      if (s === null) s = solveRoom(lv, room, ROOM_GREEDY_BUDGET, true);
      if (s === null) return ""; // one room unsolved: the level cannot score
      solves.push({ room, solve: s });
    }
  } else {
    // Fallback: the whole level as one room (no door, no entry cell).
    const allFloor = [];
    for (let c = 0; c < lv.w * lv.h; c++) if (!lv.wall[c]) allFloor.push(c);
    const whole: Room = {
      door: -1,
      entry: lv.player,
      floor: allFloor,
      playerFloor: allFloor,
      boxes: [...lv.boxes],
      goals: (() => {
        const gs: number[] = [];
        for (let c = 0; c < lv.w * lv.h; c++) if (lv.goal[c] === 1) gs.push(c);
        return gs;
      })(),
    };
    let s = solveRoom(lv, whole, ROOM_ASTAR_BUDGET, false);
    if (s === null) s = solveRoom(lv, whole, ROOM_GREEDY_BUDGET, true);
    if (s === null) return "";
    solves = [{ room: whole, solve: s }];
  }

  // Order rooms so the player never gets trapped: a room whose door ends
  // blocked by a box on a goal must be solved last (the player cannot
  // leave it afterwards). Exitable rooms first, in discovery order.
  const exitable = (entry: { room: Room; solve: RoomSolve }): boolean => {
    const { room, solve } = entry;
    if (room.door < 0) return true;
    const boxSet = new Set(solve.endBoxes);
    const walk = routePlayer(lv, boxSet, solve.endPlayer, room.door);
    return walk !== null;
  };
  solves.sort((a, b) => {
    const ea = exitable(a) ? 0 : 1;
    const eb = exitable(b) ? 0 : 1;
    return ea - eb;
  });

  // Expand.
  let moves = "";
  let player = lv.player;
  const boxes = new Set<number>(lv.boxes);
  for (const { solve } of solves) {
    for (const p of solve.pushes) {
      const stand = p.box + DIRS[p.dir ^ 1][1] * lv.w + DIRS[p.dir ^ 1][0];
      const walk = routePlayer(lv, boxes, player, stand);
      if (walk === null) return ""; // should not happen; stay valid
      moves += walk;
      moves += DIRCH[p.dir];
      boxes.delete(p.box);
      boxes.add(p.box + DIRS[p.dir][1] * lv.w + DIRS[p.dir][0]);
      player = p.box;
    }
  }
  return moves;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

function main() {
  // The harness (and the kit's scorer) point the entry at its input and
  // output through MAKEFUN_INPUT_DIR / MAKEFUN_OUTPUT_DIR when they run
  // it outside the sandbox; inside the sandbox those are unset and the
  // absolute /input and /output apply. Command-line arguments override
  // both, for local testing only.
  const inputDir = process.argv[2] ?? process.env.MAKEFUN_INPUT_DIR ?? "/input";
  const outputDir = process.argv[3] ?? process.env.MAKEFUN_OUTPUT_DIR ?? "/output";
  const ids = readdirSync(inputDir)
    .filter((name) => /^i\d{3}$/.test(name))
    .sort();
  const useIds = ids.length > 0 ? ids : ["i001", "i002", "i003", "i004", "i005"];
  for (const id of useIds) {
    const text = readFileSync(join(inputDir, id, "level.txt"), "utf8");
    const lv = parseLevel(text);
    const moves = solveLevel(lv);
    const outDir = join(outputDir, id);
    mkdirSync(outDir, { recursive: true });
    writeFileSync(join(outDir, "solution.txt"), moves, "utf8");
  }
}

main();
