# Room-wise push-optimal Sokoban solver

[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=badge&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Schyd/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=badge&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=badge&utm_campaign=sokoban-solver-01-run-20261121)

Entry for the make.fun bounty
[Build the best Sokoban solver, 21 November 2026](https://make.fun/bounty/sokoban-solver-01-run-20261121):
one program, `main.ts`, that answers every instance of a set in one run.

## How it works

The scoring cliff makes push-optimality everything: a level solved one
push over its optimum earns under half its points. So the solver never
searches the whole level at once. It uses the level's own structure:

1. **Decompose by room.** Every generated level is a hallway with one to
   four rooms off it, each room holding 3–4 boxes and as many goals and
   opening onto the hallway through one door. A box pushed through a door
   into the hallway can never be pushed back, so every box stays in its
   room and the level's push-optimal count is the sum of its rooms'
   optima. Rooms are found as the components cut off by the door cells
   (articulation points next to the hallway row).
2. **Push-space A\* per room.** A state is the boxes' positions plus the
   player's reachable region. The heuristic is the minimum-cost matching
   of boxes to goals over precomputed cell distances — admissible, so the
   first solution found is push-optimal for that room. Dead cells are
   pruned by a reverse pull search from the goals, and 2×2 frozen blocks
   of wall-or-box are rejected outright.
3. **Expand to a walk.** Each room's push list becomes a full LURD move
   string by routing the player between pushes (and between rooms, along
   the hallway) with shortest-path search. Rooms whose door ends blocked
   are solved last so the player is never trapped inside.
4. **Fixed work budgets, never the clock.** Every search is capped in
   expanded nodes, so two runs on the same input produce byte-identical
   outputs. If a room's optimal search exhausts its budget, a greedy
   best-first search falls back to any solution instead of none.

## Results

- Practice set: **1,000,000** — all 5 instances solved in exactly P\*
  pushes (13, 16, 29, 47, 60), in ~0.36 s for the whole set (the wall
  budget is 60 s). The house baseline scores 200,000 on the same set
  (only i001 at P\*).
- Generalisation: also graded on two extra sets drawn with the kit's own
  generator from other seeds — **1,000,000 on both** (extra set 1: P\*
  12/23/37/44/60; extra set 2: P\* 18/18/36/48/66, including a 21-push
  room), each in well under a second. Two runs on the same input are
  byte-identical.

## Run it

```sh
node main.ts            # reads /input/<id>/level.txt, writes /output/<id>/solution.txt
node main.ts <in> <out> # local testing against any input directory
```

Node 22+ (erasable TypeScript only, no dependencies, no network, no
clock reads). The whole repository is one small file plus this README
and the MIT licence.
