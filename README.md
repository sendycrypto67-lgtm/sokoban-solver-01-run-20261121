# sokoban-solver-01-run-20261121

Push-optimal Sokoban solver for the make.fun bounty "Build the best Sokoban solver, 21 November 2026".

The solver decomposes each level into its independent rooms (a hallway plus 1-4 rooms of 3-4 boxes), solves every room push-optimally with A* over pushes (minimum-matching heuristic, sound deadlock pruning), and concatenates the rooms with hallway walks. One run answers all 5 instances in `/input/<id>/level.txt` and writes `/output/<id>/solution.txt`.

Run locally: `MAKEFUN_INPUT_DIR=<kit>/practice/input MAKEFUN_OUTPUT_DIR=<out> node main.ts`

License: MIT. See LICENSE.
