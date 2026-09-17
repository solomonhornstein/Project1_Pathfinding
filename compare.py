"""Headless comparison of search strategies across fixed boards.

Runs BFS and A* (with each heuristic) over the same set of boards and
reports path length, nodes explored, and elapsed time. BFS is included
as ground truth: it ignores heuristics entirely and always returns a
shortest path, so any A* run with a longer path is suboptimal.

Run: python compare.py
"""

import time

from pathfinding import (Board, WALL, astar, bfs,
                         manhattan, euclidean, chebyshev, weighted_manhattan, EMPTY)

SIZE = 40
START, GOAL = (5, 5), (34, 34)


# --- Boards ------------------------------------------------------------------
#
# Each builder returns a fresh Board. None of the searches mutate the grid,
# so one board can be reused across all strategies.

def open_board():
    """No obstacles.

    Every monotone route from start to goal costs the same here, so this
    board isolates how a heuristic handles ties.
    """
    return Board(SIZE, SIZE)


def wall_with_gap():
    """A vertical wall with a single opening the path has to find."""
    board = Board(SIZE, SIZE)
    for r in range(SIZE):
        if r != 30:
            board.grid[r][20] = WALL
    return board

def _clear_endpoints(board):
    """Make sure obstacle placement didn't bury the start or goal."""
    board.grid[START[0]][START[1]] = EMPTY
    board.grid[GOAL[0]][GOAL[1]] = EMPTY
    return board

def long_detour():
    """Serpentine walls with alternating gaps.

    The true path is far longer than the straight-line distance, which is
    the condition under which an inflated heuristic can commit to a route
    that looks good and isn't.
    """
    board = Board(SIZE, SIZE)
    for i, r in enumerate(range(12, SIZE - 6, 7)):
        if i % 2 == 0:
            cols = range(0, SIZE - 4)      # gap on the right
        else:
            cols = range(4, SIZE)          # gap on the left
        for c in cols:
            board.grid[r][c] = WALL
    return _clear_endpoints(board)


def scattered():
    """A regular field of short obstacles.

    Many near-equal routes exist, so this board stresses tie-breaking
    rather than the ability to find a single corridor.
    """
    board = Board(SIZE, SIZE)
    for r in range(4, SIZE - 2, 6):
        for c in range(4, SIZE - 2, 6):
            for dc in range(3):
                board.grid[r][c + dc] = WALL
    return _clear_endpoints(board)


BOARDS = [
    ("open", open_board),
    ("wall with gap", wall_with_gap),
    ("long detour", long_detour),
    ("scattered", scattered),
]


# --- Strategies --------------------------------------------------------------

STRATEGIES = [
    ("bfs",       None),          # None means "no heuristic": run BFS
    ("manhattan", manhattan),
    ("euclidean", euclidean),
    ("chebyshev", chebyshev),
    ("weighted",  weighted_manhattan),
]


def run(h, board):
    """Time one search. h=None runs BFS. Returns (path_len, explored, ms)."""
    t0 = time.perf_counter()
    if h is None:
        path, explored = bfs(board, START, GOAL)
    else:
        path, explored = astar(board, START, GOAL, h=h)
    elapsed_ms = (time.perf_counter() - t0) * 1000
    return len(path), len(explored), elapsed_ms


# --- Reporting ---------------------------------------------------------------

def compare_board(label, build):
    """Run every strategy on one board and print a table of results."""
    board = build()

    print(f"\n{label}")
    print(f"{'strategy':<12}{'path':>8}{'explored':>12}{'ms':>10}")
    print("-" * 42)

    optimal = None
    for name, h in STRATEGIES:
        path_len, explored, ms = run(h, board)

        # BFS runs first and defines the true shortest path length.
        if optimal is None:
            optimal = path_len

        flag = " SUBOPTIMAL" if path_len > optimal else ""
        print(f"{name:<12}{path_len:>8}{explored:>12}{ms:>10.1f}{flag}")


def main():
    print(f"grid {SIZE}x{SIZE}, start {START}, goal {GOAL}")
    for label, build in BOARDS:
        compare_board(label, build)
    print()


if __name__ == "__main__":
    main()