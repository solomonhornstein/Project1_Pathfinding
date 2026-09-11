"""
A* Pathfinding Exploration Project
Grid rendering and mouse input.

Controls:
    Left click / drag    paint walls
    Right click / drag   erase
    Shift + left click   set start
    Alt + left click     set goal
    C                    clear the board
"""

import pygame

# --- Configuration -----------------------------------------------------------

ROWS, COLS = 40, 40
CELL = 18
WIDTH, HEIGHT = COLS * CELL, ROWS * CELL

EMPTY, WALL = 0, 1

BG       = (30, 30, 30)
GRIDLINE = (55, 55, 55)
WALL_C   = (200, 200, 200)
START_C  = (80, 200, 120)
GOAL_C   = (220, 80, 80)
EXPLORED_C = (60, 90, 140)
PATH_C     = (240, 200, 60)

# (row delta, column delta) — up, down, left, right
ORTHOGONAL = [(-1, 0), (1, 0), (0, -1), (0, 1)]


# --- Board -------------------------------------------------------------------

class Board:
    """Holds the grid state and knows how to draw itself."""

    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.clear()

    def clear(self):
        # A comprehension builds a fresh list per row. [[EMPTY] * cols] * rows
        # would store the same row object repeatedly, so one write hits them all.
        self.grid = [[EMPTY] * self.cols for _ in range(self.rows)]
        self.start = None
        self.goal = None
        self.explored = []
        self.path = []

    def cell_at(self, pos):
        """Mouse (x, y) -> (row, col). y gives the row, x gives the column."""
        x, y = pos
        return y // CELL, x // CELL

    def in_bounds(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_wall(self, r, c):
        return self.grid[r][c] == WALL

    def paint_wall(self, r, c):
        if (r, c) != self.start and (r, c) != self.goal:
            self.grid[r][c] = WALL

    def erase(self, r, c):
        self.grid[r][c] = EMPTY
        if (r, c) == self.start:
            self.start = None
        if (r, c) == self.goal:
            self.goal = None

    def set_start(self, r, c):
        if (r, c) == self.goal:
            return
        self.grid[r][c] = EMPTY
        self.start = (r, c)

    def set_goal(self, r, c):
        if (r, c) == self.start:
            return
        self.grid[r][c] = EMPTY
        self.goal = (r, c)

    def draw(self, screen):
        screen.fill(BG)

        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == WALL:
                    self._fill_cell(screen, r, c, WALL_C)

        for r, c in self.explored:
            self._fill_cell(screen, r, c, EXPLORED_C)

        for r, c in self.path:
            self._fill_cell(screen, r, c, PATH_C)

        if self.start is not None:
            self._fill_cell(screen, *self.start, START_C)
        if self.goal is not None:
            self._fill_cell(screen, *self.goal, GOAL_C)

        self._draw_gridlines(screen)

    def _fill_cell(self, screen, r, c, color):
        """Single place where (row, col) becomes (x, y). Column is x."""
        pygame.draw.rect(screen, color, (c * CELL, r * CELL, CELL, CELL))

    def _draw_gridlines(self, screen):
        for c in range(self.cols + 1):
            pygame.draw.line(screen, GRIDLINE, (c * CELL, 0), (c * CELL, HEIGHT))
        for r in range(self.rows + 1):
            pygame.draw.line(screen, GRIDLINE, (0, r * CELL), (WIDTH, r * CELL))

    def neighbors(self, r, c):
        """Yield walkable cells adjacent to (r, c)."""
        for dr, dc in ORTHOGONAL:
            nr, nc = r + dr, c + dc
            if self.in_bounds(nr, nc) and not self.is_wall(nr, nc):
                yield nr, nc

# --- Main loop ---------------------------------------------------------------

def handle_event(event, board):
    """Discrete actions: one response per occurrence."""
    if event.type == pygame.QUIT:
        return False

    if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
        mods = pygame.key.get_mods()
        r, c = board.cell_at(event.pos)
        if board.in_bounds(r, c):
            if mods & pygame.KMOD_SHIFT:
                board.set_start(r, c)
            elif mods & pygame.KMOD_ALT:
                board.set_goal(r, c)

    if event.type == pygame.KEYDOWN and event.key == pygame.K_c:
        board.clear()

    return True


def handle_drag(board):
    """Continuous state: checked fresh every frame so dragging works."""
    left, _, right = pygame.mouse.get_pressed()
    if not (left or right):
        return

    mods = pygame.key.get_mods()
    if mods & (pygame.KMOD_SHIFT | pygame.KMOD_ALT):
        return  # modifier held: that click is placing start/goal, not painting

    r, c = board.cell_at(pygame.mouse.get_pos())
    if board.in_bounds(r, c):
        if left:
            board.paint_wall(r, c)
        else:
            board.erase(r, c)

def reconstruct_path(came_from, start, goal):
    """Walk backwards from goal to start. Returns [] if goal wasn't reached."""
    if goal != start and goal not in came_from:
        return []

    path = [goal]
    current = goal
    while current != start:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path

from collections import deque


def bfs(board, start, goal):
    """Breadth-first search. Returns (path, explored).

    Explores in order of steps from the start, so the first route found
    to the goal is the shortest. Ignores movement cost entirely.
    """
    queue = deque([start])
    came_from = {}
    visited = {start}
    explored = []

    while queue:
        current = queue.popleft()
        explored.append(current)

        if current == goal:
            break

        for neighbor in board.neighbors(*current):
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                queue.append(neighbor)
                pass

    return reconstruct_path(came_from, start, goal), explored

import heapq

MOVE_COST = 1


def manhattan(a, b):
    """Steps along the grid, ignoring walls. Admissible for 4-way movement."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(board, start, goal, h=manhattan):
    """A* search. Returns (path, explored).

    Expands the cell with the lowest f = g + h, where g is the known cost
    from the start and h estimates the cost remaining to the goal.
    """
    count = 0
    open_heap = [(h(start, goal), count, start)]
    came_from = {}
    g_score = {start: 0}
    closed = set()
    explored = []

    while open_heap:
        _, _, current = heapq.heappop(open_heap)

        if current in closed:
            continue            # stale duplicate; a cheaper copy already ran
        closed.add(current)
        explored.append(current)

        if current == goal:
            break

        for neighbor in board.neighbors(*current):
            tentative_g = g_score[current] + MOVE_COST

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                count += 1
                f = tentative_g + h(neighbor, goal)
                heapq.heappush(open_heap, (f, count, neighbor))

    return reconstruct_path(came_from, start, goal), explored

def handle_event(event, board):
    if event.type == pygame.QUIT:
        return False

    if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
        mods = pygame.key.get_mods()
        r, c = board.cell_at(event.pos)
        if board.in_bounds(r, c):
            if mods & pygame.KMOD_SHIFT:
                board.set_start(r, c)
            elif mods & pygame.KMOD_ALT:
                board.set_goal(r, c)

    if event.type == pygame.KEYDOWN:
        if event.key == pygame.K_c:
            board.clear()
        elif event.key == pygame.K_SPACE:
            if board.start is not None and board.goal is not None:
                board.path, board.explored = astar(board, board.start, board.goal)
                print(f"path: {len(board.path)} cells, explored: {len(board.explored)}")
            else:
                print("set a start (shift-click) and goal (alt-click) first")
    return True

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("A* Pathfinding")
    clock = pygame.time.Clock()

    board = Board(ROWS, COLS)

    running = True
    while running:
        for event in pygame.event.get():
            running = handle_event(event, board) and running

        handle_drag(board)

        board.draw(screen)
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()