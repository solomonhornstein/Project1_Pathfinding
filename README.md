# Project 1: A* Pathfinding Exploration

An interactive grid where you paint obstacles, place a start and a goal, and
watch A* search for a shortest route. BFS is included for comparison, and four
heuristics can be swapped at runtime.

ADV Machine Learning and AI — Mr. Cochran — Durham Academy

## Setup

Requires Python 3.12 and `pygame-ce` (identical API to `pygame`; the import
statement is unchanged). Python 3.14 has no `pygame` wheels.

```
python -m pip install pygame-ce
python pathfinding.py      # interactive visualizer
python compare.py          # headless comparison across four boards
```

## Controls

| Input | Action |
|---|---|
| Left click / drag | Paint walls |
| Right click / drag | Erase |
| Shift + left click | Set start |
| Alt + left click | Set goal |
| `1` `2` `3` `4` | manhattan / euclidean / chebyshev / weighted |
| `Space` | Run A* |
| `C` | Clear |

Explored cells draw in blue, the path in yellow. The search animates at
`STEPS_PER_FRAME` expansions per frame.

## A* vs. BFS and Dijkstra

All three share one loop; what changes is which cell is expanded next.

| | Expands next | Weighted terrain | Uses goal position |
|---|---|---|---|
| BFS | Oldest (FIFO) | No | No |
| Dijkstra | Lowest `g` | Yes | No |
| A* | Lowest `g + h` | Yes | Yes |

`g` is the known cost from the start, `h` an estimate of the cost remaining.
Setting `h = 0` reduces A* to Dijkstra exactly — the heuristic is the only
structural difference.

## Heuristics

| Key | Name | Formula | Admissible on a 4-way grid? |
|---|---|---|---|
| `1` | Manhattan | `|dr| + |dc|` | Yes — exact on open ground |
| `2` | Euclidean | `sqrt(dr² + dc²)` | Yes, underestimates |
| `3` | Chebyshev | `max(|dr|, |dc|)` | Yes, underestimates more |
| `4` | Weighted Manhattan | `1.5 × manhattan` | No — may overestimate |

Admissible means never overestimating the true remaining cost, which is what
guarantees an optimal path.

## Results

40×40 grid, start (5, 5), goal (34, 34). `explored` = cells expanded.

| Board | Strategy | Path | Explored |
|---|---|---|---|
| Open | bfs | 59 | 1540 |
| | manhattan | 59 | **59** |
| | euclidean | 59 | 1091 |
| | chebyshev | 59 | 1098 |
| | weighted | 59 | 59 |
| Wall with gap | bfs | 59 | 1085 |
| | manhattan | 59 | 119 |
| | euclidean | 59 | 717 |
| | chebyshev | 59 | 732 |
| | weighted | **63** | 118 ← suboptimal |
| Long detour | bfs | 191 | 1411 |
| | manhattan | 191 | 1222 |
| | euclidean | 191 | 1236 |
| | chebyshev | 191 | 1236 |
| | weighted | 191 | 1221 |
| Scattered | bfs | 59 | 1435 |
| | manhattan | 59 | 71 |
| | euclidean | 59 | 1008 |
| | chebyshev | 59 | 1015 |
| | weighted | 59 | 71 |

**Manhattan on the open board expanded only the path itself** — 59 cells for a
59-cell path, zero wasted work, against BFS's 1540 for the same path.

**Admissible does not mean good.** Euclidean and Chebyshev are admissible and
optimal but explored ~1000 cells where Manhattan explored 59. On a 4-way grid
Manhattan is *exactly* the remaining distance; the other two assume diagonal
shortcuts that don't exist, so they underestimate and carry less information.

**Inadmissibility is relative to the board.** Weighted Manhattan went
suboptimal on wall-with-gap (63 vs. 59) but stayed optimal on the detour board.
There the true cost is 191 while Manhattan estimates 58, so `1.5 × 58` is still
a large underestimate — it never overestimates, so it never misleads.

**A\* degenerates toward Dijkstra when `h` is uninformative.** On the detour
board Manhattan explored 1222 against BFS's 1411. A heuristic reporting 58 when
the answer is 191 barely contributes to `f`, so `f ≈ g` and the search loses
direction. A*'s advantage is proportional to how well `h` reflects the space.

## Implementation notes

**Tie-breaking.** The first version explored the entire bounding rectangle —
correct, not a bug: on an open grid every monotone route costs the same, so
`g + h` is identical everywhere inside it. Heap entries are
`(f, -tentative_g, count, cell)`; negating `g` breaks ties toward cells further
from the start, pushing the search deeper instead of fanning out. `count`
breaks remaining ties by insertion order so two cells are never compared
directly. Scaling `h` by 1.001 would have a similar effect but sacrifices
admissibility, so `-g` was preferred.

**Animated search.** `astar_steps` is a generator yielding once per expansion;
the main loop advances it `STEPS_PER_FRAME` times per frame. `explored` is
passed in rather than created inside, so the board and the search share one
list object and the frontier draws as it grows. `astar` wraps the same
generator for headless use, so the comparison numbers describe exactly the code
that runs on screen.

**Structure.** Coordinate translation lives in exactly two places — `cell_at`
(mouse → row/col) and `_fill_cell` (row/col → x/y) — since grid indexing is
row-first while `pygame.draw.rect` takes x first. Heuristics are passed as
function arguments so all four run through an identical search. Input is split
by mechanism: discrete actions as events, drag-to-paint polled each frame.

## AI use

Claude (Anthropic) was used throughout this project. Full conversation:

https://claude.ai/share/5614601f-90f1-4d0b-8869-64854857f754

I used Claude (Anthropic) throughout this project, alongside the course-recommended resources. I used it to set up my environment (diagnosing a pygame install failure caused by Python 3.14 having no wheels, and moving to 3.12), and to explain concepts I hadn't used before — generators, heapq, and what makes a heuristic admissible. For the algorithms, Claude gave me annotated scaffolds with the core logic left blank, and I wrote the neighbor-expansion block in BFS and the g_score comparison and update block in A* myself. It reviewed my file structure and reorganized it into the current class-based layout, and it diagnosed two bugs: a duplicate handle_event definition that Python was silently overwriting, and a missing tie-break term that was causing A* to explore the entire bounding rectangle. It drafted the README from results I generated, which I then edited. I chose Claude over FlintK12 because I wanted explanations of why the code worked rather than working code, and it was willing to leave gaps for me to fill.