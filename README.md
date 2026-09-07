# Movhex

C implementation of a dynamic route-planning system on a rectangular hexagonal grid.

The project was developed for the final exam of the *Algorithms and Data Structures* course at Politecnico di Milano.

The program reads commands from standard input and computes the minimum travel cost between hexagonal cells while supporting dynamic updates of terrain costs and directed air routes.

## Problem overview

The map is a rectangular grid of hexagons. Each cell has:

- A ground exit cost in the range `[0, 100]`.
- Up to five outgoing directed air routes.
- Ground connections to its valid adjacent hexagons.

A ground exit cost equal to `0` means that a cell cannot be left through ground connections, although it can still be reached or potentially left through an air route with positive cost.

The supported commands are:

- `init <columns> <rows>`
- `change_cost <x> <y> <value> <radius>`
- `toggle_air_route <x1> <y1> <x2> <y2>`
- `travel_cost <source_x> <source_y> <destination_x> <destination_y>`

The complete assignment statement is available in [`specifica_movhex.pdf`](specifica_movhex.pdf).

## Solution approach

The map is modeled as an implicit weighted directed graph:

- Each hexagonal cell corresponds to one graph node.
- Ground movements correspond to edges toward the six possible neighbouring hexagons.
- The cost of a ground edge is the exit cost of the source cell.
- Air routes are additional directed weighted edges.

Ground edges are not stored explicitly because hexagonal adjacency is fixed by the grid geometry. Neighbours are generated when needed using the parity of the row.

The conversion from coordinates `(x, y)` to a linear array index is performed by the macro:

```c
#define IDX(x, y, map) ((y) * (map)->colonne + (x))
```

This allows the whole map to be stored in contiguous arrays.

## Data representation

### Cells

Each cell is represented by the `Cell` structure:

```c
typedef struct {
    uint32_t offset;
    uint16_t deg;
    uint8_t costo_out;
    uint8_t _pad;
} Cell;
```

- `costo_out` is the ground exit cost.
- `deg` is the number of outgoing air routes.
- `offset` identifies the beginning of the cell's air routes inside the global route arrays.

### Air routes

Air routes are stored in a compact CSR-like representation inside the `Map` structure:

```c
typedef struct {
    int righe;
    int colonne;
    Cell *cells;
    uint32_t *air_dest;
    uint8_t *air_cost;
    int cellsN;
    int airR;
} Map;
```

Instead of allocating a separate list for every cell, all air routes are stored in two global arrays:

- `air_dest`: destination cell index for each route.
- `air_cost`: cost associated with each route.

For a given cell, `offset` and `deg` define the contiguous range of its outgoing routes. The helper function `cell_air_range(...)` retrieves this range.

This representation keeps memory compact and avoids per-route dynamic allocations. Since each cell can have at most five outgoing air routes, the number of air edges remains bounded.

## Hexagonal grid geometry

The implementation uses an offset-coordinate representation for the rectangular hexagonal grid.

The function `get_neighbors(...)` generates the valid neighbours of a cell. The neighbour offsets depend on whether the row is even or odd:

```c
static const int DX = {[2]
    {-1, +1, -1, 0, -1, 0},
    {-1, +1,  0, +1, 0, +1}
};

static const int DY = {[2]
    {0, 0, +1, +1, -1, -1},
    {0, 0, +1, +1, -1, -1}
};
```

This avoids storing the six ground edges for every cell.

The functions `offset_to_cube(...)` and `hex_distance(...)` are also provided to convert offset coordinates into cube coordinates and calculate hexagonal distance.

## Air route updates

The function `toggle_air_route(...)` adds or removes a directed air route.

When a route already exists, it is removed from the global CSR arrays using:

```c
air_shift_left_one(M, found);
```

When a route does not exist, it is inserted at the end of the source cell's route range. The function:

```c
ensure_air_capacity(M, M->airR + 1);
```

resizes the global route arrays when necessary.

After each insertion or removal, `bump_offsets(...)` updates the offsets of subsequent cells to preserve the CSR layout.

The cost of a newly inserted air route is calculated as the floor of the average between:

- The source cell ground exit cost.
- The costs of the source cell's existing outgoing air routes.

The implementation enforces the maximum of five outgoing routes per cell through:

```c
#define MAX_ROUTES 5
```

## Cost updates

The command `change_cost` is implemented by the function:

```c
change_cost_bfs(int cx, int cy, int v, int r, Map *M)
```

The function performs a breadth-first search from the centre cell up to the requested radius.

For each visited cell, it computes the distance-dependent variation:

```c
delta = floor(v * (r - dist) / r);
```

The same variation is applied to:

- The ground exit cost of the visited cell.
- The costs of all air routes leaving that cell.

All costs are clamped to the valid interval `[0, 100]`.

A bitset is used to mark visited cells, reducing the memory required for the BFS visited set. The BFS queue and the bitset are statically reused across calls, avoiding repeated allocations for frequent updates.

## Shortest-path queries

The command `travel_cost` is implemented by:

```c
travel_cost(int x1, int y1, int x2, int y2, const Map *M)
```

The function solves a shortest-path problem over:

- Dynamically generated ground edges.
- Explicit directed air-route edges.

Instead of using a binary heap, the implementation uses a bucket-based variant of Dijkstra's algorithm, similar to Dial's algorithm.

The approach is possible because every edge weight is bounded:

```c
#define CMAX 101
```

Ground and air-route costs are always in the range `[0, 100]`. Distances are placed into circular buckets using:

```c
int b = node_dist % CMAX;
```

The key bucket operations are:

```c
bucket_push(node_idx, node_dist);
bucket_pop();
```

This avoids the logarithmic overhead of a priority queue and is suitable for the bounded integer edge costs required by the problem.

The search stops as soon as the destination node is extracted from the buckets:

```c
if (u_idx == dst_idx) return du;
```

## Reused query workspace

Shortest-path queries reuse global work arrays:

```c
static int32_t  *best_cost_per_cell;
static uint32_t *seen_generation;
static uint32_t current_generation;
```

`best_cost_per_cell` stores the best known cost for each cell.

`seen_generation` and `current_generation` implement lazy initialization: cells belonging to an older query do not need to be reset individually before the next query. A cell is considered active in the current search only when:

```c
seen_generation[cell] == current_generation
```

This reduces the initialization work for repeated `travel_cost` operations.

The workspace is reset only when the map is reinitialized through `init`.

## Input and output

The program reads commands from standard input and prints one response per command.

Example:

```text
init 100 100
change_cost 10 20 -10 5
travel_cost 0 0 20 0
toggle_air_route 0 0 20 0
travel_cost 0 0 20 0
```

Possible outputs include:

```text
OK
KO
-1
<minimum_cost>
```

## Build

Compile with GCC:

```bash
gcc -O2 -Wall -Wextra -std=c11 movhex.c -o movhex
```

The program uses functions from the math library, therefore on systems that require explicit linking with `libm` compile with:

```bash
gcc -O2 -Wall -Wextra -std=c11 movhex.c -o movhex -lm
```

## Run

Run the executable and provide commands through standard input:

```bash
./movhex
```

Or use an input file:

```bash
./movhex < input.txt
```

## Repository structure

```text
.
├── README.md
├── main.c
└── specifica_movhex.pdf
```

## Author

**Mohammed Hajjioui**  
Computer Engineering graduate, Politecnico di Milano.
