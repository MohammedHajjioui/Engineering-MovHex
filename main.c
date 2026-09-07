#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define MAX_ROUTES 5
#define IDX(x, y, map) ((y) * (map)->colonne + (x))

// Accesso cella CSR
#define CELL_AT_IDX(M, idx)     ((M)->cells[(idx)])
#define CELL_AT_XY(M, x, y)     ((M)->cells[IDX((x),(y),(M))])

#define LB_CACHE_CAP 4
#define CMAX 101    //per travel cost2


typedef struct {
    int dest_x;
    int dest_y;
    uint8_t costo;
} AirRoute;

// ====== 7.1 — Strutture CSR ======
typedef struct {
    uint32_t offset;    // indice iniziale nel pool globale delle rotte
    uint16_t deg;       // numero di rotte uscenti (0..5)
    uint8_t  costo_out; // costo_uscita [0..100]
    uint8_t  _pad;      // (allineamento)
} Cell;

typedef struct {
    int righe;
    int colonne;

    // celle e pool rotte in formato CSR
    Cell     *cells;     // size = N = righe*colonne
    uint32_t *air_dest;  // size = R = somma dei gradi
    uint8_t  *air_cost;  // size = R

    int cellsN;          // N
    int airR;            // R
} Map;


////////////////// firme
int get_neighbors(int x, int y, int out_x[6], int out_y[6], const Map* mappa);

/////////////////////////////////// gestione mappa
// Range rotte uscenti di una cella (CSR)
static inline void cell_air_range(const Map *M, int idx, uint32_t *start, uint16_t *deg) {
    const Cell *c = &M->cells[idx];
    *start = c->offset;
    *deg   = c->deg;
}

void destroy_map(Map* mappa) {
    if (!mappa) return;

    // libera pool CSR (nessuna free per-singola rotta!)
    free(mappa->air_dest);  mappa->air_dest = NULL;
    free(mappa->air_cost);  mappa->air_cost = NULL;

    // libera celle
    free(mappa->cells);     mappa->cells    = NULL;

    // azzera metadati
    mappa->righe   = 0;
    mappa->colonne = 0;
    mappa->cellsN  = 0;
    mappa->airR    = 0;
}

int init_map(Map* mappa, int colonne, int righe) {
    if (!mappa) return 0;
    if (righe <= 0 || colonne <= 0) {
        fprintf(stderr, "Errore: dimensioni mappa non valide (%d righe, %d colonne)\n", righe, colonne);
        return 0;
    }

    // libera eventuale mappa precedente
    destroy_map(mappa);

    const int N = righe * colonne;
    Cell *cells = (Cell*)malloc(sizeof(Cell) * (size_t)N);
    if (!cells) {
        fprintf(stderr, "Errore: allocazione celle fallita\n");
        return 0;
    }

    // inizializza celle: costo_out=1, deg=0, offset=0
    for (int i = 0; i < N; ++i) {
        cells[i].offset    = 0u;
        cells[i].deg       = 0u;
        cells[i].costo_out = 1u;
        cells[i]._pad      = 0u;
    }

    // inizialmente nessuna rotta: i pool restano NULL/0
    mappa->righe    = righe;
    mappa->colonne  = colonne;
    mappa->cells    = cells;
    mappa->air_dest = NULL;
    mappa->air_cost = NULL;
    mappa->cellsN   = N;
    mappa->airR     = 0;

    return 1; // OK
}

//////////////////////////////////////////////////  gestione mappa esagonale    //////////////////////////////////////////

void offset_to_cube(int col, int row, int *cx, int *cy, int *cz) {
    *cx = col - (row - (row & 1)) / 2;
    *cz = row;
    *cy = -*cx - *cz;
}

int hex_distance(int col1, int row1, int col2, int row2) {
    int x1, y1, z1;
    int x2, y2, z2;

    offset_to_cube(col1, row1, &x1, &y1, &z1);
    offset_to_cube(col2, row2, &x2, &y2, &z2);

    return (abs(x1 - x2) + abs(y1 - y2) + abs(z1 - z2)) / 2;
}



////////////////////////////////////////////// toggle air route //////////////////////////////////////////////////
// Assunti: MAX_ROUTES = 5
static int ensure_air_capacity(Map *M, int newR) {
    if (newR <= M->airR) return 1; // cap = size esatta; cresciamo di +N se vuoi meno realloc
    // crescita: +max(64, N/8) per ammortizzare (opzionale). Qui andiamo esatto per semplicità:
    uint32_t *nd = realloc(M->air_dest, sizeof(uint32_t) * (size_t)newR);
    if (!nd) return 0;
    M->air_dest = nd;

    uint8_t *nc  = realloc(M->air_cost, sizeof(uint8_t) * (size_t)newR);
    if (!nc) return 0;
    M->air_cost = nc;
    return 1;
}

// Sposta a destra il segmento [pos, M->airR) di 'delta' posizioni
static void air_shift_right(Map *M, int pos, int delta) {
    if (delta <= 0) return;
    for (int i = M->airR - 1; i >= pos; --i) {
        M->air_dest[i + delta] = M->air_dest[i];
        M->air_cost[i + delta] = M->air_cost[i];
    }
    M->airR += delta;
}

// Sposta a sinistra il segmento [pos, M->airR) di 1 posizione (rimozione singola)
static void air_shift_left_one(Map *M, int pos) {
    for (int i = pos + 1; i < M->airR; ++i) {
        M->air_dest[i - 1] = M->air_dest[i];
        M->air_cost[i - 1] = M->air_cost[i];
    }
    M->airR -= 1;
}

// Aggiorna offset di tutte le celle successive a 'cell_idx' di +delta (pos può essere offset+deg)
static void bump_offsets(Map *M, int cell_idx, int delta) {
    const int N = M->cellsN;
    for (int i = cell_idx + 1; i < N; ++i) {
        M->cells[i].offset = (uint32_t)((int)M->cells[i].offset + delta);
    }
}

int toggle_air_route(int x1, int y1, int x2, int y2, Map* M) {
    if (x1 < 0 || y1 < 0 || x2 < 0 || y2 < 0 ||
        x1 >= M->colonne || y1 >= M->righe ||
        x2 >= M->colonne || y2 >= M->righe) return 0;

    const int from = IDX(x1, y1, M);
    const int to   = IDX(x2, y2, M);

    Cell *c = &M->cells[from];
    uint32_t start = c->offset;
    uint16_t deg   = c->deg;

    // cerca se esiste già
    int found = -1;
    for (uint16_t k = 0; k < deg; ++k) {
        int pos = (int)start + (int)k;
        if ((int)M->air_dest[pos] == to) { found = pos; break; }
    }

    if (found >= 0) {
        // RIMUOVI la rotta
        air_shift_left_one(M, found);
        c->deg -= 1;
        bump_offsets(M, from, -1);
        return 1; // OK
    }

    // non esiste: prova ad aggiungere
    if (deg >= MAX_ROUTES) return 0; // KO: pieno

    // costo medio arrotondato per difetto = (costo_out + sum(rotte))/ (deg+1)
    int somma = c->costo_out;
    for (uint16_t k = 0; k < deg; ++k) somma += M->air_cost[start + k];
    int media = somma / (deg + 1);
    if (media < 0) media = 0; else if (media > 100) media = 100;

    // Inserisci in coda al blocco della cella: posizione = start + deg
    const int ins_pos = (int)start + (int)deg;

    if (!ensure_air_capacity(M, M->airR + 1)) return 0;
    // Prima apriamo uno spazio a destra da ins_pos
    air_shift_right(M, ins_pos, 1);

    // Scriviamo il nuovo elemento
    M->air_dest[ins_pos] = (uint32_t)to;
    M->air_cost[ins_pos] = (uint8_t)media;

    // Aggiorna deg della cella e offset delle successive
    c->deg += 1;
    bump_offsets(M, from, +1);

    return 1; // OK
}


/////////////////////////////////////////////////////// travel cost 2 //////////////////////////////////////////////////////////

// workspace riutilizzabile per le query che verra cancellato in caso di modifiche
static int32_t  *best_cost_per_cell = NULL;   // ex g_dist
static uint32_t *seen_generation    = NULL;   // ex g_tag
static uint32_t  current_generation = 1;      // ex g_gen
static int       workspace_cells    = 0;      // ex g_N


static int *bucket_head = NULL;      // size CMAX
static int *bucket_next = NULL;      // size N
static int  bucket_pos  = 0;         // dist % CMAX
static int  bucket_next_capacity = 0; // <<< spostato a livello di file

// Da chiamare all’inizio di ogni travel_cost
static inline void buckets_reset(void) {
    if (!bucket_head) return;
    for (int i = 0; i < CMAX; ++i) bucket_head[i] = -1;
    bucket_pos = 0;
}

int ensure_workspace(int cell_count) {
    if (cell_count != workspace_cells) {
        free(best_cost_per_cell); best_cost_per_cell = NULL;
        free(seen_generation);    seen_generation    = NULL;

        best_cost_per_cell = (int32_t*)malloc(sizeof(int32_t) * cell_count);
        seen_generation    = (uint32_t*)malloc(sizeof(uint32_t) * cell_count);
        if (!best_cost_per_cell || !seen_generation) {
            free(best_cost_per_cell); best_cost_per_cell = NULL;
            free(seen_generation);    seen_generation    = NULL;
            workspace_cells = 0;
            return 0;
        }
        workspace_cells = cell_count;
    }
    return 1;
}

int ensure_buckets(int cell_count) {
    if (!bucket_head) {
        bucket_head = (int*)malloc(sizeof(int) * CMAX);
        if (!bucket_head) return 0;
    }
    if (bucket_next == NULL || bucket_next_capacity != cell_count) {
        free(bucket_next); bucket_next = NULL;
        bucket_next = (int*)malloc(sizeof(int) * cell_count);
        if (!bucket_next) { bucket_next_capacity = 0; return 0; }
        bucket_next_capacity = cell_count;
    }
    return 1;
}

// === RESET completo dello stato Dial ===
static void reset_travelcost_state(void) {
    // workspace
    free(best_cost_per_cell);  best_cost_per_cell = NULL;
    free(seen_generation);     seen_generation    = NULL;
    workspace_cells = 0;
    current_generation = 1;

    // buckets
    free(bucket_next);  bucket_next = NULL;
    free(bucket_head);  bucket_head = NULL;
    bucket_pos = 0;
    bucket_next_capacity = 0; // <<< importante
}

static inline void bucket_push(int node_idx, int node_dist) {
    int b = node_dist % CMAX;
    bucket_next[node_idx] = bucket_head[b];
    bucket_head[b] = node_idx;
}

static inline int bucket_pop(void) {
    int scanned = 0;
    while (bucket_head[bucket_pos] == -1) {
        bucket_pos = (bucket_pos + 1) % CMAX;
        if (++scanned > CMAX) return -1; // nessun nodo
    }
    int v = bucket_head[bucket_pos];
    bucket_head[bucket_pos] = bucket_next[v];
    return v;
}


// ======== travel_cost con Dial + generation id ========
int travel_cost(int x1, int y1, int x2, int y2, const Map* M) {
    if (x1 == x2 && y1 == y2) return 0;
    if (x1 < 0 || y1 < 0 || x2 < 0 || y2 < 0 ||
        x1 >= M->colonne || y1 >= M->righe ||
        x2 >= M->colonne || y2 >= M->righe)
        return -1;

    const int cell_count = M->righe * M->colonne;
    if (!ensure_workspace(cell_count)) return -1;
    if (!ensure_buckets(cell_count))   return -1;
    if (!best_cost_per_cell || !seen_generation || !bucket_next) return -1;

    // start intransitabile e senza rotte positive => -1
    const int src_idx = IDX(x1, y1, M);
    const int dst_idx = IDX(x2, y2, M);

    const uint8_t start_cost = CELL_AT_IDX(M, src_idx).costo_out;
    if (start_cost == 0) {
        uint32_t s; uint16_t d;
        cell_air_range(M, src_idx, &s, &d);
        int has_valid = 0;
        for (uint16_t k = 0; k < d; ++k) if (M->air_cost[s+k] > 0) { has_valid = 1; break; }
        if (!has_valid) return -1;
    }

    // init
    if (current_generation == UINT16_MAX) {
        memset(seen_generation, 0, sizeof(*seen_generation) * (size_t)workspace_cells);
        current_generation = 0;
    }
    current_generation++;
    buckets_reset();

    best_cost_per_cell[src_idx] = 0;
    seen_generation[src_idx]    = current_generation;
    bucket_push(src_idx, 0);

    for (;;) {
        if (bucket_head[bucket_pos] == -1) {
            int scanned = 0;
            while (bucket_head[bucket_pos] == -1) {
                bucket_pos = (bucket_pos + 1) % CMAX;
                if (++scanned > CMAX) return -1;
            }
        }

        const int u_idx = bucket_pop();
        const int du = (seen_generation[u_idx] == current_generation) ? best_cost_per_cell[u_idx] : INT32_MAX;
        if (u_idx == dst_idx) return du;
        if (du == INT32_MAX) continue;

        const int ux = u_idx % M->colonne;
        const int uy = u_idx / M->colonne;

        // terra: paga costo_uscita della cella u
        const uint8_t cost_uscita = CELL_AT_IDX(M, u_idx).costo_out;
        if (cost_uscita > 0) {
            const int is_odd = (uy & 1);
            static const int DX[2][6] = {{-1,+1,-1, 0,-1, 0},{-1,+1, 0,+1, 0,+1}};
            static const int DY[2][6] = {{ 0, 0,+1,+1,-1,-1},{ 0, 0,+1,+1,-1,-1}};
            for (int k = 0; k < 6; ++k) {
                const int vx = ux + DX[is_odd][k];
                const int vy = uy + DY[is_odd][k];
                if ((unsigned)vx >= (unsigned)M->colonne || (unsigned)vy >= (unsigned)M->righe) continue;

                const int v_idx = IDX(vx, vy, M);
                const int nd = du + (int)cost_uscita;
                if (seen_generation[v_idx] != current_generation || nd < best_cost_per_cell[v_idx]) {
                    best_cost_per_cell[v_idx] = nd;
                    seen_generation[v_idx]    = current_generation;
                    bucket_push(v_idx, nd);
                }
            }
        }

        // rotte aeree: paga il costo della rotta
        uint32_t start; uint16_t deg;
        cell_air_range(M, u_idx, &start, &deg);
        for (uint16_t k = 0; k < deg; ++k) {
            const int v_idx = (int)M->air_dest[start + k];
            const int nd = du + (int)M->air_cost[start + k];
            if (seen_generation[v_idx] != current_generation || nd < best_cost_per_cell[v_idx]) {
                best_cost_per_cell[v_idx] = nd;
                seen_generation[v_idx]    = current_generation;
                bucket_push(v_idx, nd);
            }
        }
    }
}






/////////////////////////////////////////////////////// change cost /////////////////////////////////////////////////////////////


// Restituisce fino a 6 vicini validi di (x, y) nella mappa
// Ritorna il numero di vicini trovati
int get_neighbors(int x, int y, int out_x[6], int out_y[6], const Map* mappa) {
    // Offset per colonne pari
    static const int dx_even[6] = { -1, 1, -1, 0, -1, 0 };
    static const int dy_even[6] = { 0, 0, 1, 1, -1, -1 };

    // Offset per colonne dispari
    static const int dx_odd[6]  = { -1, 1, 0, 1, 0, 1 };
    static const int dy_odd[6]  = { 0, 0, 1, 1, -1, -1 };

    const int* dx = (y % 2 == 0) ? dx_even : dx_odd;
    const int* dy = (y % 2 == 0) ? dy_even : dy_odd;

    int count = 0;
    for (int i = 0; i < 6; i++) {
        int nx = x + dx[i];
        int ny = y + dy[i];

        // Controllo confini mappa
        if (nx >= 0 && nx < mappa->colonne &&
            ny >= 0 && ny < mappa->righe) {
            out_x[count] = nx;
            out_y[count] = ny;
            count++;
            }
    }
    return count;
}


int change_cost_bfs(int cx, int cy, int v, int r, Map* M) {
    if (r <= 0) return 1; // niente da fare

    const int W = M->colonne, H = M->righe, N = W * H;

    // --- bitset visited (riuso statico)
    static uint8_t *visited = NULL; static int visited_bytes = 0;
    const int need_bytes = (N + 7) >> 3;
    if (need_bytes > visited_bytes) {
        uint8_t *nb = realloc(visited, need_bytes);
        if (!nb) return 0;
        visited = nb; visited_bytes = need_bytes;
    }
    memset(visited, 0, visited_bytes);
    #define VIS_SET(i)   ( visited[(i)>>3] |=  (uint8_t)(1u << ((i)&7)) )
    #define VIS_GET(i) ( (visited[(i)>>3] &   (uint8_t)(1u << ((i)&7))) != 0 )

    // ---coda di indici (riuso statico)
    static int *queue = NULL; static int qcap = 0;
    if (qcap < N) {
        int *nq = realloc(queue, sizeof(int) * (size_t)N);
        if (!nq) return 0;
        queue = nq; qcap = N;
    }

    int front = 0, back = 0;
    const int src = IDX(cx, cy, M);
    queue[back++] = src; VIS_SET(src);

    for (int dist = 0; dist < r && front < back; ++dist) {
        const int level_end = back;
        for (; front < level_end; ++front) {
            const int u = queue[front];
            const int ux = u % W, uy = u / W;

            // delta = floor(v * (r - dist)/r)
            const int delta = (int)floorf((float)v * ((float)(r - dist) / (float)r));

            // aggiorna costo uscita
            Cell *cu = &M->cells[u];
            int nuovo = (int)cu->costo_out + delta;
            if (nuovo < 0) nuovo = 0; else if (nuovo > 100) nuovo = 100;
            cu->costo_out = (uint8_t)nuovo;

            // aggiorna rotte aeree uscenti (CSR)
            uint32_t s; uint16_t d;
            cell_air_range(M, u, &s, &d);
            for (uint16_t k = 0; k < d; ++k) {
                int nc = (int)M->air_cost[s + k] + delta;
                if (nc < 0) nc = 0; else if (nc > 100) nc = 100;
                M->air_cost[s + k] = (uint8_t)nc;
            }

            // vicini (esagoni)
            const int is_odd = (uy & 1);
            static const int DX[2][6] = {{-1,+1,-1, 0,-1, 0},{-1,+1, 0,+1, 0,+1}};
            static const int DY[2][6] = {{ 0, 0,+1,+1,-1,-1},{ 0, 0,+1,+1,-1,-1}};
            for (int i = 0; i < 6; ++i) {
                const int vx = ux + DX[is_odd][i];
                const int vy = uy + DY[is_odd][i];
                if ((unsigned)vx >= (unsigned)W || (unsigned)vy >= (unsigned)H) continue;
                const int vidx = IDX(vx, vy, M);
                if (!VIS_GET(vidx)) { VIS_SET(vidx); queue[back++] = vidx; }
            }
        }
    }
    return 1;
}



///////////////////////////////////////////parsing I/O ////////////////////////////////////////////
// per semplicità la metto globale. eventualmente posso metterla nel main
Map mappa;

// FAST I/O: leggere interi molto velocemente
static inline int fast_read_int(int *out){
    int c = getchar();
    // salta tutto finché non trovo '-' o una cifra
    while (c!='-' && (c<'0' || c>'9')) {
        if (c==EOF) return 0;
        c = getchar();
    }
    int neg = (c=='-');
    if (neg) c = getchar();
    int x = 0;
    while (c>='0' && c<='9'){
        x = x*10 + (c - '0');
        c = getchar();//_unlocked();
    }
    *out = neg ? -x : x;
    return 1;
}


void process_commands(void) {
    char command[32];

    while (scanf("%31s", command) == 1) {
        if (strcmp(command, "init") == 0) {
            int cols, rows;
            if (!fast_read_int(&cols) || !fast_read_int(&rows)) break;

            reset_travelcost_state();
            if (init_map(&mappa, cols, rows)) {
                puts("OK");
                break;
            }
        } else if (strcmp(command, "travel_cost") == 0) {
            int xp, yp, xd, yd;
            if (!fast_read_int(&xp) || !fast_read_int(&yp)
             || !fast_read_int(&xd) || !fast_read_int(&yd)) break;
            puts("-1"); // nel primo ciclo rispondi placeholder come prima
        } else {
            puts("KO");
        }
    }

    while (scanf("%31s", command) == 1) {
        if (strcmp(command, "travel_cost") == 0) {
            int xp, yp, xd, yd;
            if (!fast_read_int(&xp) || !fast_read_int(&yp) || !fast_read_int(&xd) || !fast_read_int(&yd)) break;
            int res = travel_cost(xp, yp, xd, yd, &mappa);
            printf("%d\n", res);
        } else if (strcmp(command, "init") == 0) {
            int cols, rows;
            if (!fast_read_int(&cols) || !fast_read_int(&rows)) break;

            reset_travelcost_state();
            destroy_map(&mappa);
            puts(init_map(&mappa, cols, rows) ? "OK" : "Errore");
        } else if (strcmp(command, "change_cost") == 0) {
            int x, y, v, r;
            if (!fast_read_int(&x) || !fast_read_int(&y)
             || !fast_read_int(&v) || !fast_read_int(&r)) break;

            if (x < 0 || y < 0 || x >= mappa.colonne || y >= mappa.righe || r <= 0 || v < -10 || v > 10) {
                puts("KO"); continue;
            }
            if (v == 0) { puts("OK"); continue; }

            int ok = change_cost_bfs(x, y, v, r, &mappa);
            if (ok) { puts("OK"); }
            else     puts("KO");
        } else if (strcmp(command, "toggle_air_route") == 0) {
            int x1,y1,x2,y2;
            if (!fast_read_int(&x1) || !fast_read_int(&y1)
             || !fast_read_int(&x2) || !fast_read_int(&y2)) break;
            if (toggle_air_route(x1,y1,x2,y2,&mappa)) {
                puts("OK");
            } else puts("KO");
        } else {
            fprintf(stderr, "Comando sconosciuto: %s\n", command);
            break;
        }
    }

}

int main(void) {
    setvbuf(stdout, NULL, _IOFBF, 1<<20);

    mappa.cells = NULL;
    mappa.air_dest = NULL;
    mappa.air_cost = NULL;
    mappa.righe = 0;
    mappa.colonne = 0;
    mappa.cellsN = 0;
    mappa.airR = 0;

    process_commands();
    destroy_map(&mappa);
    return 0;
}


