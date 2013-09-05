#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <time.h>
#include <assert.h>


#define WIDTH 100
#define HEIGHT 43


typedef char screen_t[HEIGHT][WIDTH + 1];

typedef struct { int x, y; } point_t;

typedef struct { point_t p0, p1, d, s; int err; } bresenham_t;


/**
 * Compare function for sorting points indexes lexicographically
 */
static int compare(void *restrict v, const void *restrict p0, const void *restrict p1) {

    point_t *vec = v;

    const size_t i0 = *(const size_t*)p0;
    const size_t i1 = *(const size_t*)p1;

    if (vec[i0].x != vec[i1].x) {

        return vec[i0].x - vec[i1].x;
    }

    return vec[i0].y - vec[i1].y;
}


/**
 * Initialize the data for the Bresenham algorithm between two given point_ts
 */
static void init_bresenham(const point_t p0, const point_t p1, bresenham_t *b) {

    b->p0 = p0;
    b->p1 = p1;

    b->d.x = abs(p0.x - p1.x);
    b->d.y = abs(p0.y - p1.y);

    b->s.x = p0.x < p1.x ? 1 : -1;
    b->s.y = p0.y < p1.y ? 1 : -1;

    b->err = b->d.x - b->d.y;
}


/**
 * Iterate the Bresenham algorithm to get the next pixel
 */
static bool iter_bresenham(bresenham_t *b) {

    if (b->p0.x == b->p1.x && b->p0.y == b->p1.y) return true;

    int e2 = 2 * b->err;

    if (e2 > -b->d.y) {

        b->err  -= b->d.y;
        b->p0.x += b->s.x;
    }

    if (b->p0.x == b->p1.x && b->p0.y == b->p1.y) return true;

    if (e2 < b->d.x) {

        b->err  += b->d.x;
        b->p0.y += b->s.y;
    }

    return false;
}


/**
 * Maintain a min_y and max_y coords for a line-delimited column
 */
static void get_column_limits(const int x, int *restrict min_y, int *restrict max_y, bresenham_t *b) {

    while (b->p0.x == x) {

        if (b->p0.y < *min_y) *min_y = b->p0.y;
        if (b->p0.y > *max_y) *max_y = b->p0.y;

        if (iter_bresenham(b)) break;
    }
}


/**
 * Draw a vertical-based trapezoid in screen
 */
static void draw_tapezoid(const int x0, const int x1, bresenham_t *b0, bresenham_t *b1, screen_t *restrict s) {

    for (int x = x0; x <= x1; x++) {

        int min_y = INT_MAX;
        int max_y = INT_MIN;

        get_column_limits(x, &min_y, &max_y, b0);
        get_column_limits(x, &min_y, &max_y, b1);

        for (int y = min_y; y <= max_y; y++) {

            if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {

                (*s)[y][x] = '@';
            }
        }
    }
}


/**
 * True if b-a to c-a is a counter-clockwise turn
 */
static inline bool ccw(const point_t a, const point_t b, const point_t c) {

    return (b.x-a.x) * (c.y-a.y) > (b.y-a.y) * (c.x-a.x);
}


/**
 * Draw a simple polygon in screen
 */
static void draw_simple_polygon(const size_t n, const point_t *v, screen_t *screen) {

    /* Compute sort indexes */

    size_t s2v[n];

    for (size_t i = 0; i < n; i++) s2v[i] = i;

    qsort_r(s2v, n, sizeof(*s2v), (void*)v, compare);

    /* Current trapezoids linked list */

    struct trapezoid {
        struct segment { size_t first, last; bresenham_t b; } s0, s1;
        struct trapezoid *next;
    } head;

    head.next = NULL;

    /* Sweep line */

    for (size_t i = 0; i < n - 1; i++) {

        size_t curr = s2v[i];
        size_t till = s2v[i + 1];
        size_t pred = (curr + n-1) % n;
        size_t succ = (curr + 1) % n;

        bool prev = compare((void*)v, &curr, &pred) > 0;
        bool next = compare((void*)v, &curr, &succ) < 0;

        /* Case 1: last vertex */

        if (prev && !next) {

            for (struct trapezoid *t = &head, *u = NULL; t->next; t = t->next) {

                bool first  = t->next->s0.last == curr;
                bool second = t->next->s1.last == curr;
                bool internal = first && second;

                if (first || second) {

                    /* Case 1.1: external vertex (merge vertices) */

                    if (!u && !internal) {
                        u = t;
                        continue;
                    }

                    if (!internal) {
                        if (u->next->s1.last == curr) {
                            u->next->s1 = t->next->s1;
                        } else {
                            u->next->s0 = t->next->s0;
                        }
                    }

                    /* Case 1.2: internal vertex (delete vertex) */

                    void *aux = t->next->next;
                    free(t->next);
                    t->next = aux;

                    break;
                }
            }
        }

        /* Case 2: middle vertex (modify vertex) */

        if (prev == next) {

            for (struct trapezoid *t = head.next; t; t = t->next) {

                bool first  = t->s0.last == curr;
                bool second = t->s1.last == curr;

                if (first || second) {

                    struct segment *s = first ? &t->s0 : &t->s1;

                    s->first = curr;
                    s->last = prev ? succ : pred;
                    init_bresenham(v[s->first], v[s->last], &s->b);

                    break;
                }
            }
        }

        /* Case 3: first vertex */

        if (!prev && next) {

            if (ccw(v[curr], v[pred], v[succ])) {

                int a = pred;
                pred = succ;
                succ = a;
            }

            struct trapezoid *new = malloc(sizeof(*new));
            struct segment top, pot;

            top.first = curr; top.last = pred;
            pot.first = curr; pot.last = succ;

            init_bresenham(v[curr], v[pred], &top.b);
            init_bresenham(v[curr], v[succ], &pot.b);

            /* Case 3.1: internal vertex (split vertex) */

            bool internal = false;

            for (struct trapezoid *t = head.next; t; t = t->next) {

                if (ccw(v[t->s0.first], v[curr], v[t->s0.last]) &&
                    ccw(v[t->s1.first], v[t->s1.last], v[curr]))
                {
                    new->s0 = t->s0;
                    new->s1 = top;
                    t->s0 = pot;

                    internal = true;
                    break;
                }
            }

            /* Case 3.2: external vertex (create vertex) */

            if (!internal) {

                new->s0 = top;
                new->s1 = pot;
            }

            new->next = head.next;
            head.next = new;
        }

        /* Draw trapezoids */

        for (struct trapezoid *t = head.next; t; t = t->next) {

            draw_tapezoid(v[curr].x, v[till].x, &t->s0.b, &t->s1.b, screen);
        }
    }

    /* Free linked list */

    for (struct trapezoid *t = head.next, *a; t; t = a) {

        a = t->next;
        free(t);
    }
}


/**
 * Rotate a polygon phi degrees about the origin, and center in the screen
 */
static void rotate_center(const double phi, const size_t n, const point_t *restrict from, point_t *restrict to) {

    double s = sin(phi);
    double c = cos(phi);

    for (size_t i = 0; i < n; i++) {

        to[i].x =  WIDTH / 2 + round(c * from[i].x - s * from[i].y);
        to[i].y = HEIGHT / 2 + round(s * from[i].x + c * from[i].y);
    }
}


/**
 * Fill screen with spaces
 */
static void clear_screen(screen_t *screen) {

    memset(screen, ' ', sizeof(*screen));

    for (size_t h = 0; h < HEIGHT; h++) {

        (*screen)[h][WIDTH] = '\n';
    }
}


int main(void) {

    #define SIZE (sizeof(polygon) / sizeof(*polygon))

    point_t polygon[] = {{-13, -13}, {0, -7}, {13, -13}, {7, 0}, {13, 13}, {0, 7}, {-13, 13}, {-7, 0}};
    point_t rotated[SIZE];
    screen_t screen;

    for (size_t t = 0; ; t++) {

        clear_screen(&screen);

        double phi = M_PI * sin(t / 10.0);

        rotate_center(phi, SIZE, polygon, rotated);

        draw_simple_polygon(SIZE, rotated, &screen);

        printf("%.*s", (int)sizeof(screen), (char*)screen);

        nanosleep((struct timespec[]){{0, 100000000L}}, NULL);
    }

    return 0;
}

