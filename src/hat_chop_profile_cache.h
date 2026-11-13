/* v7 speed-up: remember finished hair chops per game thread.
 *
 * The game often rebuilds the same Sim several times in a row (CAS, outfit
 * changes, LOD rebuilds). hc_profile_apply is a pure function of its inputs, so
 * when the hair part's written vertex bytes, its triangles, its vertex layout,
 * the hat/head geometry and the cut are all the same, the earlier output is
 * copied instead of recalculated. Keys are 64-bit hashes of all of those bytes.
 * Small LRU, per thread (inside Ctx), so no locking.
 */
#ifndef HAT_CHOP_PROFILE_CACHE_H
#define HAT_CHOP_PROFILE_CACHE_H
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define HC_CACHE_SLOTS 8
#define HC_CACHE_MAX_BYTES (24u * 1024u * 1024u)   /* per thread */

typedef struct {
    uint64_t key;
    int count, nt, stride, used;
    unsigned char *vb;            /* count*stride output bytes */
    uint32_t *F;                  /* nt*3 part-relative output indices */
    HCProfileApplyStats stats;
    uint32_t age;
} HCCacheEntry;

typedef struct {
    HCCacheEntry e[HC_CACHE_SLOTS];
    uint32_t clock;
    size_t bytes;
    uint32_t hits, misses;
} HCProfileCache;

static uint64_t hc_hash_bytes(uint64_t h, const void *data, size_t n)
{
    const unsigned char *p = (const unsigned char *)data;
    size_t i = 0;
    for (; i + 4 <= n; i += 4) {
        uint32_t w;
        memcpy(&w, p + i, 4);
        h = (h ^ w) * UINT64_C(1099511628211);
    }
    for (; i < n; i++) h = (h ^ p[i]) * UINT64_C(1099511628211);
    return h;
}

/* Hat and head geometry of a prepared state (computed once per state). */
static uint64_t hc_shape_geometry_hash(const HCShapeState *s)
{
    uint64_t h = UINT64_C(14695981039346656037);
    h = hc_hash_bytes(h, &s->nhat, sizeof(s->nhat));
    if (s->hatTri && s->nhat > 0) h = hc_hash_bytes(h, s->hatTri, (size_t)s->nhat * 9 * sizeof(double));
    h = hc_hash_bytes(h, &s->nhead, sizeof(s->nhead));
    if (s->headTri && s->nhead > 0) h = hc_hash_bytes(h, s->headTri, (size_t)s->nhead * 9 * sizeof(double));
    return h;
}

static uint64_t hc_cache_key(const HCShapeState *s, uint64_t geometryHash, const unsigned char *vb, int count,
                             int stride, const uint32_t *F, int nt, const HCPayloadLayout *layout)
{
    uint64_t h = UINT64_C(14695981039346656037);
    int head[4] = {count, stride, nt, s->cut};
    h = hc_hash_bytes(h, head, sizeof(head));
    h = hc_hash_bytes(h, &geometryHash, sizeof(geometryHash));
    h = hc_hash_bytes(h, s->plane, sizeof(s->plane));
    h = hc_hash_bytes(h, s->center, sizeof(s->center));
    h = hc_hash_bytes(h, s->scale, sizeof(s->scale));
    h = hc_hash_bytes(h, s->shift, sizeof(s->shift));
    h = hc_hash_bytes(h, &s->fadeDepth, sizeof(s->fadeDepth));
    h = hc_hash_bytes(h, layout, sizeof(*layout));
    h = hc_hash_bytes(h, F, (size_t)nt * 3 * sizeof(uint32_t));
    h = hc_hash_bytes(h, vb, (size_t)count * stride);
    return h;
}

static const HCCacheEntry *hc_cache_find(HCProfileCache *c, uint64_t key, int count, int nt, int stride)
{
    int i;
    for (i = 0; i < HC_CACHE_SLOTS; i++) {
        HCCacheEntry *e = c->e + i;
        if (e->used && e->key == key && e->count == count && e->nt == nt && e->stride == stride) {
            e->age = ++c->clock;
            c->hits++;
            return e;
        }
    }
    c->misses++;
    return NULL;
}

static void hc_cache_drop(HCProfileCache *c, HCCacheEntry *e)
{
    if (!e->used) return;
    c->bytes -= (size_t)e->count * e->stride + (size_t)e->nt * 3 * sizeof(uint32_t);
    free(e->vb); free(e->F);
    memset(e, 0, sizeof(*e));
}

static void hc_cache_store(HCProfileCache *c, uint64_t key, const unsigned char *vb, int count, int stride,
                           const uint32_t *F, int nt, const HCProfileApplyStats *stats)
{
    size_t need = (size_t)count * stride + (size_t)nt * 3 * sizeof(uint32_t);
    HCCacheEntry *slot = NULL;
    int i;
    if (need > HC_CACHE_MAX_BYTES / 2) return;
    for (i = 0; i < HC_CACHE_SLOTS; i++) if (!c->e[i].used) { slot = c->e + i; break; }
    while (!slot || c->bytes + need > HC_CACHE_MAX_BYTES) {
        HCCacheEntry *old = NULL;
        for (i = 0; i < HC_CACHE_SLOTS; i++)
            if (c->e[i].used && (!old || c->e[i].age < old->age)) old = c->e + i;
        if (!old) break;
        hc_cache_drop(c, old);
        if (!slot) slot = old;
    }
    if (!slot || c->bytes + need > HC_CACHE_MAX_BYTES) return;
    slot->vb = (unsigned char *)malloc((size_t)count * stride);
    slot->F = (uint32_t *)malloc((size_t)nt * 3 * sizeof(uint32_t));
    if (!slot->vb || !slot->F) { free(slot->vb); free(slot->F); slot->vb = NULL; slot->F = NULL; return; }
    memcpy(slot->vb, vb, (size_t)count * stride);
    memcpy(slot->F, F, (size_t)nt * 3 * sizeof(uint32_t));
    slot->key = key; slot->count = count; slot->nt = nt; slot->stride = stride;
    slot->stats = *stats; slot->age = ++c->clock; slot->used = 1;
    c->bytes += need;
}
#endif
