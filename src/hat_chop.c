/* Hat Chop 7.3: 32-bit ASI, Steam-style TS3W.exe 1.67 port (PE 52DEC247).
 * Automatically lowers existing SimHair vertices beneath worn accessory hats
 * weighted to b__Hat_grip__. Source packages are never edited.
 * Shared gravity math is also exercised by the offline executable.
 * Settings and log: Game\Bin\HatChop.ini / HatChop.log.
 * ASI loader entry: InitializeASI (Ultimate ASI Loader).
 * Research: HANDOFF_HATCHOP_FOR_CHATGPT.md and research/hat-chop/.
 */
#define _USE_MATH_DEFINES
#include <windows.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include <limits.h>

#ifndef HC_VERSION
#define HC_VERSION 4
#endif

/* the game (Steam-style TS3W.exe 1.67, PE 0x52DEC247) */
#define SITE_FILL        0x005DC197u
#define TARGET_FILL      0x005D3760u
#define SITE_WRITE      0x005D3880u
#define TARGET_WRITE    0x005D1010u
#define GEOM_VTBL        0x00FE32C0u   /* CAS/GeometryResource vtable, ctor at 0x005C6E90 */

#define MB_DRAW_BEGIN    0x134         /* ModelBuilder: drawables, 0xD0 each */
#define MB_DRAW_END      0x138
#define DRAW_SIZE        0xD0
#define DRAW_PARTS_BEGIN 0xA4          /* drawable: part entries, 0x50 each */
#define DRAW_PARTS_END   0xA8
#define PART_SIZE        0x50
#define PART_GEOM        0x10          /* part entry: GeometryResource* (low 2 bits set = none) */

#define G_MAT_BEGIN      0x28          /* GeometryResource: materials, 16 bytes each, shader hash at +4 */
#define G_MAT_END        0x2C
#define G_ELEM_BEGIN     0x38          /* vertex elements, 12 bytes: u32 usage, u32 type, u8 size */
#define G_ELEM_END       0x3C
#define G_SUB_BEGIN      0x48          /* index lists, 12 bytes: u32 data, u32 count, u8 index size (GEOM parser) */
#define G_SUB_END        0x4C
#define G_JOINT_BEGIN    0x58          /* joint name hashes, 4 bytes each */
#define G_JOINT_END      0x5C
#define G_VCOUNT         0x78
#define G_STRIDE         0x7C
#define G_VDATA          0x80

#define SHADER_SIMHAIR   0x84FD7152u   /* FNV32("simhair") */
#define BONE_HAT_GRIP    0x4952E727u   /* FNV32("b__hat_grip__") */
#define USAGE_POSITION   1
#define USAGE_BONES      4
#define USAGE_WEIGHTS    5

/* Preserve the hair outline; extend the cut beyond the brim and retain boundary support. */
#include "hat_chop_gravity.h"
#include "hat_chop_cut.h"
#ifdef HC_PROFILE_BUILD
#include "hat_chop_profile_apply.h"
#ifdef HC_HAT_FIT
#include "hat_chop_profile_cache.h"
#endif
#endif

/* Keep the original packed X, Z and W exactly. Downward quantization adds
 * less than one SHORT4 unit of clearance and cannot move hair sideways. */
static int packed_chop(Shell *s, float *P, int nv, const uint32_t *F, int nt,
                       int16_t *packed, int stride, int *left)
{
    int i, round, moved = 0;
    float *before;
    int16_t *ys;
    if (!P || !packed || !left || nv <= 0 || nv > HC_MAX_HAIR_VERT || stride < 8) {
        if (left) *left = -1;
        return 0;
    }
    before = (float *)malloc(sizeof(float) * 3 * nv);
    ys = (int16_t *)malloc(sizeof(int16_t) * nv);
    if (!before || !ys) { free(before); free(ys); *left = -1; return 0; }
    for (i = 0; i < nv; i++) {
        const int16_t *v = (const int16_t *)((const uint8_t *)packed + (size_t)i * stride);
        if (v[3] <= 0) { free(before); free(ys); *left = -1; return 0; }
        ys[i] = v[1];
    }
    memcpy(before, P, sizeof(float) * 3 * nv);
    for (round = 0; round < 4; round++) {
        chop_mesh(s, P, nv, F, nt, left);
        if (*left < 0) { memcpy(P, before, sizeof(float) * 3 * nv); break; }
        for (i = 0; i < nv; i++) {
            const int16_t *v = (const int16_t *)((const uint8_t *)packed + (size_t)i * stride);
            volatile float storedY = (float)ys[i] / v[3];
            if (P[3*i+1] < before[3*i+1] && P[3*i+1] != storedY) {
                double y = floor((double)P[3*i+1] * v[3]);
                if (y < INT16_MIN || y > INT16_MAX) { *left = -1; break; }
                ys[i] = (int16_t)y;
                P[3*i+1] = (float)ys[i] / v[3];
            }
        }
        if (*left < 0) { memcpy(P, before, sizeof(float) * 3 * nv); break; }
        *left = gravity_crossing_count(s, P, nv, F, nt);
        if (*left < 0) { memcpy(P, before, sizeof(float) * 3 * nv); break; }
        if (*left == 0) break;
    }
    for (i = 0; i < nv; i++) moved += P[3*i+1] != before[3*i+1];
    free(before); free(ys);
    return moved;
}

#ifdef HATCHOP_TEST
/* Offline executable uses the same SHORT4 quantization and packed_chop path as the hook.
 * Inputs: hat triangles float3x3, hair float3, indices uint32x3; outputs final and baseline float3. */
static void *load(const char *path, long *sz)
{
    FILE *f = fopen(path, "rb");
    void *d;
    if (!f) return NULL;
    if (fseek(f, 0, SEEK_END) || (*sz = ftell(f)) <= 0 || *sz > 36000000 || fseek(f, 0, SEEK_SET)) {
        fclose(f); return NULL;
    }
    d = malloc(*sz);
    if (!d || fread(d, 1, *sz, f) != (size_t)*sz) { fclose(f); free(d); return NULL; }
    fclose(f);
    return d;
}

static int save(const char *path, const float *P, int nv)
{
    FILE *f = fopen(path, "wb");
    int ok;
    if (!f) return 0;
    ok = fwrite(P, sizeof(float)*3, nv, f) == (size_t)nv;
    if (fclose(f)) ok = 0;
    return ok;
}

#ifdef HC_PROFILE_BUILD
#include "hat_chop_profile_test_main.h"
#else
int main(int argc, char **argv)
{
    long sh = 0, sp = 0, si = 0;
    float *ht = NULL, *P = NULL, *originalP = NULL;
    uint32_t *F = NULL;
    int16_t *packed = NULL;
    double *tri = NULL;
    static Shell s;
    int ntri, nv, nt, i, k, moved, left, rc = 2;
    if (argc != 6 && argc != 7) { puts("usage: hat_tris.bin hair_pos.bin hair_idx.bin out.bin baseline.bin [out_idx.bin]"); return 2; }
    ht = (float *)load(argv[1], &sh);
    P = (float *)load(argv[2], &sp);
    F = (uint32_t *)load(argv[3], &si);
    if (!ht || !P || !F || sh%36 || sp%12 || si%12) { puts("invalid mesh files"); goto done; }
    ntri = (int)(sh / 36); nv = (int)(sp / 12); nt = (int)(si / 12);
    if (nv > 200000 || nt > 1000000 || ntri > 200000) goto done;
    tri = (double *)malloc(sizeof(double)*9*ntri);
    packed = (int16_t *)malloc(sizeof(int16_t)*4*nv);
    if (!tri || !packed) goto done;
    for (i = 0; i < 9*ntri; i++) tri[i] = ht[i];
    for (i = 0; i < nv; i++) {
        double m = 0, w;
        for (k = 0; k < 3; k++) {
            double v = P[3*i+k];
            if (!isfinite(v) || fabs(v) > 100) goto done;
            if (fabs(v) > m) m = fabs(v);
        }
        w = 0x7ffd / (int)(ceil(m) < 1 ? 1 : ceil(m));
        packed[4*i+3] = (int16_t)w;
        for (k = 0; k < 3; k++) {
            packed[4*i+k] = (int16_t)lrintf(P[3*i+k]*(float)w);
            P[3*i+k] = (float)packed[4*i+k] / (float)w;
        }
    }
    if (!save(argv[5], P, nv)) goto done;
    originalP = (float *)malloc(sizeof(float)*3*nv);
    if (!originalP) goto done;
    memcpy(originalP, P, sizeof(float)*3*nv);
    if (!shell_build(&s, tri, ntri)) { tri = NULL; puts("hat field failed"); goto done; }
    tri = NULL; /* shell owns it */
    moved = packed_chop(&s, P, nv, F, nt, packed, 8, &left);
    printf("gravity packed hat %d tris hair %d verts %d tris moved %d edges still crossing %d\n",
           ntri, nv, nt, moved, left);
    if (left || !save(argv[4], P, nv)) { rc = 1; goto done; }
    if (argc == 7) {
        FILE *f;
        unsigned char *hidden = (unsigned char *)calloc(nt, 1);
        int dropped = 0, ok;
        if (!hidden || !build_hidden_mask(&s, originalP, nv, F, nt, hidden)) {
            free(hidden); puts("hair cut mask failed"); goto done;
        }
        for (i = 0; i < nt; i++)
            if (hidden[i]) {
                F[3*i+1] = F[3*i+2] = F[3*i]; dropped++;
            }
        free(hidden);
        f = fopen(argv[6], "wb");
        if (!f) goto done;
        ok = fwrite(F, sizeof(uint32_t)*3, nt, f) == (size_t)nt;
        if (fclose(f) || !ok) goto done;
        printf("hidden upper hair triangles chopped %d\n", dropped);
    }
    rc = 0;
done:
    shell_free(&s); free(tri); free(ht); free(P); free(originalP); free(F); free(packed);
    return rc;
}
#endif
#else
/* ------------------------------------------------------------------ plugin */
static CRITICAL_SECTION g_logLock;
static volatile LONG g_started = 0;
static char g_dir[MAX_PATH];
static int g_enabled = 1, g_logParts = 0;
static volatile LONG g_installed = 0, g_layoutBad = 0;
static volatile LONG g_lines = 0, g_builds = 0;
static DWORD g_tls = TLS_OUT_OF_INDEXES;

typedef int(__attribute__((thiscall)) *fill_fn)(void *self, void *info);
typedef int(__stdcall *write_fn)(int, int, int, int, int, int, int, int, int, int, int, int, int);
static fill_fn o_fill = (fill_fn)TARGET_FILL;
static write_fn o_write = (write_fn)TARGET_WRITE;

static void hc_log(const char *fmt, ...)
{
    char path[MAX_PATH], line[1024];
    va_list ap;
    SYSTEMTIME t;
    FILE *f;
    if (InterlockedIncrement(&g_lines) > 3000)
        return;
    va_start(ap, fmt);
    _vsnprintf(line, sizeof(line) - 1, fmt, ap);
    line[sizeof(line) - 1] = 0;
    va_end(ap);
    GetLocalTime(&t);
    EnterCriticalSection(&g_logLock);
    _snprintf(path, sizeof(path), "%sHatChop.log", g_dir);
    f = fopen(path, "a");
    if (f) {
        fprintf(f, "%02d:%02d:%02d.%03d %s\n", t.wHour, t.wMinute, t.wSecond, t.wMilliseconds, line);
        fclose(f);
    }
    LeaveCriticalSection(&g_logLock);
}

/* ---- safe-ish reads: every object is checked before its fields are trusted */
static int memory_range(const void *p, size_t n, int write)
{
    uintptr_t a = (uintptr_t)p, end;
    if (a < 0x10000 || n > UINTPTR_MAX - a) return 0;
    end = a + n;
    do {
        MEMORY_BASIC_INFORMATION mb;
        DWORD protect;
        uintptr_t next;
        if (!VirtualQuery((const void *)a, &mb, sizeof(mb)) || mb.State != MEM_COMMIT ||
            (mb.Protect & (PAGE_GUARD | PAGE_NOACCESS))) return 0;
        protect = mb.Protect & 0xff;
        if (protect != PAGE_READONLY && protect != PAGE_READWRITE && protect != PAGE_WRITECOPY &&
            protect != PAGE_EXECUTE_READ && protect != PAGE_EXECUTE_READWRITE && protect != PAGE_EXECUTE_WRITECOPY) return 0;
        if (write && protect != PAGE_READWRITE && protect != PAGE_WRITECOPY &&
            protect != PAGE_EXECUTE_READWRITE && protect != PAGE_EXECUTE_WRITECOPY) return 0;
        next = (uintptr_t)mb.BaseAddress + mb.RegionSize;
        if (next <= a) return 0;
        a = next;
    } while (a < end);
    return 1;
}
static int readable(const void *p, size_t n) { return memory_range(p, n, 0); }

#define U32(p, off) (*(uint32_t *)((uint8_t *)(p) + (off)))

#ifdef HC_PROFILE_BUILD
#define HC_CAPTURED_LAYOUT_DEFINED 1
typedef struct {
    uint32_t geom, vb;
    int count, stride;
    HCPayloadLayout layout;
} HCCapturedLayout;
#endif

typedef struct {
    int count, stride, posOff, bonesOff, weightsOff, weightsSize;
    const uint8_t *data;
    const uint32_t *joints;
    int njoints;
    uint32_t shader;
    uint32_t subs, subsEnd;            /* index lists */
} GeomInfo;

static int geom_info(const void *g, GeomInfo *gi)
{
    uint32_t eb, ee, jb, je, mb_, me, off = 0, i;
    memset(gi, 0, sizeof(*gi));
    gi->posOff = gi->bonesOff = gi->weightsOff = -1;
    if (((uintptr_t)g & 3) || !readable(g, 0x88) || U32(g, 0) != GEOM_VTBL)
        return 0;
    gi->count = (int)U32(g, G_VCOUNT);
    gi->stride = (int)U32(g, G_STRIDE);
    gi->data = (const uint8_t *)U32(g, G_VDATA);
    if (gi->count <= 0 || gi->count > 200000 || gi->stride <= 0 || gi->stride > 256)
        return 0;
    if (!readable(gi->data, (size_t)gi->count * gi->stride))
        return 0;
    eb = U32(g, G_ELEM_BEGIN);
    ee = U32(g, G_ELEM_END);
    if (ee < eb || (ee - eb) % 12 || ee - eb > 12 * 32 || !readable((void *)eb, ee - eb))
        return 0;
    for (i = eb; i < ee; i += 12) {
        uint32_t usage = *(uint32_t *)i;
        int size = *(uint8_t *)(i + 8);
        if (usage == USAGE_POSITION && size == 12 && gi->posOff < 0) gi->posOff = (int)off;
        if (usage == USAGE_BONES && size == 4 && gi->bonesOff < 0) gi->bonesOff = (int)off;
        if (usage == USAGE_WEIGHTS && (size == 16 || size == 4) && gi->weightsOff < 0) {
            gi->weightsOff = (int)off;
            gi->weightsSize = size;
        }
        if (size <= 0 || off + (uint32_t)size > (uint32_t)gi->stride) return 0;
        off += size;
    }
    if ((int)off != gi->stride || gi->posOff < 0)
        return 0;
    jb = U32(g, G_JOINT_BEGIN);
    je = U32(g, G_JOINT_END);
    if (je >= jb && (je - jb) % 4 == 0 && je - jb <= 4 * 256 && (je == jb || readable((void *)jb, je - jb))) {
        gi->joints = (const uint32_t *)jb;
        gi->njoints = (int)((je - jb) / 4);
    }
    mb_ = U32(g, G_MAT_BEGIN);
    me = U32(g, G_MAT_END);
    if (me > mb_ && (me - mb_) % 16 == 0 && readable((void *)mb_, 16))
        gi->shader = *(uint32_t *)(mb_ + 4);
    gi->subs = U32(g, G_SUB_BEGIN);
    gi->subsEnd = U32(g, G_SUB_END);
    if (gi->subsEnd < gi->subs || (gi->subsEnd - gi->subs) % 12 || gi->subsEnd - gi->subs > 12 * 16 ||
        (gi->subsEnd > gi->subs && !readable((void *)gi->subs, gi->subsEnd - gi->subs)))
        gi->subs = gi->subsEnd = 0;
    return 1;
}

/* All triangles of a GEOM as vertex indices (3 per triangle). -> count, or -1 on bad data. Caller frees *out. */
static int geom_tris(const GeomInfo *gi, uint32_t **out)
{
    uint32_t s;
    int total = 0, n = 0;
    *out = NULL;
    for (s = gi->subs; s < gi->subsEnd; s += 12) {
        uint32_t cnt = *(uint32_t *)(s + 4);
        int isz = *(uint8_t *)(s + 8);
        if ((isz != 2 && isz != 4) || cnt % 3 || cnt > 3000000 || !readable((void *)*(uint32_t *)s, (size_t)cnt * isz))
            return -1;
        if (cnt > 3000000u - (uint32_t)total) return -1;
        total += (int)cnt;
    }
    if (!total) return 0;
    *out = (uint32_t *)malloc(sizeof(uint32_t) * total);
    if (!*out) return -1;
    for (s = gi->subs; s < gi->subsEnd; s += 12) {
        const uint8_t *d = (const uint8_t *)*(uint32_t *)s;
        uint32_t cnt = *(uint32_t *)(s + 4), j;
        int isz = *(uint8_t *)(s + 8);
        for (j = 0; j < cnt; j++) {
            uint32_t v = isz == 2 ? ((const uint16_t *)d)[j] : ((const uint32_t *)d)[j];
            if (v >= (uint32_t)gi->count) { free(*out); *out = NULL; return -1; }
            (*out)[n++] = v;
        }
    }
    return n / 3;
}

/* A hat accessory: not hair, and most of its vertices follow b__Hat_grip__. */
static int is_hat(const GeomInfo *gi)
{
    int j, hat = -1, i, on = 0;
    if (gi->shader == SHADER_SIMHAIR || !gi->joints)
        return 0;
    for (j = 0; j < gi->njoints; j++)
        if (gi->joints[j] == BONE_HAT_GRIP)
            hat = j;
    if (hat < 0)
        return 0;
    if (gi->njoints == 1)
        return 1;
    if (gi->bonesOff < 0 || gi->weightsOff < 0)
        return 0;
    for (i = 0; i < gi->count; i++) {
        const uint8_t *v = gi->data + (size_t)i * gi->stride;
        const uint8_t *b = v + gi->bonesOff;
        float best = -1;
        int bi = -1, k;
        for (k = 0; k < 4; k++) {
            float w = gi->weightsSize == 16 ? ((const float *)(v + gi->weightsOff))[k] : (v + gi->weightsOff)[k] / 255.0f;
            if (w > best) { best = w; bi = b[k]; }
        }
        if (bi == hat) on++;
    }
    return on * 10 >= gi->count * 8;
}

/* v7.2 full hat: headwear under which The Sims 4 hides ALL the hair (hijabs, wraps, bonnets, turbans).
 * The TS4 Hats import marks such a part with three vertices at one hidden spot inside the head (the last
 * three of the mesh): x and z hold these exact bit patterns. Any bones; no other part has it. */
#define HC_MARK_X_BITS 0x38F0C0DEu
#define HC_MARK_Z_BITS 0xB8FA11CAu
static int hc_full_marker(const GeomInfo *gi)
{
    int i;
    if (gi->shader == SHADER_SIMHAIR || gi->count < 3)
        return 0;
    for (i = gi->count - 3; i < gi->count; i++) {
        const uint8_t *v = gi->data + (size_t)i * gi->stride + gi->posOff;
        uint32_t x, z;
        memcpy(&x, v, 4);
        memcpy(&z, v + 8, 4);
        if (x != HC_MARK_X_BITS || z != HC_MARK_Z_BITS)
            return 0;
    }
    return 1;
}

/* ---- per-thread build context: the hats of the ModelBuilder being filled */
typedef struct {
    const void *mb;                    /* set only while FillDrawable runs */
    uint32_t key[3];                   /* which hat set the shell was built for */
    int hats, chopFailed;
    int fullHat, fullLogs;             /* v7.2: a full hat is worn (all hair is hidden, no chop) */
    uint32_t skipKey;                  /* v7: last logged set of non-hat accessories */
    Shell shell;
#ifdef HC_PROFILE_BUILD
    HCShapeState shape;
    int shapeReady, captureCount, captureFailed;
    uint32_t headKey;
    HCCapturedLayout captured[256];
#ifdef HC_HAT_FIT
    HCProfileCache cache;              /* finished chops of recent identical builds */
    uint64_t shapeGeomHash;
#endif
#endif
} Ctx;

#ifdef HC_PROFILE_BUILD
static void hc_profile_capture(int geom, int elems, int stride, int vb, int boneByteFlag);
static void hc_profile_prepare_runtime(Ctx *c, const void *mb);
#endif

static Ctx *ctx_get(void)
{
    Ctx *c = (Ctx *)TlsGetValue(g_tls);
    if (!c) {
        c = (Ctx *)calloc(1, sizeof(Ctx));
        if (c && !TlsSetValue(g_tls, c)) { free(c); return NULL; }
    }
    return c;
}

static void scan_hats(Ctx *c, const void *mb)
{
    uint32_t db, de, d, pb, pe, p, key[3] = {0, 0, 0};
    double *tri = NULL;
    int ntri = 0, cap = 0, hats = 0, full = 0;
#ifdef HC_HAT_FIT
    struct { uint32_t g, count; int nt, start; double cover; } parts[64];
    int nparts = 0;
#endif
    c->mb = mb;
    c->fullHat = 0;
    if (!readable(mb, MB_DRAW_END + 4))
        { c->hats = 0; return; }
    db = U32(mb, MB_DRAW_BEGIN);
    de = U32(mb, MB_DRAW_END);
    if (de < db || (de - db) % DRAW_SIZE || de - db > DRAW_SIZE * 64 || !readable((void *)db, de - db))
        { c->hats = 0; return; }
    /* pass 1: which hats (the key decides whether the cached shell still fits) */
    for (d = db; d < de; d += DRAW_SIZE) {
        pb = U32(d, DRAW_PARTS_BEGIN);
        pe = U32(d, DRAW_PARTS_END);
        if (pe < pb || (pe - pb) % PART_SIZE || pe - pb > PART_SIZE * 256 || (pe > pb && !readable((void *)pb, pe - pb)))
            continue;
        for (p = pb; p < pe; p += PART_SIZE) {
            uint32_t g = U32(p, PART_GEOM);
            GeomInfo gi;
            uint32_t *F;
            int nt, i, k;
            if (!g || (g & 3) || !geom_info((void *)g, &gi))
                continue;
#ifdef HC_HAT_FIT
            if (hc_full_marker(&gi))
                full = 1;
#endif
            if (!is_hat(&gi))
                continue;
            nt = geom_tris(&gi, &F);
            if (nt <= 0) { free(F); continue; }
            if (nt > 200000 - ntri) { free(F); free(tri); c->hats = 0; return; }
            hats++;
            key[0] ^= g;
            key[1] += (uint32_t)gi.count;
            key[2] += (uint32_t)nt * 2654435761u;
            if (ntri + nt > cap) {
                double *nt2;
                cap = (ntri + nt) * 2;
                nt2 = (double *)realloc(tri, sizeof(double) * 9 * cap);
                if (!nt2) { free(F); free(tri); c->hats = 0; return; }
                tri = nt2;
            }
            for (i = 0; i < nt; i++)
                for (k = 0; k < 3; k++) {
                    const float *q = (const float *)(gi.data + (size_t)F[3 * i + k] * gi.stride + gi.posOff);
                    tri[9 * (ntri + i) + 3 * k] = q[0];
                    tri[9 * (ntri + i) + 3 * k + 1] = q[1];
                    tri[9 * (ntri + i) + 3 * k + 2] = q[2];
                }
#ifdef HC_HAT_FIT
            if (nparts < 64) {
                parts[nparts].g = g; parts[nparts].count = (uint32_t)gi.count; parts[nparts].nt = nt;
                parts[nparts].start = ntri;
                parts[nparts].cover = hc_fit_crown_cover_any_age(tri + (size_t)9 * ntri, nt);
                nparts++;
            }
#endif
            ntri += nt;
            free(F);
        }
    }
#ifdef HC_HAT_FIT
    if (full) {                        /* nothing to chop: hk_fill hides the whole hairstyle */
        c->fullHat = 1;
        c->hats = hats;
        free(tri);
        return;
    }
    /* Only accessories that cover the scalp top are hats for the chop. If no single
     * part does but all hat-bone parts together do (a hat split in pieces), keep all. */
    if (hats && nparts == hats) {
        int keep = 0, j, out = 0, all;
        for (j = 0; j < nparts; j++) keep += parts[j].cover >= HC_FIT_MIN_COVER;
        all = !keep && hc_fit_crown_cover_any_age(tri, ntri) >= HC_FIT_MIN_COVER;
        key[0] = key[1] = key[2] = 0;
        for (j = 0; j < nparts; j++) {
            int use = all || parts[j].cover >= HC_FIT_MIN_COVER;
            if (!use) continue;
            if (out != parts[j].start)
                memmove(tri + (size_t)9 * out, tri + (size_t)9 * parts[j].start, sizeof(double) * 9 * parts[j].nt);
            out += parts[j].nt;
            key[0] ^= parts[j].g;
            key[1] += parts[j].count;
            key[2] += (uint32_t)parts[j].nt * 2654435761u;
        }
        ntri = out;
        hats = keep ? keep : all ? hats : 0;
        {
            /* log once per change of worn accessories, not on every rebuild */
            uint32_t skipKey = 2166136261u;
            for (j = 0; j < nparts; j++)
                if (!all && parts[j].cover < HC_FIT_MIN_COVER) skipKey = (skipKey ^ parts[j].g) * 16777619u;
            if (skipKey != 2166136261u && skipKey != c->skipKey)
                for (j = 0; j < nparts; j++)
                    if (!all && parts[j].cover < HC_FIT_MIN_COVER)
                        hc_log("hat part %08X skipped: covers %d%% of the scalp top (not a hat, hair stays)",
                               parts[j].g, (int)(parts[j].cover * 100 + .5));
            c->skipKey = skipKey;
        }
    }
#endif
    c->hats = hats;
    if (!hats) {
#ifdef HC_PROFILE_BUILD
        c->shapeReady = 0;
#endif
        free(tri); return;
    }
    if (c->shell.ok && memcmp(c->key, key, sizeof(key)) == 0) { free(tri); return; }
    memcpy(c->key, key, sizeof(key));
#ifdef HC_PROFILE_BUILD
    c->shapeReady = 0;
#endif
    {
        DWORD t0 = GetTickCount();
        int ok = shell_build(&c->shell, tri, ntri);      /* takes over tri */
        if (g_logParts || !ok)
            hc_log("hat set %08X: %d hat part(s), %d triangles, shell %s (pivot %.3f %.3f %.3f) in %lu ms", key[0],
                   hats, ntri, ok ? "ok" : "FAILED", c->shell.px, c->shell.py, c->shell.pz,
                   (unsigned long)(GetTickCount() - t0));
    }
}

static int chop_hidden_indices(Ctx *c, void *self, void *info);
#ifdef HC_HAT_FIT
static int hc_full_hat_hide_hair(Ctx *c, void *self, void *info);
#endif

static int __attribute__((thiscall)) hk_fill(void *self, void *info)
{
    Ctx *c = ctx_get();
    int r;
    if (c) {
        c->chopFailed = 0;
#ifdef HC_PROFILE_BUILD
        c->captureCount = c->captureFailed = 0;
#endif
        scan_hats(c, self);
#ifdef HC_HAT_FIT
        if (c->fullHat) {
            r = o_fill(self, info);
            if (g_enabled && !g_layoutBad)
                hc_full_hat_hide_hair(c, self, info);
            c->mb = NULL;
            return r;
        }
#endif
#ifdef HC_PROFILE_BUILD
        if (c->hats && c->shell.ok) {
            hc_profile_prepare_runtime(c, self);
            if (!c->shapeReady) c->chopFailed = 1;
        }
#endif
    }
    r = o_fill(self, info);
    if (c && c->hats && c->shell.ok && !c->chopFailed && !g_layoutBad && g_enabled)
        chop_hidden_indices(c, self, info);
    if (c)
        c->mb = NULL;
    return r;
}

static void decode(const int16_t *s, float *p)
{
    float w = (float)s[3];
    p[0] = s[0] / w;
    p[1] = s[1] / w;
    p[2] = s[2] / w;
}

static int __stdcall hk_write(int geom, int elems, int stride, int vb, int a5, int a6, int a7, int a8, int a9, int a10,
                              int a11, int a12, int a13)
{
#ifdef HC_PROFILE_BUILD
    hc_profile_capture(geom, elems, stride, vb, a13);
    return o_write(geom, elems, stride, vb, a5, a6, a7, a8, a9, a10, a11, a12, a13);
#else
    int ret = o_write(geom, elems, stride, vb, a5, a6, a7, a8, a9, a10, a11, a12, a13);
    Ctx *c = (Ctx *)TlsGetValue(g_tls);
    GeomInfo gi;
    int vstride = stride & 0xFFFF, i, nchk, nt, moved, left;
    float *P;
    uint32_t *F;
    if (!c || !c->mb || !c->hats || !c->shell.ok || g_layoutBad)
        return ret;
    if (!geom_info((void *)geom, &gi) || gi.shader != SHADER_SIMHAIR)
        return ret;
    if (vstride < 8 || ret - vb != gi.count * vstride || !memory_range((void *)vb, (size_t)gi.count * vstride, 1)) {
        c->chopFailed = 1;
        hc_log("skip hair %08X: written %d bytes, expected %d x %d", geom, ret - vb, gi.count, vstride);
        return ret;
    }
    /* the written positions must be the hair's own (format check) */
    nchk = gi.count < 8 ? gi.count : 8;
    for (i = 0; i < nchk; i++) {
        float q[3];
        const float *src = (const float *)(gi.data + (size_t)i * gi.stride + gi.posOff);
        decode((const int16_t *)(vb + i * vstride), q);
        if (!isfinite(q[0]) || !isfinite(q[1]) || !isfinite(q[2]) ||
            fabsf(q[0] - src[0]) > 2e-3f || fabsf(q[1] - src[1]) > 2e-3f || fabsf(q[2] - src[2]) > 2e-3f) {
            c->chopFailed = 1;
            InterlockedExchange(&g_layoutBad, 1);
            hc_log("LAYOUT MISMATCH vertex %d: buffer %.4f %.4f %.4f, hair %.4f %.4f %.4f -> Hat Chop stays off", i, q[0],
                   q[1], q[2], src[0], src[1], src[2]);
            return ret;
        }
    }
    if (!g_enabled) {
        if (g_logParts)
            hc_log("hair %08X: %d vertices under %d hat(s) (log only)", geom, gi.count, c->hats);
        return ret;
    }
    nt = geom_tris(&gi, &F);
    if (nt < 0) {
        c->chopFailed = 1;
        hc_log("skip hair %08X: unreadable triangles", geom);
        return ret;
    }
    P = (float *)malloc(sizeof(float) * 3 * gi.count);
    if (!P) { c->chopFailed = 1; free(F); return ret; }
    for (i = 0; i < gi.count; i++) {
        const int16_t *v = (const int16_t *)(vb + i * vstride);
        if (v[3] <= 0) { c->chopFailed = 1; free(P); free(F); return ret; }
        decode(v, P + 3 * i);
    }
    {
        DWORD t0 = GetTickCount();
        moved = packed_chop(&c->shell, P, gi.count, F, nt, (int16_t *)vb, vstride, &left);
        if (left != 0) c->chopFailed = 1;
        /* Commit only a verified packed result; failed jobs keep the game's buffer. */
        if (left == 0)
            for (i = 0; i < gi.count; i++) {
                int16_t *v = (int16_t *)(vb + i * vstride);
                float originalY = (float)v[1] / v[3];
                if (P[3*i+1] < originalY)
                    v[1] = (int16_t)lrint((double)P[3*i+1] * v[3]);
            }
        if (InterlockedIncrement(&g_builds) <= 60 || g_logParts || left != 0)
            hc_log("hair %08X: %d of %d vertices chopped below the hat, %d edge(s) still crossing, %lu ms", geom, moved,
                   gi.count, left, (unsigned long)(GetTickCount() - t0));
    }
    free(P);
    free(F);
    return ret;
#endif
}

#ifdef HC_PROFILE_BUILD
#include "hat_chop_profile_indices.h"
#else
#include "hat_chop_indices.h"
#endif

/* ---- install */
static int patch_call(uint32_t site, uint32_t expect_target, void *hook)
{
    uint8_t *p = (uint8_t *)site;
    DWORD old;
    int32_t rel;
    if (!readable(p, 5) || p[0] != 0xE8)
        return 0;
    rel = *(int32_t *)(p + 1);
    if (site + 5 + rel != expect_target)
        return 0;
    if (!VirtualProtect(p, 5, PAGE_EXECUTE_READWRITE, &old))
        return 0;
    *(int32_t *)(p + 1) = (int32_t)((uint32_t)hook - (site + 5));
    VirtualProtect(p, 5, old, &old);
    FlushInstructionCache(GetCurrentProcess(), p, 5);
    return 1;
}

static float ini_float(const char *path, const char *key, const char *def, float lo, float hi)
{
    char buf[64];
    float v;
    GetPrivateProfileStringA("HatChop", key, def, buf, sizeof(buf), path);
    v = (float)atof(buf);
    if (!isfinite(v) || v < lo || v > hi) v = (float)atof(def);
    return v;
}

static void read_ini(void)
{
    char path[MAX_PATH];
    _snprintf(path, sizeof(path), "%sHatChop.ini", g_dir);
    g_enabled = GetPrivateProfileIntA("HatChop", "Enabled", 1, path);
    g_logParts = GetPrivateProfileIntA("HatChop", "LogParts", 0, path);
    g_downClear = ini_float(path, "Clearance", "0.0005", 0.0001f, 0.01f);
    g_gridStep = ini_float(path, "GridStep", "0.002", 0.001f, 0.005f);
}

/* UAL can load an ASI before the packed executable has restored these sites. */
static int calls_ready(void)
{
    const uint8_t *f = (const uint8_t *)SITE_FILL, *w = (const uint8_t *)SITE_WRITE;
    return readable((void *)GEOM_VTBL, 0x64) && U32((void *)GEOM_VTBL, 0x60) == 0x005C6F20u &&
        readable(f, 5) && readable(w, 5) && f[0] == 0xE8 && w[0] == 0xE8 &&
        SITE_FILL + 5 + *(const int32_t *)(f+1) == TARGET_FILL &&
        SITE_WRITE + 5 + *(const int32_t *)(w+1) == TARGET_WRITE;
}

static int install(void)
{
    if (!calls_ready()) return 0;
    g_tls = TlsAlloc();
    if (g_tls == TLS_OUT_OF_INDEXES) { hc_log("OFF: no thread-local slot"); return -1; }
    if (!patch_call(SITE_FILL, TARGET_FILL, (void *)hk_fill)) {
        TlsFree(g_tls); g_tls = TLS_OUT_OF_INDEXES;
        hc_log("OFF: FillDrawable call changed during initialization"); return -1;
    }
    if (!patch_call(SITE_WRITE, TARGET_WRITE, (void *)hk_write)) {
        int undone = patch_call(SITE_FILL, (uint32_t)hk_fill, (void *)TARGET_FILL);
        if (undone) { TlsFree(g_tls); g_tls = TLS_OUT_OF_INDEXES; }
        hc_log("OFF: part-writer call changed; first hook rollback %s", undone ? "ok" : "FAILED");
        return -1;
    }
    InterlockedExchange(&g_installed, 1);
    hc_log("ON: gravity hooks installed (%s)", g_enabled ? "automatic hair chopping" : "log only");
    return 1;
}

static DWORD WINAPI wait_install(LPVOID ignored)
{
    DWORD start = GetTickCount();
    (void)ignored;
    do {
        int result = install();
        if (result) return 0;
        Sleep(100);
    } while (GetTickCount() - start < 120000);
    hc_log("OFF: expected game code was not ready within 120 seconds (unsupported build or another hook)");
    return 0;
}

static void start_plugin(void)
{
    uint8_t *exe = (uint8_t *)GetModuleHandleW(NULL);
    IMAGE_NT_HEADERS *nt;
    HMODULE pinned;
    char *slash;
    HANDLE worker;
    int result;
    if (InterlockedCompareExchange(&g_started, 1, 0)) return;
    InitializeCriticalSection(&g_logLock);
    GetModuleFileNameA(NULL, g_dir, sizeof(g_dir));
    g_dir[sizeof(g_dir)-1] = 0;
    slash = strrchr(g_dir, '\\');
    if (!slash) return;
    slash[1] = 0;
    read_ini();
    if (!readable(exe, sizeof(IMAGE_DOS_HEADER)) || ((IMAGE_DOS_HEADER *)exe)->e_magic != IMAGE_DOS_SIGNATURE ||
        ((IMAGE_DOS_HEADER *)exe)->e_lfanew <= 0 || ((IMAGE_DOS_HEADER *)exe)->e_lfanew > 0x100000) return;
    nt = (IMAGE_NT_HEADERS *)(exe + ((IMAGE_DOS_HEADER *)exe)->e_lfanew);
    if (!readable(nt, sizeof(*nt)) || nt->Signature != IMAGE_NT_SIGNATURE) return;
#ifdef HC_SUBVERSION
    hc_log("Hat Chop %d.%d ASI starting: base %p timestamp %08lX Enabled=%d Clearance=%.6f GridStep=%.6f",
           HC_VERSION, HC_SUBVERSION, exe, (unsigned long)nt->FileHeader.TimeDateStamp, g_enabled, g_downClear, g_gridStep);
#else
    hc_log("Hat Chop %d ASI starting: base %p timestamp %08lX Enabled=%d Clearance=%.6f GridStep=%.6f",
           HC_VERSION, exe, (unsigned long)nt->FileHeader.TimeDateStamp, g_enabled, g_downClear, g_gridStep);
#endif
    if ((uintptr_t)exe != 0x400000 || nt->FileHeader.TimeDateStamp != 0x52DEC247 ||
        nt->FileHeader.Machine != IMAGE_FILE_MACHINE_I386) {
        hc_log("OFF: needs TS3W.exe 1.67 Steam-style (base 00400000, timestamp 52DEC247)"); return;
    }
    if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN,
                           (LPCWSTR)(uintptr_t)&start_plugin, &pinned)) {
        hc_log("OFF: could not keep the module loaded"); return;
    }
    result = install();
    if (result) return;
    hc_log("Waiting for packed game code to become ready");
    worker = CreateThread(NULL, 0, wait_install, NULL, 0, NULL);
    if (worker) CloseHandle(worker);
    else hc_log("OFF: could not start initialization worker");
}

/* Standard cdecl ASI entry point; MonoPatcher exports are compatibility fallbacks. */
__declspec(dllexport) void __cdecl InitializeASI(void) { start_plugin(); }
typedef void (*AddInternalCall)(const char *name, const void *method);
__declspec(dllexport) void InitMonoPatcherPlugin(void *context) { (void)context; start_plugin(); }
__declspec(dllexport) void ICallSetup(AddInternalCall add) { (void)add; start_plugin(); }

#ifndef HATCHOP_HOST_TEST
BOOL WINAPI DllMain(HINSTANCE h, DWORD reason, LPVOID reserved)
{
    (void)reserved;
    if (reason == DLL_PROCESS_ATTACH) { DisableThreadLibraryCalls(h); }
    return TRUE;
}
#endif
#endif
