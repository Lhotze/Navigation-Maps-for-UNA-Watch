// Offline Navigation - routing, step 2: contraction hierarchy.
//
//   build_ch <graph dir> <profile 0|1> <out file> [--test N]
//
// profile: 0 bicycle, 1 pedestrian (cost columns 2p, 2p+1 of edges.bin).
//
// Reads the graph written by extract_graph.py, orders the nodes with the usual
// edge-difference heuristic, adds shortcuts, and writes for every node the arcs
// that lead to HIGHER-ranked nodes:
//    outUp : arcs  v -> x  with rank(x) > rank(v)     (forward search)
//    inUp  : arcs  x -> v  with rank(x) > rank(v)     (backward search)
// A query then runs two small searches that only go "up" and meet at the top.
//
// 'via' of an arc: high bit set = an original edge (bit 0 = direction, bits
// 1..30 = edge number); otherwise the middle node of a shortcut.
//
// Output (little endian):  'CHG1', u32 n, u32 nOutUp, u32 nInUp,
//    rank[n]  (u32), outOff[n+1], inOff[n+1] (u32), outUp[] and inUp[] (to, w, via)
//
// --test N runs N random queries and compares them with plain Dijkstra.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <queue>
#include <random>
#include <string>
#include <vector>

using u32 = uint32_t;
static const u32 INF = 0xFFFFFFFFu;
static const u32 ORIG = 0x80000000u;

#pragma pack(push, 1)
struct Edge {
    u32 a, b;
    float length;
    u32 shape;
    uint16_t nshape;
    uint8_t cls, flags;
    u32 name;
    u32 cost[4];
};
#pragma pack(pop)
static_assert(sizeof(Edge) == 40, "edge size");

struct Arc { u32 to, w, via; };

static std::vector<std::vector<Arc>> g_out, g_in;     // the graph that shrinks while contracting
static u32 N;

// --- witness search ---------------------------------------------------------------
struct Witness {
    std::vector<u32> dist, stamp;
    u32 cur = 0;
    explicit Witness(u32 n) : dist(n, INF), stamp(n, 0) {}
    // Is there a path src -> dst of weight <= limit, not through 'skip'?
    bool run(u32 src, u32 dst, u32 skip, u32 limit, int maxSettled) {
        ++cur;
        using P = std::pair<u32, u32>;
        std::priority_queue<P, std::vector<P>, std::greater<P>> q;
        dist[src] = 0; stamp[src] = cur;
        q.push({0, src});
        int settled = 0;
        while (!q.empty()) {
            auto [d, u] = q.top(); q.pop();
            if (stamp[u] != cur || d > dist[u]) continue;
            if (u == dst) return d <= limit;
            if (d > limit || ++settled > maxSettled) return false;
            for (const Arc& a : g_out[u]) {
                if (a.to == skip) continue;
                u32 nd = d + a.w;
                if (nd > limit) continue;
                if (stamp[a.to] != cur || nd < dist[a.to]) {
                    stamp[a.to] = cur; dist[a.to] = nd;
                    q.push({nd, a.to});
                }
            }
        }
        return false;
    }
};

struct Shortcut { u32 from, to, w, via; };

// Shortcuts that contracting v would need (witness-checked). apply=false only counts.
static void shortcutsFor(Witness& wit, u32 v, std::vector<Shortcut>& out, int maxSettled) {
    out.clear();
    for (const Arc& in : g_in[v]) {
        u32 u = in.to;
        if (u == v) continue;
        // largest candidate from this u
        u32 maxw = 0;
        for (const Arc& o : g_out[v]) if (o.to != u && o.to != v) maxw = std::max(maxw, in.w + o.w);
        if (maxw == 0) continue;
        // one search per (u, x) is simple; limit keeps it cheap
        for (const Arc& o : g_out[v]) {
            u32 x = o.to;
            if (x == u || x == v) continue;
            u32 w = in.w + o.w;
            if (!wit.run(u, x, v, w, maxSettled)) out.push_back({u, x, w, v});
        }
    }
}

static void removeArcsTo(std::vector<Arc>& list, u32 who) {
    list.erase(std::remove_if(list.begin(), list.end(), [&](const Arc& a) { return a.to == who; }), list.end());
}

static void addArc(std::vector<Arc>& list, u32 to, u32 w, u32 via) {
    for (Arc& a : list) {
        if (a.to == to) {
            if (w < a.w) { a.w = w; a.via = via; }
            return;
        }
    }
    list.push_back({to, w, via});
}

// --- query (also used for the test) ------------------------------------------------
struct CH {
    u32 n = 0;
    std::vector<u32> rank, outOff, inOff;
    std::vector<Arc> outUp, inUp;
};

struct Query {
    const CH& ch;
    std::vector<u32> df, db, sf, sb;
    u32 cur = 0;
    long settledTotal = 0;
    explicit Query(const CH& c) : ch(c), df(c.n, INF), db(c.n, INF), sf(c.n, 0), sb(c.n, 0) {}

    u32 run(u32 s, u32 t) {
        ++cur;
        using P = std::pair<u32, u32>;
        std::priority_queue<P, std::vector<P>, std::greater<P>> qf, qb;
        df[s] = 0; sf[s] = cur; qf.push({0, s});
        db[t] = 0; sb[t] = cur; qb.push({0, t});
        u32 best = INF;
        auto step = [&](std::priority_queue<P, std::vector<P>, std::greater<P>>& q, std::vector<u32>& d,
                        std::vector<u32>& st, std::vector<u32>& od, std::vector<u32>& ost,
                        const std::vector<u32>& off, const std::vector<Arc>& arcs) {
            auto [dd, u] = q.top(); q.pop();
            if (st[u] != cur || dd > d[u]) return;
            ++settledTotal;
            if (ost[u] == cur && od[u] != INF) best = std::min(best, dd + od[u]);
            for (u32 i = off[u]; i < off[u + 1]; ++i) {
                const Arc& a = arcs[i];
                u32 nd = dd + a.w;
                if (st[a.to] != cur || nd < d[a.to]) {
                    st[a.to] = cur; d[a.to] = nd;
                    q.push({nd, a.to});
                }
            }
        };
        for (;;) {
            const bool f = !qf.empty() && qf.top().first < best;
            const bool b = !qb.empty() && qb.top().first < best;
            if (!f && !b) break;
            if (f && (!b || qf.top().first <= qb.top().first))
                step(qf, df, sf, db, sb, ch.outOff, ch.outUp);
            else
                step(qb, db, sb, df, sf, ch.inOff, ch.inUp);
        }
        return best;
    }
};

int main(int argc, char** argv) {
    if (argc < 4) {
        std::fprintf(stderr, "usage: build_ch <graph dir> <profile 0|1> <out file> [--test N]\n");
        return 1;
    }
    std::string dir = argv[1];
    int prof = std::atoi(argv[2]);
    std::string outPath = argv[3];
    int tests = 0;
    for (int i = 4; i + 1 < argc; ++i) if (!std::strcmp(argv[i], "--test")) tests = std::atoi(argv[i + 1]);
    auto t0 = std::chrono::steady_clock::now();
    auto secs = [&] { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); };

    // --- load
    FILE* f = std::fopen((dir + "/nodes.bin").c_str(), "rb");
    if (!f) { std::perror("nodes.bin"); return 1; }
    u32 nNodes; if (std::fread(&nNodes, 4, 1, f) != 1) return 1; std::fclose(f);
    f = std::fopen((dir + "/edges.bin").c_str(), "rb");
    if (!f) { std::perror("edges.bin"); return 1; }
    u32 nEdges; if (std::fread(&nEdges, 4, 1, f) != 1) return 1;
    std::vector<Edge> edges(nEdges);
    if (std::fread(edges.data(), sizeof(Edge), nEdges, f) != nEdges) return 1;
    std::fclose(f);
    N = nNodes;

    // --- original graph for this profile
    g_out.assign(N, {}); g_in.assign(N, {});
    std::vector<std::vector<Arc>> origOut(N);
    long nArcs = 0;
    for (u32 i = 0; i < nEdges; ++i) {
        const Edge& e = edges[i];
        if (e.a == e.b) continue;
        u32 cf = e.cost[2 * prof], cb = e.cost[2 * prof + 1];
        if (cf != INF) { addArc(g_out[e.a], e.b, cf, ORIG | (i << 1)); addArc(g_in[e.b], e.a, cf, ORIG | (i << 1)); ++nArcs; }
        if (cb != INF) { addArc(g_out[e.b], e.a, cb, ORIG | (i << 1) | 1); addArc(g_in[e.a], e.b, cb, ORIG | (i << 1) | 1); ++nArcs; }
    }
    origOut = g_out;
    std::fprintf(stderr, "profile %d: %u nodes, %ld arcs (%.1fs)\n", prof, N, nArcs, secs());

    // every arc ever created (original + shortcuts), for the final split by rank
    struct Rec { u32 from, to, w, via; };
    std::vector<Rec> all;
    for (u32 u = 0; u < N; ++u) for (const Arc& a : g_out[u]) all.push_back({u, a.to, a.w, a.via});

    // --- contraction
    Witness wit(N);
    std::vector<u32> level(N, 0), delN(N, 0), rank(N, INF);
    std::vector<Shortcut> sc;
    auto priority = [&](u32 v) -> long {
        shortcutsFor(wit, v, sc, 40);
        long removed = (long)g_in[v].size() + (long)g_out[v].size();
        return 4L * ((long)sc.size() - removed) + 2L * delN[v] + (long)level[v];
    };
    using PQ = std::pair<long, u32>;
    std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> pq;
    for (u32 v = 0; v < N; ++v) pq.push({priority(v), v});
    std::fprintf(stderr, "priorities ready (%.1fs)\n", secs());

    u32 nextRank = 0;
    long nShortcuts = 0;
    while (!pq.empty()) {
        auto [p, v] = pq.top(); pq.pop();
        if (rank[v] != INF) continue;
        long np = priority(v);
        if (!pq.empty() && np > pq.top().first) { pq.push({np, v}); continue; }

        shortcutsFor(wit, v, sc, 500);        // final decision with a wider search
        rank[v] = nextRank++;
        for (const Shortcut& s : sc) {
            addArc(g_out[s.from], s.to, s.w, s.via);
            addArc(g_in[s.to], s.from, s.w, s.via);
            all.push_back({s.from, s.to, s.w, s.via});
            ++nShortcuts;
        }
        for (const Arc& a : g_in[v])  { removeArcsTo(g_out[a.to], v); ++delN[a.to]; level[a.to] = std::max(level[a.to], level[v] + 1); }
        for (const Arc& a : g_out[v]) { removeArcsTo(g_in[a.to], v);  ++delN[a.to]; level[a.to] = std::max(level[a.to], level[v] + 1); }
        std::vector<u32> nb;
        for (const Arc& a : g_in[v]) nb.push_back(a.to);
        for (const Arc& a : g_out[v]) nb.push_back(a.to);
        std::sort(nb.begin(), nb.end()); nb.erase(std::unique(nb.begin(), nb.end()), nb.end());
        g_out[v].clear(); g_in[v].clear();
        g_out[v].shrink_to_fit(); g_in[v].shrink_to_fit();
        for (u32 x : nb) if (rank[x] == INF) pq.push({priority(x), x});
        if (nextRank % 100000 == 0) std::fprintf(stderr, "  contracted %u, shortcuts %ld (%.1fs)\n", nextRank, nShortcuts, secs());
    }
    std::fprintf(stderr, "contraction done: %ld shortcuts (%.1fs)\n", nShortcuts, secs());

    // --- split by rank, cheapest arc per (from, to)
    std::sort(all.begin(), all.end(), [](const Rec& a, const Rec& b) {
        if (a.from != b.from) return a.from < b.from;
        if (a.to != b.to) return a.to < b.to;
        return a.w < b.w;
    });
    CH ch; ch.n = N; ch.rank = rank;
    std::vector<std::vector<Arc>> up(N), inup(N);
    for (size_t i = 0; i < all.size(); ++i) {
        if (i > 0 && all[i].from == all[i - 1].from && all[i].to == all[i - 1].to) continue;   // dearer duplicate
        const Rec& r = all[i];
        if (rank[r.from] < rank[r.to]) up[r.from].push_back({r.to, r.w, r.via});
        else                           inup[r.to].push_back({r.from, r.w, r.via});
    }
    ch.outOff.assign(N + 1, 0); ch.inOff.assign(N + 1, 0);
    for (u32 v = 0; v < N; ++v) {
        ch.outOff[v + 1] = ch.outOff[v] + (u32)up[v].size();
        ch.inOff[v + 1] = ch.inOff[v] + (u32)inup[v].size();
    }
    for (u32 v = 0; v < N; ++v) {
        ch.outUp.insert(ch.outUp.end(), up[v].begin(), up[v].end());
        ch.inUp.insert(ch.inUp.end(), inup[v].begin(), inup[v].end());
    }
    std::fprintf(stderr, "up arcs: out %zu, in %zu (avg %.2f / %.2f per node)\n",
                 ch.outUp.size(), ch.inUp.size(), (double)ch.outUp.size() / N, (double)ch.inUp.size() / N);

    f = std::fopen(outPath.c_str(), "wb");
    if (!f) { std::perror("out"); return 1; }
    std::fwrite("CHG1", 1, 4, f);
    u32 hdr[3] = {N, (u32)ch.outUp.size(), (u32)ch.inUp.size()};
    std::fwrite(hdr, 4, 3, f);
    std::fwrite(ch.rank.data(), 4, N, f);
    std::fwrite(ch.outOff.data(), 4, N + 1, f);
    std::fwrite(ch.inOff.data(), 4, N + 1, f);
    std::fwrite(ch.outUp.data(), sizeof(Arc), ch.outUp.size(), f);
    std::fwrite(ch.inUp.data(), sizeof(Arc), ch.inUp.size(), f);
    std::fclose(f);

    // --- test against Dijkstra
    if (tests > 0) {
        std::mt19937 rng(7);
        // nodes that have any arc in this profile
        std::vector<u32> live;
        for (u32 v = 0; v < N; ++v) if (!origOut[v].empty()) live.push_back(v);
        Query q(ch);
        std::vector<u32> dist(N, INF);
        int ok = 0, bad = 0, unreachable = 0;
        long settledCh = 0;
        double chMs = 0;
        for (int i = 0; i < tests; ++i) {
            u32 s = live[rng() % live.size()], t = live[rng() % live.size()];
            auto a = std::chrono::steady_clock::now();
            long before = q.settledTotal;
            u32 dch = q.run(s, t);
            chMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count();
            settledCh += q.settledTotal - before;
            // plain Dijkstra on the original arcs
            std::fill(dist.begin(), dist.end(), INF);
            using P = std::pair<u32, u32>;
            std::priority_queue<P, std::vector<P>, std::greater<P>> pq2;
            dist[s] = 0; pq2.push({0, s});
            while (!pq2.empty()) {
                auto [d, u] = pq2.top(); pq2.pop();
                if (d > dist[u]) continue;
                if (u == t) break;
                for (const Arc& e : origOut[u]) {
                    u32 nd = d + e.w;
                    if (nd < dist[e.to]) { dist[e.to] = nd; pq2.push({nd, e.to}); }
                }
            }
            u32 dd = dist[t];
            if (dd == INF && dch == INF) { ++unreachable; continue; }
            if (dd == dch) ++ok; else { ++bad; if (bad < 5) std::fprintf(stderr, "MISMATCH %u->%u: dijkstra %u ch %u\n", s, t, dd, dch); }
        }
        std::fprintf(stderr, "test: %d ok, %d wrong, %d unreachable pairs; CH settles %.0f nodes / query, %.2f ms\n",
                     ok, bad, unreachable, (double)settledCh / tests, chMs / tests);
    }
    std::fprintf(stderr, "done (%.1fs)\n", secs());
    return 0;
}
