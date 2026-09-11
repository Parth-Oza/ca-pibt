// Lifelong MAPF warehouse fleet simulator
// Planners: PIBT (baseline), PIBT + static highways (H-PIBT), windowed prioritized planning (WPP, RHCR-lite),
//           CA-PIBT: PIBT + online congestion-adaptive heuristics + stall recovery (proposed)
// Execution layer: resolves vertex/edge conflicts, injects random execution delays.
// Build: g++ -O2 -std=c++17 -o sim sim.cpp
#include <bits/stdc++.h>
using namespace std;

struct Grid {
    int H = 0, W = 0;
    vector<char> ch;         // raw chars
    vector<int> freeIdx;     // list of free cells
    vector<int> emitters, services;
    inline bool isFree(int r, int c) const {
        if (r < 0 || c < 0 || r >= H || c >= W) return false;
        char x = ch[r * W + c];
        return x != '@' && x != 'T';
    }
    inline int id(int r, int c) const { return r * W + c; }
    // neighbors in 4 dirs: 0=E,1=S,2=W,3=N
    static const int DR[4], DC[4];
    inline int nb(int v, int d) const {
        int r = v / W + DR[d], c = v % W + DC[d];
        return isFree(r, c) ? id(r, c) : -1;
    }
    void load(const string& path) {
        ifstream f(path);
        if (!f) { cerr << "cannot open map " << path << endl; exit(1); }
        string line;
        vector<string> rows;
        // MovingAI format or RHCR kiva format
        getline(f, line);
        if (line.rfind("type", 0) == 0) {
            string tok; int h, w;
            f >> tok >> h >> tok >> w >> tok; // height h width w map
            H = h; W = w;
            getline(f, line);
            for (int i = 0; i < H; i++) { getline(f, line); while ((int)line.size() < W) line.push_back('@'); rows.push_back(line.substr(0, W)); }
        } else {
            // RHCR: "H,W" then 3 numeric lines
            sscanf(line.c_str(), "%d,%d", &H, &W);
            for (int i = 0; i < 3; i++) getline(f, line);
            for (int i = 0; i < H; i++) { getline(f, line); while ((int)line.size() < W) line.push_back('@'); rows.push_back(line.substr(0, W)); }
        }
        ch.resize(H * W);
        for (int r = 0; r < H; r++) for (int c = 0; c < W; c++) {
            char x = rows[r][c];
            ch[r * W + c] = x;
            if (x != '@' && x != 'T') {
                freeIdx.push_back(r * W + c);
                if (x == 'E' || x == 'r') emitters.push_back(r * W + c);
                if (x == 'S' || x == 'e') services.push_back(r * W + c);
            }
        }
    }
};
const int Grid::DR[4] = {0, 1, 0, -1};
const int Grid::DC[4] = {1, 0, -1, 0};

struct Agent {
    int pos, goal, prevPos;
    int stage = 0;            // 0 -> heading to service, 1 -> heading to emitter (alternating)
    long long prio = 0;       // PIBT priority (elapsed since last goal)
    int noProgress = 0;       // steps without decreasing distance-to-goal
    int bestDist = INT_MAX;
    int tasksDone = 0;
    int waits = 0;
    vector<uint16_t> dist;    // heuristic table (maybe congestion-weighted, fixed point x4)
    int tableAge = 0;
    int recoverGoal = -1;     // temporary retreat goal
    int recoverTTL = 0;
    vector<int> plan;         // for windowed planner: plan[k] = cell at t+k
};

struct Params {
    string map, planner = "pibt";
    int agents = 100, steps = 1000, seed = 0;
    double delayProb = 0.0;
    // CA-PIBT
    double alpha = 10.0;    // head-on flow weight (rate-normalized)
    double gamma = 5.0;     // wait hotspot weight
    double decay = 0.97;    // per-step decay of flow counters
    int refreshEvery = 20;  // each agent refreshes its table at most every k steps (staggered)
    int stallW = 30;        // steps w/o progress before recovery
    // highways
    double hwPenalty = 1.0;
    // WPP
    int window = 10, horizon = 20;
    bool caHighway = true;
    bool recover = true;
    string out = "", label = "";
};

struct Sim {
    Grid g; Params P; mt19937_64 rng;
    vector<Agent> A;
    vector<int> occ;                 // cell -> agent or -1
    vector<array<float,4>> flow;     // decayed directed flow per cell/dir
    vector<float> waitHeat;          // decayed wait counts per cell
    long long totalTasks = 0, totalWaits = 0, stallEvents = 0, recoveries = 0;
    double planTime = 0; long long conflictsResolved = 0, delaysInjected = 0;
    vector<int> throughputPerStep;
    int t = 0;

    int randFree() { return g.freeIdx[rng() % g.freeIdx.size()]; }
    int sampleGoal(Agent& a) {
        if (!g.emitters.empty() && !g.services.empty()) {
            int gcell = a.stage == 0 ? g.services[rng() % g.services.size()] : g.emitters[rng() % g.emitters.size()];
            a.stage ^= 1; return gcell;
        }
        return randFree();
    }

    // plain BFS distance table (uint16, x4 fixed point to match weighted tables)
    void bfs(int goal, vector<uint16_t>& d) {
        d.assign(g.H * g.W, UINT16_MAX);
        deque<int> q; d[goal] = 0; q.push_back(goal);
        while (!q.empty()) { int v = q.front(); q.pop_front();
            for (int k = 0; k < 4; k++) { int u = g.nb(v, k); if (u >= 0 && d[u] == UINT16_MAX) { d[u] = d[v] + 4; q.push_back(u); } } }
    }
    // edge cost for moving u -> v in direction k (fixed point x4)
    inline int edgeCostHW(int u, int v, int k) {
        // static highways: even rows prefer East (k=0), odd rows prefer West (k=2); even cols prefer South, odd prefer North
        int r = u / g.W, c = u % g.W;
        bool against = false;
        if (k == 0 || k == 2) against = ((r % 2 == 0) ? (k == 2) : (k == 0));
        else against = ((c % 2 == 0) ? (k == 3) : (k == 1));
        return (int)lround((1.0 + (against ? P.hwPenalty : 0.0)) * 4);
    }
    inline int edgeCostCA(int u, int v, int k) {
        double base = P.caHighway ? edgeCostHW(u, v, k) / 4.0 : 1.0;
        double norm = 1.0 - P.decay; // counters -> rates in [0,1]
        double c = base + P.alpha * flow[v][(k + 2) % 4] * norm + P.gamma * waitHeat[v] * norm;
        return (int)lround(c * 4);
    }
    // weighted Dijkstra from goal over reversed edges (cost of edge u->v charged when going u->v)
    void dijkstra(int goal, vector<uint16_t>& d, bool ca) {
        int N = g.H * g.W; d.assign(N, UINT16_MAX);
        priority_queue<pair<int,int>, vector<pair<int,int>>, greater<>> pq;
        d[goal] = 0; pq.push({0, goal});
        while (!pq.empty()) { auto [dv, v] = pq.top(); pq.pop(); if (dv > d[v]) continue;
            for (int k = 0; k < 4; k++) { int u = g.nb(v, k); if (u < 0) continue; // u is predecessor: u -> v is direction (k+2)%4
                int kk = (k + 2) % 4; int w = ca ? edgeCostCA(u, v, kk) : edgeCostHW(u, v, kk);
                int nd = dv + w; if (nd < 60000 && nd < d[u]) { d[u] = nd; pq.push({nd, u}); } } }
    }
    void buildTable(Agent& a, int goal) {
        if (P.planner == "capibt") dijkstra(goal, a.dist, true);
        else if (P.planner == "hpibt") dijkstra(goal, a.dist, false);
        else bfs(goal, a.dist);
        a.tableAge = 0;
    }
    inline int hval(const Agent& a, int v) const { return a.dist[v]; }

    void assignTask(int i) {
        Agent& a = A[i]; a.goal = sampleGoal(a); buildTable(a, a.goal); a.prio = 0; a.noProgress = 0; a.bestDist = INT_MAX; a.recoverGoal = -1; a.recoverTTL = 0; if (P.planner != "wpp") a.plan.clear();
    }

    void init() {
        rng.seed(P.seed); g.load(P.map);
        int N = g.H * g.W; occ.assign(N, -1); flow.assign(N, {0,0,0,0}); waitHeat.assign(N, 0);
        vector<int> starts = g.freeIdx; shuffle(starts.begin(), starts.end(), rng);
        if (P.agents > (int)starts.size() / 2) { cerr << "too many agents" << endl; exit(1); }
        A.resize(P.agents);
        for (int i = 0; i < P.agents; i++) { A[i].pos = starts[i]; A[i].prevPos = starts[i]; occ[starts[i]] = i; A[i].stage = i % 2; assignTask(i); }
    }

    // ---------------- PIBT ----------------
    vector<int> nextPos; vector<char> decided; vector<int> order;
    bool pibtFunc(int i, int j /* parent or -1 */) {
        Agent& a = A[i];
        int target = (a.recoverGoal >= 0) ? a.recoverGoal : a.goal;
        vector<pair<int,int>> cands; // (score, cell)
        for (int k = 0; k < 4; k++) { int u = g.nb(a.pos, k); if (u >= 0) cands.push_back({hval(a, u), u}); }
        cands.push_back({hval(a, a.pos), a.pos});
        if (a.recoverGoal >= 0) { // recovery: prefer low-heat cells away from current goal path
            for (auto& c : cands) { int cell = c.second; c.first = (int)(waitHeat[cell] * 40) + (cell == a.pos ? 2 : 0) + (rng() % 3); }
        }
        // tie-break randomly
        shuffle(cands.begin(), cands.end(), rng);
        stable_sort(cands.begin(), cands.end(), [](auto& x, auto& y) { return x.first < y.first; });
        for (auto& c : cands) {
            int u = c.second;
            if (occ[u] >= 0 && occ[u] != i && nextPos[occ[u]] == a.pos) continue; // no swap
            bool takenNext = false;
            for (int k = 0; k < 4; k++) { int w = g.nb(u, k); if (w >= 0 && occ[w] >= 0 && decided[occ[w]] && nextPos[occ[w]] == u) { takenNext = true; break; } }
            if (!takenNext && occ[u] >= 0 && decided[occ[u]] && nextPos[occ[u]] == u) takenNext = true; // staying agent
            if (takenNext) continue;
            if (j >= 0 && u == A[j].pos) continue;
            nextPos[i] = u; decided[i] = 1;
            int k2 = occ[u];
            if (k2 >= 0 && k2 != i && !decided[k2]) {
                if (!pibtFunc(k2, i)) { decided[i] = 0; continue; }
            }
            return true;
        }
        nextPos[i] = a.pos; decided[i] = 1; return false;
    }
    void stepPIBT() {
        int n = A.size(); nextPos.assign(n, -1); decided.assign(n, 0); order.resize(n);
        iota(order.begin(), order.end(), 0);
        // priority: elapsed time since task start (+ stall boost), tie-break by id
        vector<long long> pr(n);
        for (int i = 0; i < n; i++) { pr[i] = A[i].prio * 1000 + (A[i].noProgress >= P.stallW ? 5000000LL : 0) + (rng() % 1000); }
        sort(order.begin(), order.end(), [&](int x, int y) { return pr[x] > pr[y]; });
        for (int i : order) if (!decided[i]) pibtFunc(i, -1);
    }

    // ---------------- Windowed prioritized planning (RHCR-lite: PP + space-time A*) ----------------
    struct RT { unordered_set<long long> v; unordered_set<long long> e; };
    static inline long long key2(int a, int b) { return ((long long)a << 32) | (unsigned)b; }
    static inline long long key3(int a, int b, int c) { return (((long long)a * 2000003LL + b) * 1000003LL) + c; }
    bool stAstar(Agent& a, RT& rt, vector<int>& path) {
        // time-expanded A* over horizon P.horizon; path[k] = cell at time t+k (path[0]=pos)
        int Hz = P.horizon; int start = a.pos;
        struct Node { int f, gcost, v, tt, parent; };
        vector<Node> nodes; nodes.reserve(20000);
        priority_queue<pair<int,int>, vector<pair<int,int>>, greater<>> pq;
        unordered_map<long long,int> best;
        auto h = [&](int v) { int d = a.dist[v]; return d == UINT16_MAX ? 100000 : d / 4; };
        nodes.push_back({h(start), 0, start, 0, -1}); pq.push({nodes[0].f, 0}); best[key2(start,0)] = 0;
        int expansions = 0;
        while (!pq.empty()) {
            auto [f, id] = pq.top(); pq.pop(); Node nd = nodes[id];
            if (nd.tt == Hz || (++expansions > 20000)) {
                if (nd.tt < Hz) return false;
                path.clear(); int cur = id; while (cur >= 0) { path.push_back(nodes[cur].v); cur = nodes[cur].parent; } reverse(path.begin(), path.end()); return true;
            }
            for (int k = 0; k < 5; k++) { int u = k < 4 ? g.nb(nd.v, k) : nd.v; if (u < 0) continue;
                int nt = nd.tt + 1;
                if (rt.v.count(key2(u, nt))) continue;
                if (u != nd.v && rt.e.count(key3(u, nd.v, nt))) continue; // someone moves u->v at same time (swap)
                long long kk = key2(u, nt); int gc = nd.gcost + 1;
                // goal-aware: waiting at goal costs 0 extra so agents park after arrival within window
                if (u == a.goal && nd.v == a.goal) gc = nd.gcost;
                auto it = best.find(kk); if (it != best.end() && nodes[it->second].gcost <= gc) continue;
                nodes.push_back({gc + h(u), gc, u, nt, id}); best[kk] = nodes.size() - 1; pq.push({nodes.back().f, (int)nodes.size() - 1});
            }
        }
        return false;
    }
    void replanWPP() {
        int n = A.size(); vector<int> ord(n); iota(ord.begin(), ord.end(), 0);
        vector<long long> pr(n); for (int i = 0; i < n; i++) pr[i] = A[i].prio * 1000 + (rng() % 1000);
        sort(ord.begin(), ord.end(), [&](int x, int y) { return pr[x] > pr[y]; });
        vector<char> stuck(n, 0);
        for (int round = 0; round < 4; round++) {
            RT rt; bool newStuck = false; bool last = (round == 3);
            for (int i = 0; i < n; i++) if (stuck[i]) { A[i].plan.assign(P.horizon + 1, A[i].pos); for (int k = 1; k <= P.horizon; k++) rt.v.insert(key2(A[i].pos, k)); }
            for (int i : ord) { if (stuck[i]) continue; Agent& a = A[i]; vector<int> path;
                bool ok = stAstar(a, rt, path);
                if (!ok) { stuck[i] = 1; newStuck = true; wppFailures++; if (!last) break; path.assign(P.horizon + 1, a.pos); for (int k = 1; k <= P.horizon; k++) rt.v.insert(key2(a.pos, k)); a.plan = path; continue; }
                a.plan = path;
                for (int k = 1; k < (int)path.size(); k++) { rt.v.insert(key2(path[k], k)); rt.e.insert(key3(path[k-1], path[k], k)); } }
            if (!newStuck || last) return;
        }
    }
    long long wppFailures = 0;
    int wppCounter = 0;
    void stepWPP() {
        int n = A.size();
        bool need = (wppCounter % P.window == 0);
        if (!need) for (int i = 0; i < n; i++) if (A[i].plan.size() < 2 || A[i].plan[0] != A[i].pos) { need = true; break; }
        if (need) { replanWPP(); wppCounter = 0; }
        nextPos.assign(n, -1);
        for (int i = 0; i < n; i++) { Agent& a = A[i]; if (a.plan.size() >= 2 && a.plan[0] == a.pos) { nextPos[i] = a.plan[1]; } else { nextPos[i] = a.pos; a.plan.clear(); } }
        wppCounter++;
    }

    // ---------------- execution layer ----------------
    void execute() {
        int n = A.size();
        vector<int> want = nextPos;
        // random delays: agent fails to move (stays)
        for (int i = 0; i < n; i++) if (want[i] != A[i].pos && P.delayProb > 0 && (double)(rng() % 1000000) / 1e6 < P.delayProb) { want[i] = A[i].pos; delaysInjected++; }
        // resolve conflicts: iterate until stable. An agent may enter cell c only if c is empty after moves, i.e.
        // occupant of c is moving away (and not swapping with us). Vertex conflicts among movers: lower index wins... (rare with PIBT)
        vector<int> claim(g.H * g.W, -1);
        bool changed = true; int iter = 0;
        while (changed && iter++ < n + 5) {
            changed = false; fill(claim.begin(), claim.end(), -1);
            for (int i = 0; i < n; i++) { int c = want[i]; if (claim[c] == -1) claim[c] = i; else { // vertex conflict
                    int j = claim[c]; // the one who is staying keeps it; else lower prio yields
                    if (want[j] == A[j].pos) { want[i] = A[i].pos; } else if (want[i] == A[i].pos) { want[j] = A[j].pos; claim[c] = i; } else { if (A[i].prio > A[j].prio) { want[j] = A[j].pos; claim[c] = i; } else want[i] = A[i].pos; }
                    changed = true; conflictsResolved++; } }
            for (int i = 0; i < n; i++) { if (want[i] == A[i].pos) continue; int o = occ[want[i]];
                if (o >= 0 && o != i) { if (want[o] == A[o].pos) { want[i] = A[i].pos; changed = true; conflictsResolved++; }
                    else if (want[o] == A[i].pos) { want[i] = A[i].pos; want[o] = A[o].pos; changed = true; conflictsResolved++; } } }
        }
        // apply
        for (int i = 0; i < n; i++) occ[A[i].pos] = -1;
        int done = 0;
        for (int i = 0; i < n; i++) { Agent& a = A[i]; a.prevPos = a.pos;
            if (want[i] == a.pos) { a.waits++; totalWaits++; waitHeat[a.pos] += 1.0f; }
            else { int k = -1; for (int d = 0; d < 4; d++) if (g.nb(a.pos, d) == want[i]) k = d; if (k >= 0) flow[a.pos][k] += 1.0f; }
            a.pos = want[i]; occ[a.pos] = i; a.prio++;
            if (!a.plan.empty()) { if (a.plan.size() >= 2 && a.plan[1] == a.pos) a.plan.erase(a.plan.begin()); else a.plan.clear(); }
            // progress tracking
            int d = a.dist[a.pos]; if (d < a.bestDist) { a.bestDist = d; a.noProgress = 0; } else a.noProgress++;
            if (a.recoverTTL > 0 && --a.recoverTTL == 0) { a.recoverGoal = -1; a.noProgress = 0; }
            if (a.pos == a.goal) { a.tasksDone++; totalTasks++; done++; assignTask(i); }
        }
        throughputPerStep.push_back(done);
        // decay
        for (auto& f : flow) for (auto& x : f) x *= (float)P.decay;
        for (auto& x : waitHeat) x *= (float)P.decay;
    }

    void maintenanceCA() {
        int n = A.size();
        for (int i = 0; i < n; i++) { Agent& a = A[i]; a.tableAge++;
            // staggered refresh of congestion-weighted tables
            if (P.planner == "capibt" && a.tableAge >= P.refreshEvery && (int)((t + i) % P.refreshEvery) == 0) { buildTable(a, a.goal); }
            // stall recovery (proposed): retreat to a nearby low-heat cell for a short while
            if (P.planner == "capibt" && P.recover && a.noProgress >= P.stallW && a.recoverGoal < 0) { stallEvents++; recoveries++;
                // choose nearby cell within radius via BFS with lowest heat
                int best = -1; float bh = 1e9; deque<pair<int,int>> q; unordered_set<int> seen; q.push_back({a.pos, 0}); seen.insert(a.pos);
                while (!q.empty()) { auto [v, dd] = q.front(); q.pop_front(); if (dd > 3) continue; if (occ[v] < 0 && waitHeat[v] < bh) { bh = waitHeat[v]; best = v; }
                    for (int k = 0; k < 4; k++) { int u = g.nb(v, k); if (u >= 0 && !seen.count(u)) { seen.insert(u); q.push_back({u, dd + 1}); } } }
                if (best >= 0) { a.recoverGoal = best; a.recoverTTL = 6; }
                a.noProgress = 0; buildTable(a, a.goal); }
            else if ((P.planner != "capibt" || !P.recover) && a.noProgress == P.stallW) stallEvents++;
        }
    }

    void run() {
        init();
        for (t = 0; t < P.steps; t++) {
            auto t0 = chrono::steady_clock::now();
            if (P.planner == "wpp") stepWPP(); else stepPIBT();
            planTime += chrono::duration<double>(chrono::steady_clock::now() - t0).count();
            execute();
            auto t1 = chrono::steady_clock::now();
            maintenanceCA();
            planTime += chrono::duration<double>(chrono::steady_clock::now() - t1).count();
        }
        // sanity: no two agents share a cell
        { set<int> s; for (auto& a : A) s.insert(a.pos); if ((int)s.size() != (int)A.size()) { cerr << "COLLISION DETECTED" << endl; exit(2); } }
        int n = A.size();
        long long maxNoProg = 0; for (auto& a : A) maxNoProg = max<long long>(maxNoProg, a.noProgress);
        double thr = (double)totalTasks / P.steps;
        double waitsPer = (double)totalWaits / n / P.steps;
        // per-agent task fairness (std of tasksDone)
        double mu = (double)totalTasks / n, var = 0; for (auto& a : A) var += (a.tasksDone - mu) * (a.tasksDone - mu); var /= n;
        printf("map=%s planner=%s agents=%d steps=%d seed=%d delay=%.2f throughput=%.4f waits_frac=%.4f stall_events=%lld recoveries=%lld conflicts=%lld delays=%lld ms_per_step=%.3f task_std=%.3f\n",
               P.map.c_str(), P.planner.c_str(), n, P.steps, P.seed, P.delayProb, thr, waitsPer, stallEvents, recoveries, conflictsResolved, delaysInjected, 1000.0 * planTime / P.steps, sqrt(var));
        if (!P.out.empty()) { ofstream o(P.out, ios::app);
            o << P.map << "," << (P.label.empty() ? P.planner : P.label) << "," << n << "," << P.steps << "," << P.seed << "," << P.delayProb << "," << thr << "," << waitsPer << "," << stallEvents << "," << recoveries << "," << conflictsResolved << "," << delaysInjected << "," << 1000.0 * planTime / P.steps << "," << sqrt(var) << "," << P.alpha << "," << P.gamma << "," << P.stallW << "," << P.window << "\n"; }
    }
};

int main(int argc, char** argv) {
    Params P;
    for (int i = 1; i < argc; i++) { string a = argv[i]; auto val = [&]() { return string(argv[++i]); };
        if (a == "--map") P.map = val(); else if (a == "--planner") P.planner = val(); else if (a == "--agents") P.agents = stoi(val());
        else if (a == "--steps") P.steps = stoi(val()); else if (a == "--seed") P.seed = stoi(val()); else if (a == "--delay") P.delayProb = stod(val());
        else if (a == "--alpha") P.alpha = stod(val()); else if (a == "--gamma") P.gamma = stod(val()); else if (a == "--decay") P.decay = stod(val());
        else if (a == "--refresh") P.refreshEvery = stoi(val()); else if (a == "--stallw") P.stallW = stoi(val()); else if (a == "--hw") P.hwPenalty = stod(val());
        else if (a == "--window") P.window = stoi(val()); else if (a == "--horizon") P.horizon = stoi(val()); else if (a == "--out") P.out = val(); else if (a == "--label") P.label = val(); else if (a == "--cahw") P.caHighway = (stoi(val()) != 0); else if (a == "--recover") P.recover = (stoi(val()) != 0);
        else { cerr << "unknown arg " << a << endl; return 1; } }
    Sim s; s.P = P; s.run();
    return 0;
}
