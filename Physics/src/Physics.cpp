#include <algorithm>
#include <cmath>

#include "Physics/Physics.h"

namespace Physics
{
    // ---------- world ----------
    void World::setTerrain(const Terrain& t, float k)
    {
        terrain.clear();
        for (const auto& pl : t.polylines)
        {
            for (size_t i = 1; i < pl.points.size(); ++i)
            {
                terrain.push_back({ { pl.points[i - 1].x * k, pl.points[i - 1].y * k },
                                    { pl.points[i].x * k,     pl.points[i].y * k     } });
            }
        }
    }

    // ---------- geometry / mass ----------
    Vec2 rotate(Vec2 v, float a)
    {
        float c = std::cos(a);
        float s = std::sin(a);

        return { v.x * c - v.y * s, v.x * s + v.y * c };
    }

    MassProps massProps(const RocketSpec& s, float fuel)
    {
        fuel = std::clamp(fuel, 0.0f, s.fuelCapacity);
        float frac = s.fuelCapacity > 0 ? fuel / s.fuelCapacity : 0.0f;

        // dry body, box with the avg hull width
        float avgW = s.noseHalfWidth + s.baseHalfWidth;
        float H = 2.0f * s.halfHeight;
        // dry I = m*x^2 -> dI = dm*x^2, (rho = m/L -> dm = rho * dx) -> dI = x^2*rho*dx
        // int[-L/2, L/2](x^2*rho*dx) = rho*L^3/12 = m*L^2/12
        float Id = s.dryMass / 12.0f * (avgW * avgW + H * H);

        // Fuel: box filling the tank from the bottom of the hull upward
        float fuelH = s.tankHeight * frac;
        float fuelY = s.halfHeight - fuelH * 0.5f;
        // fuel I
        float If = fuel / 12.0f * (s.tankWidth * s.tankWidth + fuelH * fuelH);

        MassProps m;
        m.mass = s.dryMass + fuel;
        float comY = (s.dryMass * s.dryComY + fuel * fuelY) / m.mass;
        m.com = { 0.0f, comY };

        // dry, fuel COMs to whole COM
        float dd = s.dryComY - comY;
        float df = fuelY - comY;
        // parallel-axis theorem, I = Icom + M*d^2
        //     I = [Idry + Mdry*d_dry^2] + [Ifuel + Mfuel * d_fuel^2]
        // alpha[rad/s^2]=tau[N*m]/I[kg*m^2] of RCS rise as fuel burns
        m.inertia = Id + s.dryMass * dd * dd + If + fuel * df * df;
        return m;
    }

    Vec2 toWorld(const RocketSpec& s, const BodyState& b, Vec2 local)
    {
        return b.pos + rotate(local - massProps(s, b.fuel).com, b.angle);
    }

    BodyState makeState(const RocketSpec& s, Vec2 center, float angle, float fuelFraction)
    {
        BodyState b;
        b.fuel = s.fuelCapacity * std::clamp(fuelFraction, 0.0f, 1.0f);
        b.angle = angle;
        b.pos = center + rotate(massProps(s, b.fuel).com, angle);   // hull center -> COM
        return b;
    }

    // ---------- nozzles ----------
    namespace
    {
        float halfWidthAt(const RocketSpec& s, float y)
        {
            float t = std::clamp((y + s.halfHeight) / (2.0f * s.halfHeight), 0.0f, 1.0f);
            return s.noseHalfWidth + (s.baseHalfWidth - s.noseHalfWidth) * t;
        }
    }

    Nozzle mainNozzle(const RocketSpec& s)
    {
        return { { 0.0f, s.halfHeight }, { 0.0f, 1.0f } };
    }

    // clockwise, nose is pushed right and the tail is pushed left (pure couple).
    // nozzles sit on the hull sides, exhaust leaving sideways
    std::array<Nozzle, 2> rcsNozzles(const RocketSpec& s, int dir)
    {
        float yn = -s.rcsArm;
        float yt = s.rcsArm;

        float wn = halfWidthAt(s, yn);
        float wt = halfWidthAt(s, yt);
        if (dir >= 0)
        {
            return { { { { -wn, yn }, { -1.0f, 0.0f } }, { {  wt, yt }, {  1.0f, 0.0f } } } };
        }
        return     { { { {  wn, yn }, {  1.0f, 0.0f } }, { { -wt, yt }, { -1.0f, 0.0f } } } };
    }

    float plumeLength(float newtons)
    {
        return newtons * kPlumeMetersPerNewton;
    }

    // ---------- collision ----------
    namespace
    {
        // contacts are found slightly before touching. solver stop in advance instead of sunk in
        constexpr float kSkin = 0.02f;         // [m]
        // allowed penetration, avoids jitter, keep resting contact stable (no contact disappear next frame, reappear, and jitter)
        constexpr float kSlop = 0.005f;        // [m]
        // below this closing speed there is no bounce (restitution is ignored), (no endless micro-bouncing at rest)
        constexpr float kRestThreshold = 1.0f; // [m/s]
        // iters for velocity solver
        constexpr int   kVelIters = 8;
        // iters for position-correction pass
        constexpr int   kPosIters = 4;

        using Poly = std::array<Vec2, 4>;

        // perp dot product ccw, edge normal (not normalized), v=perp(r)*w
        Vec2 perp(Vec2 v)
        {
            return { -v.y, v.x };
        }

        // Euclidean length
        float length(Vec2 v)
        {
            return std::sqrt(dot(v, v));
        }

        Poly worldHull(const RocketSpec& s, const BodyState& b, const MassProps& mp)
        {
            Poly out;
            auto h = hullLocal(s);
            for (int i = 0; i < 4; ++i)
                out[i] = b.pos + rotate(h[i] - mp.com, b.angle);

            return out;
        }

        struct Hit
        {
            Vec2  n;             // direction to push A out of B
            float depth = 1e30f; // penetration along n, inf->real
            int   owner = 0;     // which polygon the separating axis came from: 0 = A, 1 = B
            int   edge  = 0;     // edge index on the owner
        };

        // SAT between two convex polygons. A 2-point "polygon" is a segment.
        // B's axes are tested first so a terrain segment wins over hull edges
        // terrain nb = 2 (segment), rocket nb = 4
        bool overlap(const Vec2* A, int na, const Vec2* B, int nb, Hit& hit)
        {
            // reset, depth = 1e30, first valid axis wins
            hit = {};
            // two convex shapes are disjoint if and only if some axis exists, where their projections don’t overlap
            auto test = [&](Vec2 axis, int owner, int edge)
            {
                float len = length(axis);
                if (len < 1e-9f) // skip degenerate edge
                    return true;

                Vec2 n = axis * (1.0f / len); // norm = v / ||v||

                // project convex hull onto axis -> interval [amin, amax] along n
                float amin = 1e30f;
                float amax = -1e30f;
                for (int i = 0; i < na; ++i)
                {
                    float d = dot(A[i], n);
                    amin = std::min(amin, d);
                    amax = std::max(amax, d);
                }

                float bmin = 1e30f;
                float bmax = -1e30f;
                for (int i = 0; i < nb; ++i)
                {
                    float d = dot(B[i], n);
                    bmin = std::min(bmin, d);
                    bmax = std::max(bmax, d);
                }

                // overlap amount, how far to push the hull along +/-n, until lowest point passes the segment’s highest
                // if both are positive, the intervals overlap. negative value means a gap in that direction
                float d1 = bmax - amin; // push A along +n
                float d2 = amax - bmin; // push A along -n

                // if interval separated by gap of at least kSkin -> axis separates them (no contact)
                // smaller than kSkin still overlapping (slightly negative depth), speculative-contact skin
                if (d1 <= -kSkin || d2 <= -kSkin)
                    return false;

                // smallest across all axes (so far) -> store as min overlap
                float depth = std::min(d1, d2);
                if (depth < hit.depth)
                {
                    // normal is flipped, points in cheaper push dir for A
                    hit.depth = depth;
                    hit.n = (d1 < d2) ? n : n * -1.0f;

                    hit.owner = owner;
                    hit.edge = edge;
                }
                return true;
            };
            // how many edges to test per shape
            // polygon - n edges, segment - 1 edge (2 vert)
            // (i+1)%2 visits the same edge twice, a->b and b->a = opposite normals, identical axes,  2-vert gets 1
            int eb = (nb == 2) ? 1 : nb;
            // for each edge perp(edge) = edge normal = candidate axis
            // (i+1) % n wraps last vert back to the first, B's axes are tested first (owner 1), then A’s (owner 0)
            // separating axis exits immediately, if none separate = overlap, hit holds the axis of least penetration
            for (int i = 0; i < eb; ++i)
                if (!test(perp(B[(i + 1) % nb] - B[i]), 1, i))
                    return false;
            int ea = (na == 2) ? 1 : na;
            for (int i = 0; i < ea; ++i)
                if (!test(perp(A[(i + 1) % na] - A[i]), 0, i))
                    return false;
            // no separating axis was found -> overlap (or within the skin), hit holds the minimum-translation axis
            return true;
        }

        struct Contact
        {
            int a = 0;        // idx body A
            int b = -1;       // idx body B = -1, terrain

            Vec2 n;           // contact normal, pushing axis
            Vec2 point;       // contact point, world position
            Vec2 rA;          // offset from the A's COM to the point
            Vec2 rB;          // offset from the B's COM to the point

            // sequential impulses solver
            float jn = 0;     // accumulated normal impulses, prevent passing through
            float jt = 0;     // accumulated tangent (friction) impulses, Coulomb friction

            // K=M^-1 + R*I^-1 * R^t
            float kn = 0;     // effective inverse masses along normal
            float kt = 0;     // effective inverse masses along tangent
            float target = 0; // desired post-contact normal velocity (for bounce)
        };

        // Hit into contact points (pos out), polygon-vs-polygon, polygon-vs-segment manifold gathering
        void gather(const Vec2* A, int na, const Vec2* B, int nb, const Hit& h, std::vector<Vec2>& out)
        {
            const float eps = 0.05f; // [m] within counts as contact, rolling/jittering
            
            // reference shape, owns the winning edge, face being touched
            // incident shape, other one, vertices poke into that face
            // owner=1 -> R = B, I = A, terrain, hull
            // owner=0 -> R = A, I = B, hull, terrain
            const Vec2* R = h.owner ? B : A;  int nr = h.owner ? nb : na;
            const Vec2* I = h.owner ? A : B;  int ni = h.owner ? na : nb;

            // winning ref edge (R e0 -> e1), %nr wrap last vert to first, for a 2-vert seg
            Vec2 e0 = R[h.edge];
            Vec2 e1 = R[(h.edge + 1) % nr];
            Vec2 dir = e1 - e0;
            float len = length(dir);

            // check incident points lie along the reference edge, if real polygon (ni > 2) and the edge isn’t degenerate
            bool lateral = ni > 2 && len > 1e-9f; // segment endpoints are not range-checked, SAT already trusted the hull edge
            if (len > 1e-9f)
                dir = dir * (1.0f / len); // u vec ref edge

            // deepest incident points
            // I = A(owner=1), deepest A vert = min dot(p, n)
            // I = B(owner=0), deepest B vert = max dot(p, n), converts to min
            float sgn = h.owner ? 1.0f : -1.0f;

            // deepest penetration depth over all incident vert
            float vmin = 1e30f;
            for (int i = 0; i < ni; ++i)
                vmin = std::min(vmin, sgn * dot(I[i], h.n));

            for (int i = 0; i < ni; ++i)
            {
                Vec2 p = I[i];

                // skip too shallow along collision normal
                if (sgn * dot(p, h.n) > vmin + eps)
                    continue;

                if (lateral)
                {
                    // ghost collisions, segment on joints
                    // on segment's own normal, segment projects to single point. SAT about hull crosses inf line
                    // 10cm slack for: vertices near a joint between two segments can fall through the crack, since each neighbor accepts it
                    // if both neighboring seg accept vert, solver handles two normals independently without a bump
                    float t = dot(p - e0, dir);
                    if (t < -0.1f || t > len + 0.1f)
                        continue;
                }
                out.push_back(p);
            }
        }

        // F, thrust, rot thrusters, g, fuel burn -> mp after the burn
        void integrateForces(RocketRef& r, const World& w, float dt, MassProps& mp)
        {
            const RocketSpec& s = *r.spec;
            BodyState& b = *r.state;
            StepInfo& info = r.info;
            info = {};
            mp = massProps(s, b.fuel); // M, COM, I pre-burn

            float main = std::clamp(r.input.throttle, 0.0f, 1.0f) * s.maxThrust; // [N]
            float rot = std::clamp(r.input.rotate, -1.0f, 1.0f);
            float rcs = std::fabs(rot) * s.rcsThrust;                            // [N] per nozzle
            // m[kg/s] = Ft[N] / (Isp[s] * g0[m/s^2]), two rcs fire, match rcsNozzles
            float burn = (main + 2.0f * rcs) / (s.isp * kG0) * dt;               // [kg]

            // empty = no thrust
            if (b.fuel <= 0.0f)
            {
                main = 0;
                rcs = 0;
                burn = 0;
            }
            else if (burn > b.fuel) // account for fuel starvation
            {
                float k = b.fuel / burn;
                main *= k;
                rcs *= k;
                burn = b.fuel;
            }
            b.fuel = std::max(0.0f, b.fuel - burn);

            // accum total force, total torque about the COM
            Vec2  F{};
            float torque = 0.0f;
            // fire nozzles
            auto fire = [&](const Nozzle& nz, float newtons)
            {
                if (newtons <= 0.0f)
                    return;
                // nz.exhaust = dir gas leaves, in body frame, F on the rocket - opposite (* -1), negated
                Vec2 f = rotate(nz.exhaust * -1.0f, b.angle) * newtons;
                // lever arm from COM to the nozzle, world frame
                Vec2 arm = rotate(nz.pos - mp.com, b.angle);

                F += f;
                // tau = r x F
                torque += cross(arm, f);
            };

            fire(mainNozzle(s), main);

            int dir = rot > 0 ? 1 : -1;
            for (const Nozzle& nz : rcsNozzles(s, dir))
                fire(nz, rcs);

            // forces -> velocities (mass at the start of the step)
            // a = F / m + g, a = tau/I, semi-implicit Euler
            Vec2 acc = F * (1.0f / mp.mass) + w.gravity;
            b.vel += acc * dt;
            b.angVel += torque / mp.inertia * dt;

            info.mass = mp.mass;
            info.inertia = mp.inertia;
            info.fuelFlow = burn / dt;

            info.thrust = main;

            info.rcsForce = rcs;
            info.rcsDir = rcs > 0.0f ? dir : 0;

            mp = massProps(s, b.fuel); // upd after the burn
        }

    } // namespace

    // ---------- step ----------
    void stepAll(std::vector<RocketRef>& bodies, const World& w, float dt)
    {
        const int n = (int)bodies.size();
        std::vector<MassProps> mp(n);
        std::vector<Vec2> v0(n);

        for (int i = 0; i < n; ++i)
        {
            v0[i] = bodies[i].state->vel;
            integrateForces(bodies[i], w, dt, mp[i]); // info reset inside. impactSpeed, touching = clean
        }

        std::vector<Poly>  poly(n);
        std::vector<float> invM(n);
        std::vector<float> invI(n);
        std::vector<float> radius(n);

        // recompute hull after move, world space
        auto refresh = [&](int i)
        {
            poly[i] = worldHull(*bodies[i].spec, *bodies[i].state, mp[i]);
        };
        // caching invert M, invert I
        for (int i = 0; i < n; ++i)
        {
            refresh(i);
            invM[i] = 1.0f / mp[i].mass;
            invI[i] = 1.0f / mp[i].inertia;

            // distance from COM to farthest hull corner -> bounding circle that always contains the hull
            radius[i] = 0.0f;
            for (const Vec2& p : poly[i])
                radius[i] = std::max(radius[i], length(p - bodies[i].state->pos));
        }

        // broad phase for rocket pairs, bounding circles + skin don’t touch, SAT test is skipped
        // terrain has no such filter, so every segment is still tested for every rocket
        auto near = [&](int i, int j)
        {
            return length(bodies[i].state->pos - bodies[j].state->pos) <= radius[i] + radius[j] + kSkin;
        };

        // 1. find contacts. rocket vs terrain, rocket vs rocket
        std::vector<Contact> contacts;
        std::vector<Vec2> pts;
        // rocket vs terrain
        for (int i = 0; i < n; ++i)
        {
            for (const Segment& sg : w.terrain)
            {
                // skip degenerate segment
                if (length(sg.b - sg.a) < 1e-6f)
                    continue;

                Vec2 seg[2] = { sg.a, sg.b };
                Hit h;

                if (!overlap(poly[i].data(), 4, seg, 2, h))
                    continue;

                pts.clear();
                gather(poly[i].data(), 4, seg, 2, h, pts);
                for (Vec2 p : pts)
                {
                    Contact c;
                    c.a = i;     // idx rocket
                    c.b = -1;    // idx terrain
                    c.n = h.n;   // contact surface normal, push A away B(terrain)
                    c.point = p; // contact point with terrain
                    contacts.push_back(c);
                }
            }
        }
        // rocket vs rocket
        for (int i = 0; i < n; ++i)
        {
            for (int j = i + 1; j < n; ++j)
            {
                // broad phase for rocket pairs
                if (!near(i, j))
                    continue;

                Hit h;
                if (!overlap(poly[i].data(), 4, poly[j].data(), 4, h))
                    continue;

                pts.clear();
                gather(poly[i].data(), 4, poly[j].data(), 4, h, pts);
                for (Vec2 p : pts)
                {
                    Contact c;
                    c.a = i;     // idx rocket i
                    c.b = j;     // idx rocket j
                    c.n = h.n;   // contact surface normal, pushes A(i) away from B(j)
                    c.point = p; // 
                    contacts.push_back(c);
                }
            }
        }

        // 2. velocity constraints (sequential impulses over all contacts together)
        auto velAt = [&](int i, Vec2 r)
        {   // velocity of a point on body i at offset r from its COM, Vcom + omega x r
            return bodies[i].state->vel + perp(r) * bodies[i].state->angVel;
        };
        // velocity of A contact point relative to B. terrain, B stationary
        auto relVel = [&](const Contact& c)
        {
            Vec2 v = velAt(c.a, c.rA);
            if (c.b >= 0) // rockets
                v = v - velAt(c.b, c.rB);
            return v;
        };
        // +I goes to A and -I to B (Newton’s third law). 
        auto applyImpulse = [&](const Contact& c, Vec2 imp)
        {
            BodyState& A = *bodies[c.a].state;
            // linear J / m, angular(r x J) / I, for terrain, only A moves
            A.vel += imp * invM[c.a];
            A.angVel += cross(c.rA, imp) * invI[c.a];
            if (c.b >= 0)
            {
                BodyState& B = *bodies[c.b].state;
                B.vel += imp * (-invM[c.b]);
                B.angVel -= cross(c.rB, imp) * invI[c.b];
            }
        };

        // constants for each contact
        for (Contact& c : contacts)
        {
            c.rA = c.point - bodies[c.a].state->pos;                     // lever arms from A COM to the contact point
            c.rB = c.b >= 0 ? c.point - bodies[c.b].state->pos : Vec2{}; // lever arms from B COM to the contact point

            Vec2 t = perp(c.n); // tang, sliding dir, friction
            float imA = invM[c.a];
            float iiA = invI[c.a];

            float imB = c.b >= 0 ? invM[c.b] : 0.0f;
            float iiB = c.b >= 0 ? invI[c.b] : 0.0f;

            // lever arm impulse turn along n and t, torque per unit impulse
            float rnA = cross(c.rA, c.n); // normal torque arm
            float rtA = cross(c.rA, t);   // tang/friction torque arm

            float rnB = cross(c.rB, c.n);
            float rtB = cross(c.rB, t);

            // // constraint matrix K, effective inverse mass of the contact. Vrel along n, from unit I along n
            c.kn = imA + imB + rnA * rnA * iiA + rnB * rnB * iiB; // effective inverse mass at the contact along normal
            c.kt = imA + imB + rtA * rtA * iiA + rtB * rtB * iiB; // effective inverse mass at the contact along tangent
            // unit impulse changes contact point V by 1/m (from translation) + (r x n)^2 / I (from spin it causes)
            // solver divides desired dV by k -> needed impulse

            float vn = dot(relVel(c), c.n); // normal of relative velocity
            float closing = -vn;            // approaching

            // worst closing speed -> kCrashSpeed
            bodies[c.a].info.impactSpeed = std::max(bodies[c.a].info.impactSpeed, closing);
            if (c.b >= 0)
                bodies[c.b].info.impactSpeed = std::max(bodies[c.b].info.impactSpeed, closing);
            // velocity solver should aim for after the contact, calc once, before iters (from pre-solve V)
            // hard hit -> bounce at e * |Vn| away, 0 -> plain stop
            // Vn includes step's g -> rest Vn = -g * dt = -0.16 [m/s] (60Hz), kRestThreshold (1 m/s) prevents micro-bounces
            c.target = (vn < -kRestThreshold) ? -w.restitution * vn : 0.0f;

            // marks both bodies as touching (include speculative contacts, skin)
            bodies[c.a].info.touching = true;
            if (c.b >= 0)
                bodies[c.b].info.touching = true;
        }

        // SI velocity solver loop
        for (int it = 0; it < kVelIters; ++it)
        {
            // each time contact fixes own V and disturbs others -> itr micro-corrects, V error shrink. Gauss-Seidel relaxation
            for (Contact& c : contacts)
            {
                // re-fresh tang, earlier contacts have changed vel and angVel
                Vec2 t = perp(c.n);
                // ---normal impulse resolution (non-penetration)---
                // impulse change for current relative Vn to Vtarget, V error divided by kn
                float dj = -(dot(relVel(c), c.n) - c.target) / c.kn;
                // jn total impulse impulse accumulated across iterations
                float old = c.jn;
                // clamped at >=0 (ground can push, not pull/glue), enforces a one-way physical barrier
                c.jn = std::max(old + dj, 0.0f);
                // jn - old, let early itr overshoot, and later take some back, total never goes neg
                applyImpulse(c, c.n * (c.jn - old));

                // ---friction impulse resolution (Coulomb friction)---
                float djt = -dot(relVel(c), t) / c.kt; // impulse to cancel the sliding velocity
                // friction impulse can not exceed mu * Jn. normal solved first, frinction limit depends on
                float maxF = w.friction * c.jn;
                float oldt = c.jt;
                c.jt = std::clamp(oldt + djt, -maxF, maxF); // Coulomb friction, grib and slide
                applyImpulse(c, t * (c.jt - oldt));
            }
        }

        // 3. velocity -> position, stop penetrate further
        for (int i = 0; i < n; ++i)
        {
            BodyState& b = *bodies[i].state;
            // semi-implicit Euler, V first (F, contacts), pos second using the new V
            b.pos += b.vel * dt; // move COM by v*t
            b.angle += b.angVel * dt;
            b.angle = std::remainder(b.angle, 6.28318530718f); // 2*pi
        }

        // 4. Position correction loop, push out of the terrain, and out of each other (split by inverse mass), non-linear Gauss-Seidel
        for (int it = 0; it < kPosIters; ++it)
        {
            bool moved = false; // if any correction
            // rebuild world hull, pos changed (first pass) or corrected (later passes), correct geometry for SAT
            for (int i = 0; i < n; ++i)
                refresh(i);

            // terrain push-out
            for (int i = 0; i < n; ++i)
            {
                for (const Segment& sg : w.terrain)
                {
                    // skip degenerate segments
                    if (length(sg.b - sg.a) < 1e-6f)
                        continue;

                    Vec2 seg[2] = { sg.a, sg.b };
                    Hit h;
                    // speculative contacts, no push before kSlop, no jitter
                    if (overlap(poly[i].data(), 4, seg, 2, h) && h.depth > kSlop)
                    {
                        // if penetration more then kSlop shift the body out along the Hit’s normal
                        bodies[i].state->pos += h.n * (h.depth - kSlop);
                        // rebuild hull so rocket see new pos
                        refresh(i);
                        moved = true;
                    }
                }
            }
            // rocket vs rocket push-out
            for (int i = 0; i < n; ++i)
            {
                for (int j = i + 1; j < n; ++j)
                {
                    // each pair once, bounding-circle broad phase to skip distant pairs
                    if (!near(i, j))
                        continue;

                    Hit h;
                    if (overlap(poly[i].data(), 4, poly[j].data(), 4, h) && h.depth > kSlop)
                    {
                        float corr = h.depth - kSlop; // distance to resolve
                        float wsum = invM[i] + invM[j]; // sum of inverse masses
                        // i moves along +n, j along -n
                        bodies[i].state->pos += h.n * (corr * invM[i] / wsum);
                        bodies[j].state->pos += h.n * (-corr * invM[j] / wsum);

                        refresh(i); refresh(j);
                        moved = true;
                    }
                }
            }
            // no overlap
            if (!moved)
                break;
        }

        for (int i = 0; i < n; ++i)
        {
            // kinematic acceleration
            bodies[i].info.acc = (bodies[i].state->vel - v0[i]) * (1.0f / dt);
        }
    }

    StepInfo step(const RocketSpec& s, BodyState& b, const Input& in, const World& w, float dt)
    {
        std::vector<RocketRef> one{ RocketRef{ &s, &b, in, {} } };
        stepAll(one, w, dt);
        return one[0].info;
    }

} // namespace Physics