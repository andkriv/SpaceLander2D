#include <algorithm>
#include <cmath>

#include "Physics/Physics.h"

namespace Physics
{
    // ---------- World ----------
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

    // ---------- Geometry / mass ----------
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
            Vec2  n;                  // direction to push the hull out of the segment
            float depth = 1e30f;      // penetration along n, inf -> real
            bool  segAxis = false;    // if n is the segment's normal = true
        };

        // SAT between the convex hull and a segment (degenerate polygon)
        bool overlap(const Poly& P, const Segment& sg, Hit& hit)
        {
            hit = {};
            // two convex shapes are disjoint if and only if some axis exists, where their projections don’t overlap
            auto test = [&](Vec2 axis, bool segAxis)
            {
                float len = length(axis);
                if (len < 1e-9f) // skip degenerate edge
                    return true;
                Vec2 n = axis * (1.0f / len); // norm = v / ||v||

                // project convex hull onto axis -> interval [pmin, pmax] along n
                float pmin = 1e30f;
                float pmax = -1e30f;
                for (const Vec2& p : P)
                {
                    float d = dot(p, n);
                    pmin = std::min(pmin, d);
                    pmax = std::max(pmax, d);
                }

                // project segment’s two endpoints -> interval [smin, smax]
                float sa = dot(sg.a, n);
                float sb = dot(sg.b, n);
                float smin = std::min(sa, sb);
                float smax = std::max(sa, sb);

                // overlap amount, how far to push the hull along +/-n, until lowest point passes the segment’s highest
                float d1 = smax - pmin;   // push along +n
                float d2 = pmax - smin;   // push along -n

                // if interval separated by gap of at least kSkin -> axis separates them (no contact)
                // smaller than kSkin still overlapping (slightly negative depth), speculative-contact skin
                if (d1 <= -kSkin || d2 <= -kSkin)
                    return false;

                // smallest across all axes (so far) -> store as min overlap
                float depth = std::min(d1, d2);
                if (depth < hit.depth)
                {
                    hit.depth = depth;
                    hit.n = (d1 < d2) ? n : n * -1.0f;
                    hit.segAxis = segAxis;
                }
                return true;
            };

            // test segment’s normal, (seg has no area, only normal)
            if (!test(perp(sg.b - sg.a), true))
                return false;

            // test hull’s 4 edges
            for (int i = 0; i < 4; ++i)
                if (!test(perp(P[(i + 1) % 4] - P[i]), false))
                    return false;
            // no separating axis was found -> overlap (or within the skin), hit holds the minimum-translation axis
            return true;
        }

        struct Contact
        {
            Vec2 n;           // contact normal, pushing axis
            Vec2 point;       // contact point, world position
            Vec2 r;           // offset from the COM to the point

            // sequential impulses solver
            float jn = 0;     // accumulated normal impulses, prevent passing through
            float jt = 0;     // accumulated tangent (friction) impulses, Coulomb friction

            // K=M^-1 + R*I^-1 * R^t
            float kn = 0;     // effective inverse masses along normal
            float kt = 0;     // effective inverse masses along tangent
            float target = 0; // desired post-contact normal velocity (for bounce)
        };

        // Hit into contact points (out)
        void gather(const Poly& P, const Segment& sg, const Hit& h, std::vector<Contact>& out)
        {
            const float eps = 0.05f; // [m] within counts as contact, rolling/jittering
            if (h.segAxis)
            {
                // hull vertices on the segment
                // hull's deepest vertices, kept only if they lie along the segment
                Vec2 dir = sg.b - sg.a;
                float len = length(dir);
                dir = dir * (1.0f / len); // segment’s u vec

                // deepest into terrain
                float pmin = 1e30f;
                for (const Vec2& p : P)
                    pmin = std::min(pmin, dot(p, h.n));

                // contact manifold
                for (const Vec2& p : P)
                {
                    float t = dot(p - sg.a, dir);
                    // ghost collisions, segment on joints
                    // on segment's own normal, segment projects to single point. SAT about hull crosses inf line
                    // prevents vertex further the end of segment as contact
                    // 10cm slack for: vertices near a joint between two segments can fall through the crack, since each neighbor accepts it
                    if (dot(p, h.n) <= pmin + eps && t >= -0.1f && t <= len + 0.1f)
                        out.push_back({ h.n, p });
                    // near a joint, both segments may accept the same vertex -> two contacts with different normals
                    // acceptable, solver treats them as independent constraints, no ghost collision bump
                }
            }
            else
            {
                // A segment endpoint poking into the hull
                float sa = dot(sg.a, h.n);
                float sb = dot(sg.b, h.n);
                float smax = std::max(sa, sb);
                // hull must travel along +n to get clear, highest value endpoint, it has to clear (endpoint deepest inside the hull)

                if (sa >= smax - eps)
                    out.push_back({ h.n, sg.a });

                if (sb >= smax - eps)
                    out.push_back({ h.n, sg.b });
                // if the segment is parallel to the hull edge, both qualify
            }
        }

    } // namespace

    // ---------- step ----------
    StepInfo step(const RocketSpec& s, BodyState& b, const Input& in, const World& w, float dt)
    {
        StepInfo info;
        const Vec2 v0 = b.vel;
        MassProps mp = massProps(s, b.fuel);

        // 1. thrust and fuel
        float thrust = std::clamp(in.throttle, 0.0f, 1.0f) * s.maxThrust;
        // m[kg/s] = Ft[N] / (Isp[s] * g0[m/s^2])
        float burn = thrust / (s.isp * kG0) * dt;
        if (b.fuel <= 0.0f)
        {
            thrust = 0.0f;
            burn = 0.0f;
        }
        else if (burn > b.fuel) // account for fuel starvation
        {
            thrust *= b.fuel / burn;
            burn = b.fuel;
        }
        b.fuel = std::max(0.0f, b.fuel - burn);

        // 2. torque
        // u vector nose direction
        Vec2 nose{ std::sin(b.angle), -std::cos(b.angle) };
        Vec2 F = nose * thrust;

        Vec2 enginePos{ 0.0f, s.halfHeight };
        Vec2 engineArm = rotate(enginePos - mp.com, b.angle);
        // tau = r x F, and include RCS torque
        float torque = cross(engineArm, F) + std::clamp(in.rotate, -1.0f, 1.0f) * s.maxTorque;

        // 3. forces -> velocities (mass at the start of the step)
        Vec2 acc = F * (1.0f / mp.mass) + w.gravity;
        b.vel += acc * dt;
        b.angVel += torque / mp.inertia * dt;

        info.mass = mp.mass;
        info.inertia = mp.inertia;
        info.thrust = thrust;
        info.fuelFlow = burn / dt;

        mp = massProps(s, b.fuel); // upd after the burn

        // 4. velocity constraints (sequential impulses)
        // generate contact points if overlaps (or is within the skin)
        std::vector<Contact> contacts;
        // constrain velocity so the upcoming move can’t go through the ground
        Poly P = worldHull(s, b, mp);
        for (const Segment& sg : w.terrain)
        {
            if (length(sg.b - sg.a) < 1e-6f)
                continue;

            Hit h;
            if (overlap(P, sg, h))
                gather(P, sg, h, contacts);
        }

        const float invM = 1.0f / mp.mass;
        const float invI = 1.0f / mp.inertia;
        for (Contact& c : contacts)
        {
            // lever arm from the COM to the contact point
            c.r = c.point - b.pos;
            // tang, sliding direction
            Vec2 t = perp(c.n);
            float rn = cross(c.r, c.n); // [m] normal torque arm
            float rt = cross(c.r, t);   // [m] tang/friction torque arm
            // constraint matrix K
            c.kn = invM + rn * rn * invI; // effective inverse mass at the contact along normal
            c.kt = invM + rt * rt * invI; // effective inverse mass at the contact along tangent
            // unit impulse changes contact point V by 1/m (from translation) + (r x n)^2 / I (from spin it causes)
            // solver divides desired dV by k -> needed impulse

            Vec2 vp = b.vel + perp(c.r) * b.angVel; // [m/s] contact point velocity
            float vn = dot(vp, c.n);

            info.impactSpeed = std::max(info.impactSpeed, -vn);
            // velocity solver should aim for after the contact, calc once, before iters (from pre-solve V)
            // hard hit -> bounce at e * |Vn| away, 0 -> plain stop
            // Vn includes step's g -> rest Vn = -g * dt = -0.16 [m/s] (60Hz), kRestThreshold (1 m/s) prevents micro-bounces
            c.target = (vn < -kRestThreshold) ? -w.restitution * vn : 0.0f;
        }

        auto apply = [&](const Contact& c, Vec2 impulse)
        {
            // linear and angular V, changes by impulse and offset r
            b.vel += impulse * invM;
            b.angVel += cross(c.r, impulse) * invI;
        };

        // SI velocity solver loop
        for (int it = 0; it < kVelIters; ++it)
        {
            // each time contact fixes own V and disturbs others -> itr micro-corrects, V error shrink. Gauss-Seidel relaxation
            for (Contact& c : contacts)
            {
                // re-fresh tang and Vcontact, earlier contacts have changed vel and angVel
                Vec2 t = perp(c.n);
                Vec2 vp = b.vel + perp(c.r) * b.angVel;

                // ---normal impulse resolution (non-penetration)---
                // impulse change for current relative Vn to Vtarget, V error divided by kn
                float dj = -(dot(vp, c.n) - c.target) / c.kn;

                // jn total impulse impulse accumulated across iterations
                float old = c.jn;
                // clamped at >=0 (ground can push, not pull/glue), enforces a one-way physical barrier
                c.jn = std::max(old + dj, 0.0f);
                // jn - old, let early itr overshoot, and later take some back, total never goes neg
                apply(c, c.n * (c.jn - old));

                // ---friction impulse resolution (Coulomb friction)---
                // re-fresh, normal impulse changed it
                vp = b.vel + perp(c.r) * b.angVel;
                float djt = -dot(vp, t) / c.kt; // impulse to cancel the sliding velocity
                // friction impulse can not exceed mu * Jn. normal solved first, frinction limit depends on
                float maxF = w.friction * c.jn;
                float oldt = c.jt;
                c.jt = std::clamp(oldt + djt, -maxF, maxF); // Coulomb friction, grib and slide
                apply(c, t * (c.jt - oldt));
            }
        }
        info.touching = !contacts.empty();

        // 5. velocity -> position, stop penetrate further
        b.pos += b.vel * dt;
        b.angle += b.angVel * dt;
        b.angle = std::remainder(b.angle, 6.28318530718f); // 2*pi

        // 6. position correction loop, push the hull out of the terrain. non-linear Gauss-Seidel
        for (int it = 0; it < kPosIters; ++it)
        {
            bool moved = false;
            P = worldHull(s, b, mp);
            for (const Segment& sg : w.terrain)
            {
                if (length(sg.b - sg.a) < 1e-6f)
                    continue;

                Hit h;
                // speculative contacts, no push before kSlop, no jitter 
                if (overlap(P, sg, h) && h.depth > kSlop)
                {
                    // if penetration more then kSlop shift the body out along the Hit’s normal
                    b.pos += h.n * (h.depth - kSlop);
                    P = worldHull(s, b, mp);
                    moved = true;
                }
            }
            // no overlap
            if (!moved)
                break;
        }
        // linear acceleration
        info.acc = (b.vel - v0) * (1.0f / dt);
        return info;
    }

} // namespace Physics