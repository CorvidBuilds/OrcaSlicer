#include "SawtoothInterface.hpp"

#include "../ExtrusionEntityCollection.hpp"
#include "../PrintConfig.hpp"
#include "../Flow.hpp"

#include <algorithm>
#include <cstdint>

namespace Slic3r {

SawtoothParams sawtooth_params(const PrintObjectConfig &object_config, const Flow &interface_flow)
{
    SawtoothParams p;
    if (object_config.support_interface_pattern != smipSawtooth)
        return p;
    const double gap = object_config.support_top_z_distance.value;
    const double d   = interface_flow.nozzle_diameter();
    double h = object_config.support_interface_tooth_height.value;
    if (h <= 0)
        h = std::min(gap, d);
    // Zero Z gap (soluble / zero-gap interfaces): nowhere to hop into, feature is a no-op.
    if (h <= EPSILON)
        return p;
    p.tooth_height = scale_(h);
    p.spacing_min  = scale_(d);
    p.spacing_max  = scale_(3. * d);
    p.ramp         = scale_(0.5 * d);
    p.clearance    = scale_(2.5 * d);
    return p;
}

// Deterministic, lock-free replacement for the reference implementation's rand():
// de-aligns teeth between adjacent rows without a shared RNG (we run inside TBB).
static uint32_t point_hash(const Point &pt)
{
    uint64_t h = uint64_t(uint32_t(pt.x())) * 0x9E3779B97F4A7C15ull
               ^ uint64_t(uint32_t(pt.y())) * 0xC2B2AE3D27D4EB4Full;
    h ^= h >> 29; h *= 0xBF58476D1CE4E5B9ull; h ^= h >> 32;
    return uint32_t(h);
}

bool add_sawtooth_teeth(const Polyline &pl, const SawtoothParams &p, Polyline3 &out)
{
    out.points.clear();
    if (p.tooth_height <= 0 || pl.points.size() < 2)
        return false;

    // A tooth occupies: ramp up, flat tip, ramp down.
    const coord_t tooth_run = 3 * p.ramp;
    const coord_t range     = std::max<coord_t>(1, p.spacing_max - p.spacing_min);
    bool any = false;

    out.points.reserve(pl.points.size());
    out.points.emplace_back(pl.points.front().x(), pl.points.front().y(), coord_t(0));
    for (size_t i = 1; i < pl.points.size(); ++ i) {
        const Point &a   = pl.points[i - 1];
        const Point &b   = pl.points[i];
        const double len = (b - a).cast<double>().norm();
        if (len >= double(p.clearance) && len >= double(tooth_run)) {
            const Vec2d dir     = (b - a).cast<double>() / len;
            // Gap between consecutive teeth; the tooth's own run is added on top, so teeth never overlap.
            const coord_t step  = p.spacing_min + coord_t(point_hash(a) % uint32_t(range));
            auto at = [&](double t) {
                Vec2d q = a.cast<double>() + dir * t;
                return Point(coord_t(std::round(q.x())), coord_t(std::round(q.y())));
            };
            for (double t = double(step); t + tooth_run <= len - double(p.clearance); t += double(step) + tooth_run) {
                Point q0 = at(t), q1 = at(t + p.ramp), q2 = at(t + 2 * p.ramp), q3 = at(t + tooth_run);
                out.points.emplace_back(q0.x(), q0.y(), coord_t(0));
                out.points.emplace_back(q1.x(), q1.y(), p.tooth_height);
                out.points.emplace_back(q2.x(), q2.y(), p.tooth_height);
                out.points.emplace_back(q3.x(), q3.y(), coord_t(0));
                any = true;
            }
        }
        out.points.emplace_back(b.x(), b.y(), coord_t(0));
    }
    return any;
}

static void apply_to_path(ExtrusionPath &path, const SawtoothParams &p)
{
    if (path.role() != erSupportMaterialInterface || path.z_contoured)
        return;
    Polyline3 toothed;
    if (add_sawtooth_teeth(path.polyline.to_polyline(), p, toothed)) {
        path.polyline   = std::move(toothed);
        path.z_contoured = true;
    }
    // No tooth fitted: leave the path 2-D so G-code stays on the fast path.
}

static void apply_to_entity(ExtrusionEntity *e, const SawtoothParams &p)
{
    if (auto *mp = dynamic_cast<ExtrusionMultiPath*>(e)) {
        for (ExtrusionPath &path : mp->paths)
            apply_to_path(path, p);
    } else if (auto *path = dynamic_cast<ExtrusionPath*>(e)) {
        apply_to_path(*path, p);
    } else if (auto *coll = dynamic_cast<ExtrusionEntityCollection*>(e)) {
        for (ExtrusionEntity *child : coll->entities)
            apply_to_entity(child, p);
    }
    // ponytail: loops (the interface perimeter) are left flat — the teeth belong on the
    // hatching that touches the object, and a contoured closed loop would need a seam story.
}

void apply_sawtooth_teeth(ExtrusionEntitiesPtr &entities, size_t first_entity, const SawtoothParams &p)
{
    if (p.tooth_height <= 0)
        return;
    for (size_t i = first_entity; i < entities.size(); ++ i)
        apply_to_entity(entities[i], p);
}

} // namespace Slic3r
