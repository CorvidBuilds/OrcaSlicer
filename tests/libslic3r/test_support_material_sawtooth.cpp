#include <catch2/catch_all.hpp>

#include "libslic3r/Support/SawtoothInterface.hpp"

using namespace Slic3r;

// Nozzle-derived proportions, mirroring sawtooth_params() for a 0.4 mm nozzle.
static SawtoothParams make_params(double d = 0.4, double h = 0.2)
{
    SawtoothParams p;
    p.tooth_height = scale_(h);
    p.spacing_min  = scale_(d);
    p.spacing_max  = scale_(3. * d);
    p.ramp         = scale_(0.5 * d);
    p.clearance    = scale_(2.5 * d);
    return p;
}

static Polyline straight_line(double length_mm)
{
    Polyline pl;
    pl.points.emplace_back(Point(coord_t(0), coord_t(0)));
    pl.points.emplace_back(Point(scale_(length_mm), coord_t(0)));
    return pl;
}

TEST_CASE("Sawtooth teeth on a straight interface line", "[SupportMaterial]")
{
    const double d = 0.4, h = 0.2, length = 50.;
    const SawtoothParams p = make_params(d, h);

    Polyline3 out;
    REQUIRE(add_sawtooth_teeth(straight_line(length), p, out));

    size_t tips = 0;
    for (const Point3 &pt : out.points) {
        // Every point is either at the tip height or flat on the layer.
        if (pt.z() == p.tooth_height)
            ++ tips;
        else
            REQUIRE(pt.z() == 0);
    }
    // Each tooth contributes two tip points (the flat top).
    REQUIRE(tips % 2 == 0);
    const size_t teeth = tips / 2;

    // Pitch is the gap (in [spacing_min, spacing_max)) plus the tooth's own run of 3 ramps.
    const double run = 1.5 * d, usable = length - 2.5 * d;
    REQUIRE(teeth >= size_t(usable / (3. * d + run)));
    REQUIRE(teeth <= size_t(usable / (d + run)) + 1);

    // The 2-D projection is unchanged: still the same straight line, still monotonic.
    coord_t prev = -1;
    for (const Point3 &pt : out.points) {
        REQUIRE(pt.y() == 0);
        REQUIRE(pt.x() >= prev);
        prev = pt.x();
    }
    REQUIRE(out.points.front().x() == 0);
    REQUIRE(out.points.back().x() == scale_(length));
}

TEST_CASE("Sawtooth skips lines shorter than the clearance", "[SupportMaterial]")
{
    const SawtoothParams p = make_params();
    Polyline3 out;
    REQUIRE(! add_sawtooth_teeth(straight_line(0.5), p, out));
    for (const Point3 &pt : out.points)
        REQUIRE(pt.z() == 0);
}

TEST_CASE("Sawtooth is a no-op with zero tooth height", "[SupportMaterial]")
{
    SawtoothParams p = make_params();
    p.tooth_height = 0;
    Polyline3 out;
    REQUIRE(! add_sawtooth_teeth(straight_line(50.), p, out));
    REQUIRE(out.points.empty());
}

TEST_CASE("Sawtooth is deterministic and de-aligns adjacent rows", "[SupportMaterial]")
{
    const SawtoothParams p = make_params();

    // Same input twice must give identical output (no rand(), safe under TBB).
    Polyline3 a, b;
    add_sawtooth_teeth(straight_line(50.), p, a);
    add_sawtooth_teeth(straight_line(50.), p, b);
    REQUIRE(a.points.size() == b.points.size());
    for (size_t i = 0; i < a.points.size(); ++ i)
        REQUIRE(a.points[i] == b.points[i]);

    // A row 1 mm over must not have its teeth in the same places.
    Polyline shifted = straight_line(50.);
    for (Point &pt : shifted.points)
        pt.y() += scale_(1.);
    Polyline3 c;
    add_sawtooth_teeth(shifted, p, c);
    bool differs = c.points.size() != a.points.size();
    for (size_t i = 0; ! differs && i < c.points.size(); ++ i)
        differs = c.points[i].x() != a.points[i].x();
    REQUIRE(differs);
}
