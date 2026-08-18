#include <algorithm>
#include <cmath>
#include <vector>

#include "../ClipperUtils.hpp"
#include "../ExtrusionEntityCollection.hpp"
#include "../Polyline.hpp"
#include "../ShortestPath.hpp"
#include "../Surface.hpp"
#include "../VariableWidth.hpp"

#include "FillSymmetricWave.hpp"

namespace Slic3r {

namespace {

// A sine, every row the same phase, stacked at a constant vertical pitch H.
// Rows are translates of one curve, so they never meet. Shifting alternate rows
// by half a period instead would nest a peak into a trough and close a cell at
// every crossing, tiling the surface with diamonds.
//
// The base width is the dual of the channel between vertically offset copies:
//
//     b(x) = H1 * cos(theta(x)),   H1 = w_min * sqrt(1 + m^2)
//
// so the facing edges of two equal strokes would just touch at density 1. On top
// of that, alternate rows take opposite shares of a leg-centred modulation:
//
//     even:  w = b - D * cos(phase)   // widest on the descending leg
//     odd:   w = b + D * cos(phase)   // widest on the ascending leg
//
// Adjacent rows therefore keep w_even + w_odd = 2 b, so paired coverage is
// unchanged while the visible swell sits on opposite legs rather than at the
// turns. The slope has to vary continuously for that modulation to read; a
// filleted triangle wave would only move the width inside the fillets.
struct Wave
{
    coordf_t P{};     // period
    coordf_t A{};     // amplitude
    coordf_t m{};     // maximum slope, at the zero crossings
    coordf_t w_min{}; // base width at the steepest point (before parity term)
    coordf_t H1{};    // vertical extent of a paired even+odd stroke, /2 each on average
    coordf_t H{};     // row pitch
    coordf_t D{};     // parity amplitude; D = 0.5 * w_min

    coordf_t wave_y(coordf_t x) const { return A * std::sin(2 * PI * x / P); }

    // Cosine of the tangent angle: 1/sqrt(1 + m^2) at a zero crossing, 1 at a turn.
    coordf_t cos_theta(coordf_t x) const
    {
        const coordf_t slope = m * std::cos(2 * PI * x / P);
        return 1 / std::sqrt(1 + slope * slope);
    }

    // parity 0 = even (descending swell), parity 1 = odd (ascending swell).
    coordf_t width_at(coordf_t x, int parity) const
    {
        const coordf_t phase = 2 * PI * x / P;
        const coordf_t b     = H1 * cos_theta(x);
        const coordf_t delta = D * std::cos(phase);
        const coordf_t w     = (parity & 1) ? b + delta : b - delta;
        return std::max(coordf_t(SCALED_EPSILON), w);
    }

    // Recover the row index from a point on the centreline (infill frame).
    int row_parity(coordf_t x, coordf_t y) const
    {
        const int n = int(std::lround((y - wave_y(x)) / H));
        return n & 1;
    }
};

// Maximum slope. Softens the chevron look; peak-to-peak / pitch = c * m^2.
// The visible width swing now comes from the parity term D, not from m alone.
static constexpr double WAVE_MAX_SLOPE = 0.65;
// Margin on the period. Turns still carry width ~H1 where curvature peaks; at
// 1.0 the inner edge would cusp, so keep the wave a little longer than that.
static constexpr double WAVE_CURVATURE_MARGIN = 1.55;
// Fraction of w_min used as the parity amplitude. 0.5 yields ~0.20–0.60 mm at a
// 0.4 mm bead and clears the scale_(0.05) merge tolerance.
static constexpr double WAVE_PARITY_FRAC = 0.5;

Wave make_wave(coordf_t w_min, float density)
{
    Wave w;
    w.m     = WAVE_MAX_SLOPE;
    w.w_min = w_min;
    w.D     = WAVE_PARITY_FRAC * w_min;
    w.H1    = w_min * std::sqrt(1 + w.m * w.m);
    // Everything is in units of the bead, not this->spacing: the width, the row
    // pitch and the curvature limit are all relations between the stroke and
    // itself, so the motif keeps its proportions whatever the flow is.
    w.P = std::round(WAVE_CURVATURE_MARGIN * PI * w.m * w.H1);
    w.A = w.m * w.P / (2 * PI);
    w.H = std::round(w.H1 / std::max(density, 1e-6f));
    return w;
}

// One period of the centreline, x in [0, P].
std::vector<Vec2d> period_points(const Wave &W, coordf_t tol)
{
    // Chord error over a segment of length L on radius rho is about L^2 / (8 rho).
    // The tightest radius is at the turns.
    const coordf_t rho_min = W.P / (2 * PI * W.m);
    const coordf_t step    = std::sqrt(std::max(8 * rho_min * tol, coordf_t(1)));
    const size_t   n       = std::min<size_t>(1024, std::max<size_t>(8, size_t(std::ceil(W.P / step))));

    std::vector<Vec2d> pts;
    pts.reserve(n + 1);
    for (size_t i = 0; i <= n; ++i) {
        const coordf_t x = W.P * coordf_t(i) / coordf_t(n);
        pts.emplace_back(x, W.wave_y(x));
    }
    return pts;
}

Polyline to_polyline(const std::vector<Vec2d> &pts)
{
    Polyline pl;
    pl.points.reserve(pts.size());
    for (const Vec2d &p : pts) {
        const Point q(coord_t(std::lround(p.x())), coord_t(std::lround(p.y())));
        if (pl.points.empty() || pl.points.back() != q)
            pl.points.emplace_back(q);
    }
    return pl;
}

ThickPolyline to_thick(const Polyline &pl, const Wave &W, float infill_angle)
{
    ThickPolyline tp;
    tp.points = pl.points;
    if (tp.points.size() < 2)
        return tp;
    tp.width.reserve((tp.points.size() - 1) * 2);
    // Width comes from the phase and row parity of the clipped point, so
    // fragments taper correctly without carrying a width array through the clip.
    auto width_at_pt = [&](const Point &p) {
        Point q = p;
        q.rotate(infill_angle);
        const coordf_t x = coordf_t(q.x());
        const coordf_t y = coordf_t(q.y());
        return W.width_at(x, W.row_parity(x, y));
    };
    for (size_t i = 0; i + 1 < tp.points.size(); ++i) {
        tp.width.push_back(width_at_pt(tp.points[i]));
        tp.width.push_back(width_at_pt(tp.points[i + 1]));
    }
    return tp;
}

// Clipper is free to hand a fragment back either way round. Orient every path
// left-to-right in the infill frame so neighbouring rows start on the same side.
void orient_ltr(Polyline &pl, float infill_angle)
{
    if (pl.points.size() < 2)
        return;
    Point a = pl.points.front();
    Point b = pl.points.back();
    a.rotate(infill_angle);
    b.rotate(infill_angle);
    if (b.x() < a.x())
        pl.reverse();
}

// Sort clipped fragments of one row by their leftmost infill-frame x.
void sort_row_ltr(Polylines &row, float infill_angle)
{
    auto key_x = [infill_angle](const Polyline &pl) {
        Point a = pl.points.front();
        Point b = pl.points.back();
        a.rotate(infill_angle);
        b.rotate(infill_angle);
        return std::min(a.x(), b.x());
    };
    std::sort(row.begin(), row.end(), [&](const Polyline &a, const Polyline &b) { return key_x(a) < key_x(b); });
}

// Infill-frame x span of an already L→R oriented fragment.
void frag_x_span(const Polyline &pl, float infill_angle, coord_t &xmin, coord_t &xmax)
{
    Point a = pl.points.front();
    Point b = pl.points.back();
    a.rotate(infill_angle);
    b.rotate(infill_angle);
    xmin = std::min(a.x(), b.x());
    xmax = std::max(a.x(), b.x());
}

struct FrontierSpan
{
    coord_t xmin;
    coord_t xmax;
    size_t  owner;
};

bool x_overlap(coord_t a0, coord_t a1, coord_t b0, coord_t b1)
{
    return !(a1 < b0 || b1 < a0);
}

} // namespace

void FillSymmetricWave::fill_surface_extrusion(const Surface *surface, const FillParams &params,
                                               ExtrusionEntitiesPtr &out)
{
    if (params.density <= 0 || this->spacing <= 0)
        return;

    const ExPolygons expp = offset_ex(surface->expolygon, float(scale_(this->overlap - 0.5 * this->spacing)));
    if (expp.empty())
        return;

    const Wave W = make_wave(coordf_t(params.flow.scaled_spacing()), params.density);
    if (W.P < 1 || W.H < 1)
        return;

    const std::pair<float, Point> direction = _infill_direction(surface);
    const coordf_t                tolerance = std::max(scaled<double>(params.resolution), scaled<double>(0.001));
    const std::vector<Vec2d>      period    = period_points(W, tolerance);
    const coord_t                 pitch_x   = coord_t(W.P);
    const coord_t                 pitch_y   = coord_t(W.H);
    const coord_t                 pad       = coord_t(std::lround(W.A + W.H1));

    std::vector<OrientedPathRegion>  regions;
    std::vector<std::vector<size_t>> predecessors;
    Point                            start_near(0, 0);
    bool                             have_start = false;

    for (const ExPolygon &expolygon : expp) {
        BoundingBox bbox = expolygon.contour.bounding_box();
        {
            Polygon bb_polygon = bbox.polygon();
            bb_polygon.rotate(direction.first);
            bbox = bb_polygon.bounding_box();
        }
        bbox.offset(pad);
        // Motif repeats every P in x. Rows repeat every H, but parity of the
        // width law repeats every 2H, so align y to 2H so separate islands
        // agree on which rows swell on the ascending vs descending leg.
        const Point origin = align_to_grid(bbox.min, Point(pitch_x, pitch_y * 2));

        // Clip per row so Clipper's PolyTree cannot scramble bottom-to-top
        // generation. Fragments are then grouped into oriented regions and
        // chained under an x-interval frontier DAG so holes are crossed once
        // per channel merge rather than once per row.
        const int                 n_max = int(std::ceil((coordf_t(bbox.max.y()) - coordf_t(origin.y())) / W.H));
        const int                 i_max = int(std::ceil((coordf_t(bbox.max.x()) - coordf_t(origin.x())) / W.P));
        std::vector<FrontierSpan> frontier;

        for (int n = 0; n <= n_max; ++n) {
            const coordf_t     y_base = coordf_t(origin.y()) + coordf_t(n) * W.H;
            std::vector<Vec2d> pts;
            pts.reserve(period.size() * size_t(std::max(1, i_max + 1)));
            for (int i = 0; i <= i_max; ++i) {
                const coordf_t x0 = coordf_t(origin.x()) + coordf_t(i) * W.P;
                const size_t   i0 = pts.empty() ? 0 : 1;
                for (size_t k = i0; k < period.size(); ++k)
                    pts.emplace_back(x0 + period[k].x(), y_base + period[k].y());
            }
            Polyline pl = to_polyline(pts);
            if (pl.points.size() < 2)
                continue;
            pl.rotate(-direction.first);

            Polylines row = intersection_pl(Polylines{std::move(pl)}, expolygon);
            sort_row_ltr(row, direction.first);

            std::vector<FrontierSpan> row_spans;
            row_spans.reserve(row.size());
            size_t prev_in_row = size_t(-1);
            for (Polyline &frag : row) {
                orient_ltr(frag, direction.first);
                if (frag.points.size() < 2)
                    continue;

                coord_t xmin, xmax;
                frag_x_span(frag, direction.first, xmin, xmax);

                OrientedPathRegion region;
                region.paths.emplace_back(std::move(frag));
                regions.emplace_back(std::move(region));
                predecessors.emplace_back();

                if (prev_in_row != size_t(-1))
                    predecessors.back().push_back(prev_in_row);
                for (const FrontierSpan &sp : frontier)
                    if (x_overlap(xmin, xmax, sp.xmin, sp.xmax))
                        predecessors.back().push_back(sp.owner);

                const size_t idx = regions.size() - 1;
                if (!have_start) {
                    start_near = regions.back().start();
                    have_start = true;
                }
                row_spans.push_back({xmin, xmax, idx});
                prev_in_row = idx;
            }

            // Replace frontier coverage with this row's fragments so the next
            // non-empty row depends on the most recent owners at each x.
            for (const FrontierSpan &sp : row_spans) {
                frontier.erase(std::remove_if(frontier.begin(), frontier.end(),
                                              [&](const FrontierSpan &old) {
                                                  return x_overlap(sp.xmin, sp.xmax, old.xmin, old.xmax);
                                              }),
                               frontier.end());
                frontier.push_back(sp);
            }
        }
    }

    if (regions.empty())
        return;

    const std::vector<size_t> order =
        chain_oriented_regions(regions, predecessors, have_start ? &start_near : nullptr);
    const Polylines ordered = flatten_oriented_regions(regions, order);

    ThickPolylines thick;
    thick.reserve(ordered.size());
    for (const Polyline &frag : ordered) {
        ThickPolyline tp = to_thick(frag, W, direction.first);
        if (tp.points.size() >= 2 && tp.width.size() == (tp.points.size() - 1) * 2)
            thick.emplace_back(std::move(tp));
    }
    if (thick.empty())
        return;

    auto *eec = new ExtrusionEntityCollection();
    eec->no_sort = this->no_sort();
    out.push_back(eec);
    variable_width(thick, params.extrusion_role, params.flow, eec->entities);
    this->_create_gap_fill(surface, params, eec);
}

} // namespace Slic3r
