#include <algorithm>
#include <cmath>
#include <utility>

#include "../ClipperUtils.hpp"
#include "../Polygon.hpp"
#include "../ShortestPath.hpp"
#include "../Surface.hpp"

#include "FillHoneycomb.hpp"

namespace Slic3r {

void FillHoneycomb::_fill_surface_single(
    const FillParams                &params,
    unsigned int                     thickness_layers,
    const std::pair<float, Point>   &direction,
    ExPolygon                        expolygon,
    Polylines                       &polylines_out)
{
    // cache hexagons math
    CacheID cache_id(params.density, this->spacing);
    Cache::iterator it_m = this->cache.find(cache_id);
    if (it_m == this->cache.end()) {
        it_m = this->cache.insert(it_m, std::pair<CacheID, CacheData>(cache_id, CacheData()));
        CacheData &m        = it_m->second;
        coord_t min_spacing = coord_t(scale_(this->spacing)) * params.multiline;
        m.distance          = coord_t(min_spacing / params.density);
        m.hex_side          = coord_t(m.distance / (sqrt(3)/2));
        m.hex_width         = m.distance * 2; // $m->{hex_width} == $m->{hex_side} * sqrt(3);
        coord_t hex_height  = m.hex_side * 2;
        m.pattern_height    = hex_height + m.hex_side;
        m.y_short           = coord_t(m.distance * sqrt(3)/3);
        m.x_offset          = min_spacing / 2;
        m.y_offset          = coord_t(m.x_offset * sqrt(3)/3);
        m.hex_center        = Point(m.hex_width/2, m.hex_side);
    }
    CacheData &m = it_m->second;

    Polylines all_polylines;
    {
        // adjust actual bounding box to the nearest multiple of our hex pattern
        // and align it so that it matches across layers

        BoundingBox bounding_box = expolygon.contour.bounding_box();
        {
            // rotate bounding box according to infill direction
            Polygon bb_polygon = bounding_box.polygon();
            bb_polygon.rotate(direction.first, m.hex_center);
            bounding_box = bb_polygon.bounding_box();

            // extend bounding box so that our pattern will be aligned with other layers
            // $bounding_box->[X1] and [Y1] represent the displacement between new bounding box offset and old one
            // The infill is not aligned to the object bounding box, but to a world coordinate system. Supposedly good enough.
            bounding_box.merge(align_to_grid(bounding_box.min, Point(m.hex_width, m.pattern_height)));
        }

        coord_t x = bounding_box.min(0);
        while (x <= bounding_box.max(0)) {
            Polyline p;
            coord_t ax[2] = { x + m.x_offset, x + m.distance - m.x_offset };
            for (size_t i = 0; i < 2; ++ i) {
                std::reverse(p.points.begin(), p.points.end()); // turn first half upside down
                for (coord_t y = bounding_box.min(1); y <= bounding_box.max(1); y += m.y_short + m.hex_side + m.y_short + m.hex_side) {
                    p.points.push_back(Point(ax[1], y + m.y_offset));
                    p.points.push_back(Point(ax[0], y + m.y_short - m.y_offset));
                    p.points.push_back(Point(ax[0], y + m.y_short + m.hex_side + m.y_offset));
                    p.points.push_back(Point(ax[1], y + m.y_short + m.hex_side + m.y_short - m.y_offset));
                    p.points.push_back(Point(ax[1], y + m.y_short + m.hex_side + m.y_short + m.hex_side + m.y_offset));
                }
                ax[0] = ax[0] + m.distance;
                ax[1] = ax[1] + m.distance;
                std::swap(ax[0], ax[1]); // draw symmetrical pattern
                x += m.distance;
            }
            p.rotate(-direction.first, m.hex_center);
            p.simplify(5 * spacing); // simplify to 5x line width
            all_polylines.push_back(p);
        }
    }
    // Apply multiline offset if needed
    multiline_fill(all_polylines, params, spacing);

    all_polylines = intersection_pl(std::move(all_polylines), expolygon);
    chain_or_connect_infill(std::move(all_polylines), expolygon, polylines_out, this->spacing, params);
}

void FillHexagon::_fill_surface_single(
    const FillParams              &params,
    unsigned int                   /* thickness_layers */,
    const std::pair<float, Point> &direction,
    ExPolygon                      expolygon,
    Polylines                     &polylines_out)
{
    // u is the distance between neighbouring lines. The unfilled long diagonal is 5u;
    // the path is offset outward by half a line-width along the vertex (0.5 / cos(30°) = 1/√3).
    const double u = scale_(this->spacing) / double(params.density);
    const double R = u * (2.5 + 1. / std::sqrt(3.));
    // Neighbours sit far enough apart that facing walls are one line-width apart — two
    // touching beads instead of a shared centerline, so every wall is thick and every
    // path can close without revisiting a junction.
    const double  D          = R * std::sqrt(3.) + u;
    const coord_t pitch_x    = coord_t(std::round(D));
    const coord_t pitch_y    = coord_t(std::round(D * std::sqrt(3.) / 2.));
    const coord_t half_pitch = pitch_x / 2;
    const Point   hex_center(pitch_x / 2, pitch_y);

    BoundingBox bounding_box = expolygon.contour.bounding_box();
    {
        Polygon bb_polygon = bounding_box.polygon();
        bb_polygon.rotate(direction.first, hex_center);
        bounding_box = bb_polygon.bounding_box();
        bounding_box.offset(coord_t(std::round(R * 2.)));
        // Odd rows are staggered by half a cell, so the lattice repeats every two rows.
        bounding_box.merge(align_to_grid(bounding_box.min, Point(pitch_x, pitch_y * 2)));
    }

    const double half_w = R * std::sqrt(3.) / 2.;
    const double half_h = R / 2.;

    Polylines all_polylines;
    size_t    row = 0;
    for (coord_t cy = bounding_box.min(1); cy <= bounding_box.max(1); cy += pitch_y, ++ row) {
        for (coord_t cx = bounding_box.min(0) + ((row & 1) ? half_pitch : 0); cx <= bounding_box.max(0); cx += pitch_x) {
            // Pointy-top hex, CCW from the top vertex.
            Polygon hex;
            hex.points.reserve(6);
            hex.points.emplace_back(cx,                            cy + coord_t(std::round(R)));
            hex.points.emplace_back(cx + coord_t(std::round(half_w)), cy + coord_t(std::round(half_h)));
            hex.points.emplace_back(cx + coord_t(std::round(half_w)), cy - coord_t(std::round(half_h)));
            hex.points.emplace_back(cx,                            cy - coord_t(std::round(R)));
            hex.points.emplace_back(cx - coord_t(std::round(half_w)), cy - coord_t(std::round(half_h)));
            hex.points.emplace_back(cx - coord_t(std::round(half_w)), cy + coord_t(std::round(half_h)));
            Polyline p = hex.split_at_first_point();
            p.rotate(-direction.first, hex_center);
            all_polylines.push_back(std::move(p));
        }
    }

    all_polylines = intersection_pl(std::move(all_polylines), expolygon);

    // Same seam treatment as Concentric: leave a gap so the nozzle does not land back on
    // the start of the loop. Clipped boundary arcs stay open for connect_infill.
    size_t j = 0;
    for (size_t i = 0; i < all_polylines.size(); ++ i) {
        all_polylines[i].clip_end(this->loop_clipping);
        if (all_polylines[i].is_valid()) {
            if (j < i)
                all_polylines[j] = std::move(all_polylines[i]);
            ++ j;
        }
    }
    all_polylines.erase(all_polylines.begin() + j, all_polylines.end());

    chain_or_connect_infill(std::move(all_polylines), expolygon, polylines_out, this->spacing, params);
}

} // namespace Slic3r
