#ifndef slic3r_ShortestPath_hpp_
#define slic3r_ShortestPath_hpp_

#include "libslic3r.h"
#include "ExtrusionEntity.hpp"
#include "Point.hpp"

#include <utility>
#include <vector>

namespace Slic3r {

	namespace ClipperLib {
		class PolyNode;
		using PolyNodes = std::vector<PolyNode*, PointsAllocator<PolyNode*>>;
	}

std::vector<size_t> 				 chain_points(const Points &points, const Point *start_near = nullptr);
// Variant with post-processing (crossing removal + 2-opt) for object ordering.
std::vector<size_t> 				 chain_points_with_postprocessing(const Points &points, const Point *start_near = nullptr);
std::vector<size_t> 				 chain_expolygons(const ExPolygons &input_exploy);

std::vector<std::pair<size_t, bool>> chain_extrusion_entities(std::vector<ExtrusionEntity*> &entities, const Point *start_near = nullptr);
void                                 reorder_extrusion_entities(std::vector<ExtrusionEntity*> &entities, const std::vector<std::pair<size_t, bool>> &chain);
void                                 chain_and_reorder_extrusion_entities(std::vector<ExtrusionEntity*> &entities, const Point &start_near);
void                                 chain_and_reorder_extrusion_entities(std::vector<ExtrusionEntity*> &entities, const Point *start_near = nullptr);

std::vector<std::pair<size_t, bool>> chain_extrusion_paths(std::vector<ExtrusionPath> &extrusion_paths, const Point *start_near = nullptr);
void                                 reorder_extrusion_paths(std::vector<ExtrusionPath> &extrusion_paths, std::vector<std::pair<size_t, bool>> &chain);
void                                 chain_and_reorder_extrusion_paths(std::vector<ExtrusionPath> &extrusion_paths, const Point *start_near = nullptr);

Polylines 							 chain_polylines(Polylines &&src, const Point *start_near = nullptr);
inline Polylines 					 chain_polylines(const Polylines& src, const Point* start_near = nullptr) { Polylines tmp(src); return chain_polylines(std::move(tmp), start_near); }
template<typename T> inline void reorder_by_shortest_traverse(std::vector<T> &polylines_out)
{
    Points start_point;
    start_point.reserve(polylines_out.size());
    for (const T& contour : polylines_out) start_point.push_back(contour.points.front());

    std::vector<Points::size_type> order = chain_points(start_point);

    std::vector<T> Temp = polylines_out;
    polylines_out.erase(polylines_out.begin(), polylines_out.end());

    for (size_t i:order) polylines_out.emplace_back(std::move(Temp[i]));
}

// Contiguous oriented open paths that must print as a block. The chainer reorders
// regions only — never reverses a path inside a region.
//
// Invariants for chain_oriented_regions:
// - every region has at least one non-empty path
// - predecessors[i] indices are in range; the graph is a DAG
// - output is a total permutation; no polyline is reversed
// - ExtrusionEntityCollection::no_sort remains the caller's responsibility
struct OrientedPathRegion
{
    Polylines paths;

    Point start() const { return paths.front().first_point(); }
    Point end()   const { return paths.back().last_point(); }
};

// Reorder regions to cut end→start travel while honouring predecessors.
// Empty predecessors ⇒ pure greedy among remaining regions (still no reverse).
// ponytail: greedy NN only — fine for Wave/Scales strip counts; upgrade to
// pheromone (see FillRectilinear monotonic) if a profile shows long jumps.
std::vector<size_t> chain_oriented_regions(
    const std::vector<OrientedPathRegion>       &regions,
    const std::vector<std::vector<size_t>>      &predecessors,
    const Point                                 *start_near = nullptr);

Polylines flatten_oriented_regions(
    const std::vector<OrientedPathRegion> &regions,
    const std::vector<size_t>             &order);

ClipperLib::PolyNodes				 chain_clipper_polynodes(const Points &points, const ClipperLib::PolyNodes &items);

// Chain instances of print objects by an approximate shortest path.
// Returns pairs of PrintObject idx and instance of that PrintObject.
class Print;
struct PrintInstance;
// BBS
class PrintObject;
std::vector<const PrintInstance*> chain_print_object_instances(const std::vector<const PrintObject*>& print_objects, const Point* start_near);
std::vector<const PrintInstance*> 	 chain_print_object_instances(const Print &print);

// Chain lines into polylines.
Polylines 							 chain_lines(const std::vector<Line> &lines, const double point_distance_epsilon);

} // namespace Slic3r

#endif /* slic3r_ShortestPath_hpp_ */
