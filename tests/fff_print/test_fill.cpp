#include <catch2/catch_all.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Fill/Fill.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/SVG.hpp"
#include "libslic3r/libslic3r.h"

#include "test_helpers.hpp"

using namespace Slic3r;

bool test_if_solid_surface_filled(const ExPolygon& expolygon, double flow_spacing, double angle = 0, double density = 1.0);

#if 0
TEST_CASE("Adjusted solid distance", "[Fill]") {
    int surface_width = 250;
    int distance = Slic3r::Flow::solid_spacing(surface_width, 47);
    REQUIRE(distance == Catch::Approx(50));
    REQUIRE(surface_width % distance == 0);
}
#endif

TEST_CASE("Pattern path length", "[Fill]") {
    std::unique_ptr<Slic3r::Fill> filler(Slic3r::Fill::new_from_type("rectilinear"));
    filler->angle = float(-(PI)/2.0);
	FillParams fill_params;
	filler->spacing = 5;
	fill_params.dont_adjust = true;
	//fill_params.endpoints_overlap = false;
	fill_params.density = float(filler->spacing / 50.0);

    auto test = [&filler, &fill_params] (const ExPolygon& poly) -> Slic3r::Polylines {
        Slic3r::Surface surface(stTop, poly);
        return filler->fill_surface(&surface, fill_params);
    };

    SECTION("Square") {
        Slic3r::Points test_set;
        test_set.reserve(4);
        std::vector<Vec2d> points {Vec2d(0,0), Vec2d(100,0), Vec2d(100,100), Vec2d(0,100)};
        for (size_t i = 0; i < 4; ++i) {
            std::transform(points.cbegin()+i, points.cend(),   std::back_inserter(test_set), [] (const Vec2d& a) -> Point { return Point::new_scale(a.x(), a.y()); } ); 
            std::transform(points.cbegin(), points.cbegin()+i, std::back_inserter(test_set), [] (const Vec2d& a) -> Point { return Point::new_scale(a.x(), a.y()); } );
            Slic3r::Polylines paths = test(Slic3r::ExPolygon(test_set));
            REQUIRE(paths.size() == 1); // one continuous path

            // TODO: determine what the "Expected length" should be for rectilinear fill of a 100x100 polygon. 
            // This check only checks that it's above scale(3*100 + 2*50) + scaled_epsilon.
            // ok abs($paths->[0]->length - scale(3*100 + 2*50)) - scaled_epsilon, 'path has expected length';
            REQUIRE(std::abs(paths[0].length() - static_cast<double>(scale_(3*100 + 2*50))) - SCALED_EPSILON > 0); // path has expected length

            test_set.clear();
        }
    }
    SECTION("Diamond with endpoints on grid") {
        std::vector<Vec2d> points {Vec2d(0,0), Vec2d(100,0), Vec2d(150,50), Vec2d(100,100), Vec2d(0,100), Vec2d(-50,50)};
        Slic3r::Points test_set;
        test_set.reserve(6);
        std::transform(points.cbegin(), points.cend(),   std::back_inserter(test_set), [] (const Vec2d& a) -> Point { return Point::new_scale(a.x(), a.y()); } );
        Slic3r::Polylines paths = test(Slic3r::ExPolygon(test_set));
        REQUIRE(paths.size() == 1); // one continuous path
    }

    SECTION("Square with hole") {
        std::vector<Vec2d> square {Vec2d(0,0), Vec2d(100,0), Vec2d(100,100), Vec2d(0,100)};
        std::vector<Vec2d> hole {Vec2d(25,25), Vec2d(75,25), Vec2d(75,75), Vec2d(25,75) };
        std::reverse(hole.begin(), hole.end());

        Slic3r::Points test_hole;
        Slic3r::Points test_square;

        std::transform(square.cbegin(), square.cend(), std::back_inserter(test_square), [] (const Vec2d& a) -> Point { return Point::new_scale(a.x(), a.y()); } );
        std::transform(hole.cbegin(), hole.cend(), std::back_inserter(test_hole), [] (const Vec2d& a) -> Point { return Point::new_scale(a.x(), a.y()); } );

        for (double angle : {-(PI/2.0), -(PI/4.0), -(PI), PI/2.0, PI}) {
            for (double spacing : {25.0, 5.0, 7.5, 8.5}) {
				fill_params.density = float(filler->spacing / spacing);
                filler->angle = float(angle);
                ExPolygon e(test_square, test_hole);
                Slic3r::Polylines paths = test(e);
#if 0
				{
					BoundingBox bbox = get_extents(e);
					SVG svg("c:\\data\\temp\\square_with_holes.svg", bbox);
					svg.draw(e);
					svg.draw(paths);
					svg.Close();
				}
#endif
                REQUIRE((paths.size() >= 1 && paths.size() <= 3));
                // paths don't cross hole
                REQUIRE(diff_pl(paths, offset(e, float(SCALED_EPSILON*10))).size() == 0);
            }
        }
    }
    SECTION("Regression: Missing infill segments in some rare circumstances") {
        filler->angle = float(PI/4.0);
		fill_params.dont_adjust = false;
        filler->spacing = 0.654498;
        //filler->endpoints_overlap = unscale(359974);
		fill_params.density = 1;
        filler->layer_id = 66;
        filler->z = 20.15;

        Slic3r::Points points {Point(25771516,14142125),Point(14142138,25771515),Point(2512749,14142131),Point(14142125,2512749)};
        Slic3r::Polylines paths = test(Slic3r::ExPolygon(points));
        REQUIRE(paths.size() == 1); // one continuous path

        // TODO: determine what the "Expected length" should be for rectilinear fill of a 100x100 polygon. 
        // This check only checks that it's above scale(3*100 + 2*50) + scaled_epsilon.
        // ok abs($paths->[0]->length - scale(3*100 + 2*50)) - scaled_epsilon, 'path has expected length';
        REQUIRE(std::abs(paths[0].length() - static_cast<double>(scale_(3*100 + 2*50))) - SCALED_EPSILON > 0); // path has expected length
    }

    SECTION("Rotated Square") {
        Slic3r::Points square { Point::new_scale(0,0), Point::new_scale(50,0), Point::new_scale(50,50), Point::new_scale(0,50)};
        Slic3r::ExPolygon expolygon(square);
        std::unique_ptr<Slic3r::Fill> filler(Slic3r::Fill::new_from_type("rectilinear"));
		filler->bounding_box = get_extents(expolygon.contour);
        filler->angle = 0;
        
        Surface surface(stTop, expolygon);
        auto flow = Slic3r::Flow(0.69f, 0.4f, 0.50f);

		FillParams fill_params;
		fill_params.density = 1.0;
		filler->spacing = flow.spacing();

        for (auto angle : { 0.0, 45.0}) {
            surface.expolygon.rotate(angle, Point(0,0));
            Polylines paths = filler->fill_surface(&surface, fill_params);
            REQUIRE(paths.size() == 1);
        }
    }

    #if 0   // Disabled temporarily due to precision issues on the Mac VM
    SECTION("Solid surface fill") {
        Slic3r::Points points {
            Point::new_scale(6883102, 9598327.01296997),
            Point::new_scale(6883102, 20327272.01297),
            Point::new_scale(3116896, 20327272.01297),
            Point::new_scale(3116896, 9598327.01296997) 
        };
        Slic3r::ExPolygon expolygon(points);
         
        REQUIRE(test_if_solid_surface_filled(expolygon, 0.55) == true);
        for (size_t i = 0; i <= 20; ++i)
        {
            expolygon.scale(1.05);
            REQUIRE(test_if_solid_surface_filled(expolygon, 0.55) == true);
        }
    }
    #endif

    SECTION("Solid surface fill") {
        Slic3r::Points points {
                Slic3r::Point(59515297,5422499),Slic3r::Point(59531249,5578697),Slic3r::Point(59695801,6123186),
                Slic3r::Point(59965713,6630228),Slic3r::Point(60328214,7070685),Slic3r::Point(60773285,7434379),
                Slic3r::Point(61274561,7702115),Slic3r::Point(61819378,7866770),Slic3r::Point(62390306,7924789),
                Slic3r::Point(62958700,7866744),Slic3r::Point(63503012,7702244),Slic3r::Point(64007365,7434357),
                Slic3r::Point(64449960,7070398),Slic3r::Point(64809327,6634999),Slic3r::Point(65082143,6123325),
                Slic3r::Point(65245005,5584454),Slic3r::Point(65266967,5422499),Slic3r::Point(66267307,5422499),
                Slic3r::Point(66269190,8310081),Slic3r::Point(66275379,17810072),Slic3r::Point(66277259,20697500),
                Slic3r::Point(65267237,20697500),Slic3r::Point(65245004,20533538),Slic3r::Point(65082082,19994444),
                Slic3r::Point(64811462,19488579),Slic3r::Point(64450624,19048208),Slic3r::Point(64012101,18686514),
                Slic3r::Point(63503122,18415781),Slic3r::Point(62959151,18251378),Slic3r::Point(62453416,18198442),
                Slic3r::Point(62390147,18197355),Slic3r::Point(62200087,18200576),Slic3r::Point(61813519,18252990),
                Slic3r::Point(61274433,18415918),Slic3r::Point(60768598,18686517),Slic3r::Point(60327567,19047892),
                Slic3r::Point(59963609,19493297),Slic3r::Point(59695865,19994587),Slic3r::Point(59531222,20539379),
                Slic3r::Point(59515153,20697500),Slic3r::Point(58502480,20697500),Slic3r::Point(58502480,5422499)
        };
        Slic3r::ExPolygon expolygon(points);
         
        REQUIRE(test_if_solid_surface_filled(expolygon, 0.55) == true);
        REQUIRE(test_if_solid_surface_filled(expolygon, 0.55, PI/2.0) == true);
    }
    SECTION("Solid surface fill") {
        Slic3r::Points points {
            Point::new_scale(0,0),Point::new_scale(98,0),Point::new_scale(98,10), Point::new_scale(0,10)
        };
        Slic3r::ExPolygon expolygon(points);
         
        REQUIRE(test_if_solid_surface_filled(expolygon, 0.5, 45.0, 0.99) == true);
    }
}

/*
{
    my $collection = Slic3r::Polyline::Collection->new(
            Slic3r::Polyline->new([0,15], [0,18], [0,20]),
            Slic3r::Polyline->new([0,10], [0,8], [0,5]),
            );
    is_deeply
        [ map $_->[Y], map @$_, @{$collection->chained_path_from(Slic3r::Point->new(0,30), 0)} ],
        [20, 18, 15, 10, 8, 5],
        'chained path';
}

{
    my $collection = Slic3r::Polyline::Collection->new(
            Slic3r::Polyline->new([4,0], [10,0], [15,0]),
            Slic3r::Polyline->new([10,5], [15,5], [20,5]),
            );
    is_deeply
        [ map $_->[X], map @$_, @{$collection->chained_path_from(Slic3r::Point->new(30,0), 0)} ],
        [reverse 4, 10, 15, 10, 15, 20],
        'chained path';
}

{
    my $collection = Slic3r::ExtrusionPath::Collection->new(
            map Slic3r::ExtrusionPath->new(polyline => $_, role => 0, mm3_per_mm => 1),
            Slic3r::Polyline->new([0,15], [0,18], [0,20]),
            Slic3r::Polyline->new([0,10], [0,8], [0,5]),
            );
    is_deeply
        [ map $_->[Y], map @{$_->polyline}, @{$collection->chained_path_from(Slic3r::Point->new(0,30), 0)} ],
        [20, 18, 15, 10, 8, 5],
        'chained path';
}

{
    my $collection = Slic3r::ExtrusionPath::Collection->new(
            map Slic3r::ExtrusionPath->new(polyline => $_, role => 0, mm3_per_mm => 1),
            Slic3r::Polyline->new([15,0], [10,0], [4,0]),
            Slic3r::Polyline->new([10,5], [15,5], [20,5]),
            );
    is_deeply
        [ map $_->[X], map @{$_->polyline}, @{$collection->chained_path_from(Slic3r::Point->new(30,0), 0)} ],
        [reverse 4, 10, 15, 10, 15, 20],
        'chained path';
}

for my $pattern (qw(rectilinear honeycomb hilbertcurve concentric)) {
    my $config = Slic3r::Config->new_from_defaults;
    $config->set('fill_pattern', $pattern);
    $config->set('external_fill_pattern', $pattern);
    $config->set('perimeters', 1);
    $config->set('skirts', 0);
    $config->set('fill_density', 20);
    $config->set('layer_height', 0.05);
    $config->set('perimeter_extruder', 1);
    $config->set('infill_extruder', 2);
    my $print = Slic3r::Test::init_print('20mm_cube', config => $config, scale => 2);
    ok my $gcode = Slic3r::Test::gcode($print), "successful $pattern infill generation";
    my $tool = undef;
    my @perimeter_points = my @infill_points = ();
    Slic3r::GCode::Reader->new->parse($gcode, sub {
            my ($self, $cmd, $args, $info) = @_;

            if ($cmd =~ /^T(\d+)/) {
            $tool = $1;
            } elsif ($cmd eq 'G1' && $info->{extruding} && $info->{dist_XY} > 0) {
            if ($tool == $config->perimeter_extruder-1) {
            push @perimeter_points, Slic3r::Point->new_scale($args->{X}, $args->{Y});
            } elsif ($tool == $config->infill_extruder-1) {
            push @infill_points, Slic3r::Point->new_scale($args->{X}, $args->{Y});
            }
            }
            });
    my $convex_hull = convex_hull(\@perimeter_points);
    ok !(defined first { !$convex_hull->contains_point($_) } @infill_points), "infill does not exceed perimeters ($pattern)";
}

{
    my $config = Slic3r::Config->new_from_defaults;
    $config->set('infill_only_where_needed', 1);
    $config->set('bottom_solid_layers', 0);
    $config->set('infill_extruder', 2);
    $config->set('infill_extrusion_width', 0.5);
    $config->set('fill_density', 40);
    $config->set('cooling', 0);                 # for preventing speeds from being altered
        $config->set('first_layer_speed', '100%');  # for preventing speeds from being altered

        my $test = sub {
            my $print = Slic3r::Test::init_print('pyramid', config => $config);

            my $tool = undef;
            my @infill_extrusions = ();  # array of polylines
                Slic3r::GCode::Reader->new->parse(Slic3r::Test::gcode($print), sub {
                        my ($self, $cmd, $args, $info) = @_;

                        if ($cmd =~ /^T(\d+)/) {
                        $tool = $1;
                        } elsif ($cmd eq 'G1' && $info->{extruding} && $info->{dist_XY} > 0) {
                        if ($tool == $config->infill_extruder-1) {
                        push @infill_extrusions, Slic3r::Line->new_scale(
                                [ $self->X, $self->Y ],
                                [ $info->{new_X}, $info->{new_Y} ],
                                );
                        }
                        }
                        });
            return 0 if !@infill_extrusions;  # prevent calling convex_hull() with no points

                my $convex_hull = convex_hull([ map $_->pp, map @$_, @infill_extrusions ]);
            return unscale unscale sum(map $_->area, @{offset([$convex_hull], scale(+$config->infill_extrusion_width/2))});
        };

    my $tolerance = 5;  # mm^2

        $config->set('solid_infill_below_area', 0);
    ok $test->() < $tolerance,
       'no infill is generated when using infill_only_where_needed on a pyramid';

    $config->set('solid_infill_below_area', 70);
    ok abs($test->() - $config->solid_infill_below_area) < $tolerance,
       'infill is only generated under the forced solid shells';
}

{
    my $config = Slic3r::Config->new_from_defaults;
    $config->set('skirts', 0);
    $config->set('perimeters', 1);
    $config->set('fill_density', 0);
    $config->set('top_solid_layers', 0);
    $config->set('bottom_solid_layers', 0);
    $config->set('solid_infill_below_area', 20000000);
    $config->set('solid_infill_every_layers', 2);
    $config->set('perimeter_speed', 99);
    $config->set('external_perimeter_speed', 99);
    $config->set('cooling', 0);
    $config->set('first_layer_speed', '100%');

    my $print = Slic3r::Test::init_print('20mm_cube', config => $config);
    my %layers_with_extrusion = ();
    Slic3r::GCode::Reader->new->parse(Slic3r::Test::gcode($print), sub {
            my ($self, $cmd, $args, $info) = @_;

            if ($cmd eq 'G1' && $info->{dist_XY} > 0 && $info->{extruding}) {
            if (($args->{F} // $self->F) != $config->perimeter_speed*60) {
            $layers_with_extrusion{$self->Z} = ($args->{F} // $self->F);
            }
            }
            });

    ok !%layers_with_extrusion,
       "solid_infill_below_area and solid_infill_every_layers are ignored when fill_density is 0";
}

{
    my $config = Slic3r::Config->new_from_defaults;
    $config->set('skirts', 0);
    $config->set('perimeters', 3);
    $config->set('fill_density', 0);
    $config->set('layer_height', 0.2);
    $config->set('first_layer_height', 0.2);
    $config->set('nozzle_diameter', [0.35]);
    $config->set('infill_extruder', 2);
    $config->set('solid_infill_extruder', 2);
    $config->set('infill_extrusion_width', 0.52);
    $config->set('solid_infill_extrusion_width', 0.52);
    $config->set('first_layer_extrusion_width', 0);

    my $print = Slic3r::Test::init_print('A', config => $config);
    my %infill = ();  # Z => [ Line, Line ... ]
        my $tool = undef;
    Slic3r::GCode::Reader->new->parse(Slic3r::Test::gcode($print), sub {
            my ($self, $cmd, $args, $info) = @_;

            if ($cmd =~ /^T(\d+)/) {
            $tool = $1;
            } elsif ($cmd eq 'G1' && $info->{extruding} && $info->{dist_XY} > 0) {
            if ($tool == $config->infill_extruder-1) {
            my $z = 1 * $self->Z;
            $infill{$z} ||= [];
            push @{$infill{$z}}, Slic3r::Line->new_scale(
                    [ $self->X, $self->Y ],
                    [ $info->{new_X}, $info->{new_Y} ],
                    );
            }
            }
            });
    my $grow_d = scale($config->infill_extrusion_width)/2;
    my $layer0_infill = union([ map @{$_->grow($grow_d)}, @{ $infill{0.2} } ]);
    my $layer1_infill = union([ map @{$_->grow($grow_d)}, @{ $infill{0.4} } ]);
    my $diff = diff($layer0_infill, $layer1_infill);
    $diff = offset2_ex($diff, -$grow_d, +$grow_d);
    $diff = [ grep { $_->area > 2*(($grow_d*2)**2) } @$diff ];
    is scalar(@$diff), 0, 'no missing parts in solid shell when fill_density is 0';
}

{
    # GH: #2697
    my $config = Slic3r::Config->new_from_defaults;
    $config->set('perimeter_extrusion_width', 0.72);
    $config->set('top_infill_extrusion_width', 0.1);
    $config->set('infill_extruder', 2);         # in order to distinguish infill
        $config->set('solid_infill_extruder', 2);   # in order to distinguish infill

        my $print = Slic3r::Test::init_print('20mm_cube', config => $config);
    my %infill = ();  # Z => [ Line, Line ... ]
        my %other  = ();  # Z => [ Line, Line ... ]
        my $tool = undef;
    Slic3r::GCode::Reader->new->parse(Slic3r::Test::gcode($print), sub {
            my ($self, $cmd, $args, $info) = @_;

            if ($cmd =~ /^T(\d+)/) {
            $tool = $1;
            } elsif ($cmd eq 'G1' && $info->{extruding} && $info->{dist_XY} > 0) {
            my $z = 1 * $self->Z;
            my $line = Slic3r::Line->new_scale(
                    [ $self->X, $self->Y ],
                    [ $info->{new_X}, $info->{new_Y} ],
                    );
            if ($tool == $config->infill_extruder-1) {
            $infill{$z} //= [];
            push @{$infill{$z}}, $line;
            } else {
            $other{$z} //= [];
            push @{$other{$z}}, $line;
            }
            }
            });
    my $top_z = max(keys %infill);
    my $top_infill_grow_d = scale($config->top_infill_extrusion_width)/2;
    my $top_infill = union([ map @{$_->grow($top_infill_grow_d)}, @{ $infill{$top_z} } ]);
    my $perimeters_grow_d = scale($config->perimeter_extrusion_width)/2;
    my $perimeters = union([ map @{$_->grow($perimeters_grow_d)}, @{ $other{$top_z} } ]);
    my $covered = union_ex([ @$top_infill, @$perimeters ]);
    my @holes = map @{$_->holes}, @$covered;
    ok sum(map unscale unscale $_->area*-1, @holes) < 1, 'no gaps between top solid infill and perimeters';
}
*/

bool test_if_solid_surface_filled(const ExPolygon& expolygon, double flow_spacing, double angle, double density)
{
    std::unique_ptr<Slic3r::Fill> filler(Slic3r::Fill::new_from_type("rectilinear"));
	filler->bounding_box = get_extents(expolygon.contour);
    filler->angle = float(angle);

	Flow flow(float(flow_spacing), 0.4f, float(flow_spacing));
	filler->spacing = flow.spacing();

	FillParams fill_params;
	fill_params.density = float(density);
	fill_params.dont_adjust = false;

	Surface surface(stBottom, expolygon);
	Slic3r::Polylines paths = filler->fill_surface(&surface, fill_params);

    // check whether any part was left uncovered
    Polygons grown_paths;
    grown_paths.reserve(paths.size());

    // figure out what is actually going on here re: data types
    float line_offset = float(scale_(filler->spacing / 2.0 + EPSILON));
    std::for_each(paths.begin(), paths.end(), [line_offset, &grown_paths] (const Slic3r::Polyline& p) {
        polygons_append(grown_paths, offset(p, line_offset));
    });

	// Shrink the initial expolygon a bit, this simulates the infill / perimeter overlap that we usually apply.
    ExPolygons uncovered = diff_ex(offset(expolygon, - float(0.2 * scale_(flow_spacing))), grown_paths, ApplySafetyOffset::Yes);

    // ignore very small dots
    const double scaled_flow_spacing = std::pow(scale_(flow_spacing), 2);
    uncovered.erase(std::remove_if(uncovered.begin(), uncovered.end(), [scaled_flow_spacing](const ExPolygon& poly) { return poly.area() < scaled_flow_spacing; }), uncovered.end());

#if 0
	if (! uncovered.empty()) {
		BoundingBox bbox = get_extents(expolygon.contour);
		bbox.merge(get_extents(uncovered));
		bbox.merge(get_extents(grown_paths));
		SVG svg("c:\\data\\temp\\test_if_solid_surface_filled.svg", bbox);
		svg.draw(expolygon);
		svg.draw(uncovered, "red");
		svg.Close();
	}
#endif

    return uncovered.empty(); // solid surface is fully filled
}

// Length-weighted dominant direction of the layer's role_wanted extrusions, whole degrees
// [0, 180), or -1 if it has none. Needs a line pattern such as monotonic or rectilinear.
template<typename RolePred> static int dominant_fill_angle(const Layer &layer, RolePred role_wanted)
{
    std::map<int, double> weight_per_degree;

    auto account = [&weight_per_degree, &role_wanted](const ExtrusionPath &path) {
        if (!role_wanted(path.role()))
            return;
        const Points3 &pts = path.polyline.points;
        for (size_t i = 1; i < pts.size(); ++i) {
            const double dx = double(pts[i].x() - pts[i - 1].x());
            const double dy = double(pts[i].y() - pts[i - 1].y());
            const double len = std::hypot(dx, dy);
            if (len <= 0.)
                continue;
            int deg = int(std::lround(Geometry::rad2deg(std::atan2(dy, dx)))) % 180;
            if (deg < 0)
                deg += 180;
            weight_per_degree[deg] += len;
        }
    };

    for (const LayerRegion *region : layer.regions())
        for (const ExtrusionEntity *entity : region->fills.flatten().entities) {
            if (auto *path = dynamic_cast<const ExtrusionPath *>(entity))
                account(*path);
            else if (auto *multi = dynamic_cast<const ExtrusionMultiPath *>(entity))
                for (const ExtrusionPath &p : multi->paths)
                    account(p);
            else if (auto *loop = dynamic_cast<const ExtrusionLoop *>(entity))
                for (const ExtrusionPath &p : loop->paths)
                    account(p);
        }

    if (weight_per_degree.empty())
        return -1;
    return std::max_element(weight_per_degree.begin(), weight_per_degree.end(),
                            [](const auto &a, const auto &b) { return a.second < b.second; })->first;
}

template<typename RolePred> static std::vector<int> angles_per_layer(const Print &print, RolePred role_wanted)
{
    std::vector<int> angles;
    for (const Layer *layer : print.objects().front()->layers())
        angles.push_back(dominant_fill_angle(*layer, role_wanted));
    return angles;
}

static bool solid_role(ExtrusionRole role) { return is_solid_infill(role) && role != erIroning; }
static bool sparse_role(ExtrusionRole role) { return role == erInternalInfill; }
static bool ironing_role(ExtrusionRole role) { return role == erIroning; }

TEST_CASE("Infill rotation template is unaffected by a raft", "[Fill][Regression]")
{
    // More angles than raft layers, so a raft cannot alias back to the same angle.
    const std::string template_string = GENERATE("+45", "0,25,50,75,100,125,150");
    const int raft_layers = GENERATE(1, 3);
    CAPTURE(template_string, raft_layers);

    auto angles_for = [&template_string](int rafts) {
        Print print;
        // 100% density makes every layer solid, so the template shows on all 100, not just shells.
        Slic3r::Test::init_and_process_print({Slic3r::Test::cube(20)}, print,
                                            {{"solid_infill_rotate_template", template_string},
                                             {"sparse_infill_density", "100%"},
                                             {"internal_solid_infill_pattern", "monotonic"},
                                             {"layer_height", 0.2},
                                             {"raft_layers", rafts}});
        return angles_per_layer(print, solid_role);
    };

    const std::vector<int> without_raft = angles_for(0);
    const std::vector<int> with_raft    = angles_for(raft_layers);

    REQUIRE(without_raft.size() == 100);
    REQUIRE(with_raft.size() == without_raft.size());
    REQUIRE(std::count(without_raft.begin(), without_raft.end(), -1) == 0);
    CHECK(with_raft == without_raft);
}

TEST_CASE("Sparse infill rotation template turns the infill layer by layer", "[Fill]")
{
    const std::vector<int> expected_cycle = {0, 25, 50, 75, 100, 125, 150};

    Print print;
    // No shells, so every layer is sparse infill rather than solid.
    Slic3r::Test::init_and_process_print({Slic3r::Test::cube(10)}, print,
                                        {{"sparse_infill_rotate_template", "0,25,50,75,100,125,150"},
                                         {"sparse_infill_density", "40%"},
                                         {"sparse_infill_pattern", "rectilinear"},
                                         {"top_shell_layers", 0},
                                         {"bottom_shell_layers", 0},
                                         {"layer_height", 0.2}});

    const std::vector<int> angles = angles_per_layer(print, sparse_role);
    REQUIRE(angles.size() == 50);
    REQUIRE(std::count(angles.begin(), angles.end(), -1) == 0);

    std::vector<int> expected;
    for (size_t i = 0; i < angles.size(); ++i)
        expected.push_back(expected_cycle[i % expected_cycle.size()]);
    CHECK(angles == expected);
}

TEST_CASE("Infill rotation template layer count modifier holds each angle for N layers", "[Fill]")
{
    Print print;
    // "+45#2" turns 45 degrees every 2 layers, so equal angles come in pairs.
    Slic3r::Test::init_and_process_print({Slic3r::Test::cube(10)}, print,
                                        {{"solid_infill_rotate_template", "+45#2"},
                                         {"sparse_infill_density", "100%"},
                                         {"internal_solid_infill_pattern", "monotonic"},
                                         {"layer_height", 0.2}});

    const std::vector<int> angles = angles_per_layer(print, solid_role);
    REQUIRE(angles.size() == 50);
    REQUIRE(std::count(angles.begin(), angles.end(), -1) == 0);

    std::vector<int> run_lengths;
    for (size_t i = 0; i < angles.size();) {
        size_t j = i;
        while (j < angles.size() && angles[j] == angles[i])
            ++j;
        run_lengths.push_back(int(j - i));
        i = j;
    }
    // The first and last runs can be clipped by the start and end of the object.
    REQUIRE(run_lengths.size() > 3);
    const std::vector<int> interior(run_lengths.begin() + 1, run_lengths.end() - 1);
    CHECK(std::count(interior.begin(), interior.end(), 2) == int(interior.size()));
}

TEST_CASE("Z anti-aliasing keeps the infill rotation template's step", "[Fill]")
{
    Print print;
    Slic3r::Test::init_and_process_print({Slic3r::Test::cube(10)}, print,
                                        {{"solid_infill_rotate_template", "+45"},
                                         {"sparse_infill_density", "100%"},
                                         {"internal_solid_infill_pattern", "monotonic"},
                                         {"zaa_enabled", 1},
                                         {"zaa_min_z", 0.05},
                                         {"layer_height", 0.2}});

    // Z contouring varies the layer heights, so the layer count is not 10mm / 0.2mm here.
    const std::vector<int> angles = angles_per_layer(print, solid_role);
    REQUIRE(angles.size() > 10);
    REQUIRE(std::count(angles.begin(), angles.end(), -1) == 0);

    // Z contouring may change when the template advances, but each step must still be 45 degrees.
    int steps = 0;
    for (size_t i = 1; i < angles.size(); ++i) {
        const int delta = ((angles[i] - angles[i - 1]) % 180 + 180) % 180;
        CAPTURE(i, angles[i - 1], angles[i]);
        // Split rather than "delta == 0 || delta == 45" so Catch2 can show the operands.
        REQUIRE(delta % 45 == 0);
        REQUIRE(delta <= 45);
        steps += delta == 45;
    }
    CHECK(steps > 0);
}

TEST_CASE("Ironing follows the solid infill rotation template", "[Fill]")
{
    Print print;
    Slic3r::Test::init_and_process_print({Slic3r::Test::cube(10)}, print,
                                        {{"solid_infill_rotate_template", "+45"},
                                         {"internal_solid_infill_pattern", "monotonic"},
                                         {"top_surface_pattern", "monotonic"},
                                         // Every solid surface, so the comparison covers every layer.
                                         {"ironing_type", "solid"},
                                         {"sparse_infill_density", "100%"},
                                         {"ironing_angle", 0},
                                         {"ironing_angle_fixed", 0},
                                         {"layer_height", 0.2}});

    const std::vector<int> ironing = angles_per_layer(print, ironing_role);
    const std::vector<int> solid   = angles_per_layer(print, solid_role);
    REQUIRE(ironing.size() == solid.size());

    // With no fixed angle and no offset, ironing runs along the template's angle for that layer.
    int compared = 0;
    for (size_t i = 0; i < ironing.size(); ++i)
        if (ironing[i] != -1 && solid[i] != -1) {
            CAPTURE(i, ironing[i], solid[i]);
            CHECK(ironing[i] == solid[i]);
            ++compared;
        }
    // Most of the object, not one lucky layer.
    REQUIRE(compared > int(ironing.size()) / 2);
}

TEST_CASE("Solid infill direction offsets every layer when no template is set", "[Fill]")
{
    auto angles_for = [](int direction) {
        Print print;
        Slic3r::Test::init_and_process_print({Slic3r::Test::cube(10)}, print,
                                            {{"solid_infill_direction", direction},
                                             {"sparse_infill_density", "100%"},
                                             {"internal_solid_infill_pattern", "monotonic"},
                                             {"layer_height", 0.2}});
        return angles_per_layer(print, solid_role);
    };

    const std::vector<int> at_0  = angles_for(0);
    const std::vector<int> at_30 = angles_for(30);
    REQUIRE(at_0.size() == at_30.size());
    REQUIRE(std::count(at_0.begin(), at_0.end(), -1) == 0);

    for (size_t i = 0; i < at_0.size(); ++i) {
        const int delta = ((at_30[i] - at_0[i]) % 180 + 180) % 180;
        CAPTURE(i, at_0[i], at_30[i]);
        CHECK(delta == 30);
    }
}

TEST_CASE("Scales surface pattern fills the requested line density", "[Fill]")
{
    // The visible parts of the scale discs tile the plane, so the concentric arcs one line
    // distance apart cover exactly one cell area per unit of length: filled area / path length
    // has to come out as the distance between neighbouring lines. 1.0 is a solid surface, the
    // lower values are the textured surfaces that top/bottom_surface_density allows.
    const std::string pattern = GENERATE("scales4", "scales6", "scales8");
    const double      density = GENERATE(0.5, 0.75, 1.0);

    std::unique_ptr<Slic3r::Fill> filler(Slic3r::Fill::new_from_type(pattern));
    filler->spacing = 0.5;
    filler->angle   = 0.f;

    FillParams fill_params;
    fill_params.density           = float(density);
    fill_params.dont_adjust       = true;
    fill_params.anchor_length_max = 0.f; // chain the arcs, do not add connectors along the boundary

    // Wide next to the largest scale radius here (arcs * spacing / density, at most 8 mm), so the
    // arcs cut off at the boundary stay a small share of the total.
    const Slic3r::ExPolygon square(Slic3r::Polygon::new_scale({Vec2d(0, 0), Vec2d(300, 0), Vec2d(300, 300), Vec2d(0, 300)}));
    Slic3r::Surface         surface(stInternal, square);
    const Slic3r::Polylines paths = filler->fill_surface(&surface, fill_params);

    double length = 0.;
    for (const Slic3r::Polyline &pl : paths)
        length += unscale<double>(pl.length());
    const double area = unscale<double>(unscale<double>(square.area()));

    CAPTURE(pattern, density);
    REQUIRE(length > 0.);
    CHECK_THAT(area / length, Catch::Matchers::WithinRel(filler->spacing / density, 0.05));
}

TEST_CASE("Scales surface pattern never exposes a closed ring", "[Fill]")
{
    // The two discs in front of a disc meet exactly at its centre, so every arc stays open. A
    // closed ring means the lattice proportions drifted off sqrt(3) * R by R / 2.
    const std::string pattern = GENERATE("scales4", "scales6", "scales8");

    std::unique_ptr<Slic3r::Fill> filler(Slic3r::Fill::new_from_type(pattern));
    filler->spacing = 0.5;
    filler->angle   = 0.f;

    FillParams fill_params;
    fill_params.density           = 1.f;
    fill_params.dont_adjust       = true;
    fill_params.anchor_length_max = 0.f;

    // One scale radius is arcs * 0.5 mm; a 200 mm square holds many whole scales.
    const Slic3r::ExPolygon square(Slic3r::Polygon::new_scale({Vec2d(0, 0), Vec2d(200, 0), Vec2d(200, 200), Vec2d(0, 200)}));
    Slic3r::Surface         surface(stInternal, square);
    const Slic3r::Polylines paths = filler->fill_surface(&surface, fill_params);

    CAPTURE(pattern);
    REQUIRE(! paths.empty());
    for (const Slic3r::Polyline &pl : paths)
        REQUIRE(! pl.is_closed());
}

TEST_CASE("Scales surface pattern with a forced fill order sweeps one arc at a time", "[Fill]")
{
    // Inward has to emit every scale's outermost arc across the whole surface before moving to the
    // next arc in, so the arc radii come out in non-increasing runs rather than interleaved.
    const auto [pattern, arcs] = GENERATE(table<std::string, int>({{"scales4", 4}, {"scales6", 6}, {"scales8", 8}}));

    std::unique_ptr<Slic3r::Fill> filler(Slic3r::Fill::new_from_type(pattern));
    filler->spacing = 0.5;
    filler->angle   = 0.f;

    FillParams fill_params;
    fill_params.density           = 1.f;
    fill_params.dont_adjust       = true;
    fill_params.anchor_length_max = 0.f;
    fill_params.fill_order        = SurfaceFillOrder::Inward;

    const Slic3r::ExPolygon square(Slic3r::Polygon::new_scale({Vec2d(0, 0), Vec2d(100, 0), Vec2d(100, 100), Vec2d(0, 100)}));
    Slic3r::Surface         surface(stTop, square);
    const Slic3r::Polylines paths = filler->fill_surface(&surface, fill_params);
    REQUIRE(paths.size() > size_t(arcs));

    // Radius of the circle through three points of an arc, in mm. Every point of an arc lies on its
    // ring, so this recovers the ring from any fragment long enough to be well conditioned.
    const double         distance = filler->spacing / fill_params.density;
    std::vector<int>     rings;
    int                  ccw = 0, cw = 0;
    for (const Slic3r::Polyline &pl : paths) {
        if (pl.points.size() < 3)
            continue;
        const Vec2d a = unscale(pl.points.front());
        const Vec2d b = unscale(pl.points[pl.points.size() / 2]);
        const Vec2d c = unscale(pl.points.back());
        const double area2 = (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
        if (std::abs(area2) < EPSILON)
            continue; // collinear fragment, no radius or direction to read
        const double radius = (b - a).norm() * (c - b).norm() * (c - a).norm() / (2. * std::abs(area2));
        const int    ring   = int(std::lround(radius / distance - 0.5));
        // Drop fragments too short to measure rather than letting their noise decide the assertion.
        if (ring < 0 || ring >= arcs || std::abs(radius - (double(ring) + 0.5) * distance) > 0.25 * distance)
            continue;
        ++(area2 > 0. ? ccw : cw);
        rings.push_back(ring);
    }

    CAPTURE(pattern, arcs, rings.size());
    // Every ring is represented, so the filter above cannot have quietly emptied out the check.
    std::vector<int> distinct = rings;
    std::sort(distinct.begin(), distinct.end());
    distinct.erase(std::unique(distinct.begin(), distinct.end()), distinct.end());
    REQUIRE(distinct.size() == size_t(arcs));
    // Inward must start on the outer rings and finish on the inner ones. Circumradius samples from
    // clipped arcs are noisy enough that a full descending sort is brittle, so compare medians of
    // the first and last thirds of the emission sequence.
    REQUIRE(rings.size() > 30);
    const size_t third = rings.size() / 3;
    std::vector<int> head(rings.begin(), rings.begin() + third);
    std::vector<int> tail(rings.end() - third, rings.end());
    std::nth_element(head.begin(), head.begin() + head.size() / 2, head.end());
    std::nth_element(tail.begin(), tail.begin() + tail.size() / 2, tail.end());
    CHECK(head[head.size() / 2] > tail[tail.size() / 2]);
    // Every scale is drawn the same way round, so the start-of-extrusion blob always lands on the
    // same end of the arc. One direction has to account for every measurable fragment.
    CAPTURE(ccw, cw);
    CHECK(std::min(ccw, cw) == 0);
}

static double min_path_distance(const Slic3r::Polylines &paths, const Slic3r::Point &p)
{
    double best = std::numeric_limits<double>::max();
    for (const Slic3r::Polyline &pl : paths) {
        if (pl.points.size() < 2)
            continue;
        for (size_t i = 1; i < pl.points.size(); ++ i)
            best = std::min(best, Slic3r::Line(pl.points[i - 1], pl.points[i]).distance_to(p));
    }
    return unscale<double>(best);
}

TEST_CASE("Hexagon surface pattern leaves a 5-line-width hole and does not double-stroke walls", "[Fill]")
{
    const double spacing = 0.5;
    const double density = 1.0;
    const double u       = spacing / density;
    const double R       = u * (2.5 + 1. / std::sqrt(3.));
    const double D       = R * std::sqrt(3.) + u;

    std::unique_ptr<Slic3r::Fill> filler(Slic3r::Fill::new_from_type("hexagon"));
    filler->spacing = spacing;
    filler->angle   = 0.f;
    // Match Concentric: clip_end needs a seam length. Production fills set this from seam_gap.
    filler->loop_clipping = scale_(0.05);

    FillParams fill_params;
    fill_params.density           = float(density);
    fill_params.dont_adjust       = true;
    fill_params.anchor_length_max = 0.f;

    const Slic3r::ExPolygon square(Slic3r::Polygon::new_scale(
        {Vec2d(0, 0), Vec2d(200, 0), Vec2d(200, 200), Vec2d(0, 200)}));
    Slic3r::Surface         surface(stInternal, square);
    const Slic3r::Polylines paths = filler->fill_surface(&surface, fill_params);
    REQUIRE(! paths.empty());

    for (const Slic3r::Polyline &pl : paths)
        REQUIRE(! pl.is_closed());

    // Neighbouring cells own facing walls one line-width apart, so midpoints of length-~R
    // sides must not coincide. Exact shared-centerline double-stroking would.
    {
        const double cell = 0.15 * u;
        std::map<std::pair<long, long>, int> mid_counts;
        size_t sides = 0;
        for (const Slic3r::Polyline &pl : paths) {
            for (size_t i = 1; i < pl.points.size(); ++ i) {
                const Vec2d a = unscale(pl.points[i - 1]);
                const Vec2d b = unscale(pl.points[i]);
                const double len = (b - a).norm();
                if (std::abs(len - R) > 0.12 * R)
                    continue;
                const Vec2d mid = 0.5 * (a + b);
                ++ mid_counts[{long(std::llround(mid.x() / cell)), long(std::llround(mid.y() / cell))}];
                ++ sides;
            }
        }
        REQUIRE(sides > 20);
        int overlaps = 0;
        for (const auto &kv : mid_counts)
            if (kv.second > 1)
                ++ overlaps;
        CAPTURE(sides, overlaps, mid_counts.size());
        CHECK(overlaps == 0);
    }

    // Interior cells are one opened hex loop each: six sides, so at least six vertices
    // after the seam clip.
    {
        size_t interior = 0;
        for (const Slic3r::Polyline &pl : paths) {
            const bool inside = std::all_of(pl.points.begin(), pl.points.end(), [](const Slic3r::Point &p) {
                const double x = unscale<double>(p.x()), y = unscale<double>(p.y());
                return x > 20. && x < 180. && y > 20. && y < 180.;
            });
            if (! inside)
                continue;
            ++ interior;
            REQUIRE(pl.points.size() >= 6);
        }
        REQUIRE(interior > 50);
    }

    const coord_t side_tol = scale_(0.08 * R);
    const coord_t apothem  = coord_t(std::round(scale_(R * std::sqrt(3.) / 2.)));
    bool          found    = false;
    for (const Slic3r::Polyline &pl : paths) {
        for (size_t i = 1; i < pl.points.size(); ++ i) {
            const Slic3r::Point &a = pl.points[i - 1];
            const Slic3r::Point &b = pl.points[i];
            const double         len = unscale<double>((b - a).cast<double>().norm());
            if (std::abs(len - R) > 0.08 * R)
                continue;
            // Original vertical sides become (nearly) axis-aligned after the π/2 infill rotation.
            if (std::abs(a.x() - b.x()) > side_tol && std::abs(a.y() - b.y()) > side_tol)
                continue;

            const Vec2d mid = 0.5 * (a.cast<double>() + b.cast<double>());
            Vec2d       n(double(a.y() - b.y()), double(b.x() - a.x()));
            const double nlen = n.norm();
            if (nlen < 1.)
                continue;
            n /= nlen;
            // Pick the inward normal (toward the cell centre).
            Slic3r::Point center{
                coord_t(std::round(mid.x() + n.x() * double(apothem))),
                coord_t(std::round(mid.y() + n.y() * double(apothem)))};
            if (min_path_distance(paths, center) < 2.4 * u) {
                n = -n;
                center = Slic3r::Point{
                    coord_t(std::round(mid.x() + n.x() * double(apothem))),
                    coord_t(std::round(mid.y() + n.y() * double(apothem)))};
            }
            const double cx = unscale<double>(center.x());
            const double cy = unscale<double>(center.y());
            if (cx < 40. || cx > 160. || cy < 40. || cy > 160.)
                continue;

            const double hole = min_path_distance(paths, center);
            REQUIRE(hole > 2.4 * u);

            Vec2d along(double(b.x() - a.x()), double(b.y() - a.y()));
            along /= along.norm();
            const double void_r = scale_(2.5 * u);
            const Slic3r::Point v0{
                coord_t(std::round(double(center.x()) + along.x() * void_r)),
                coord_t(std::round(double(center.y()) + along.y() * void_r))};
            const Slic3r::Point v1{
                coord_t(std::round(double(center.x()) - along.x() * void_r)),
                coord_t(std::round(double(center.y()) - along.y() * void_r))};
            CHECK_THAT(unscale<double>((v0 - v1).cast<double>().norm()), Catch::Matchers::WithinRel(5. * u, 0.02));
            const double d0 = min_path_distance(paths, v0);
            const double d1 = min_path_distance(paths, v1);
            REQUIRE(d0 > 0.2 * u);
            REQUIRE(d1 > 0.2 * u);
            found = true;
            break;
        }
        if (found)
            break;
    }
    REQUIRE(found);

    double length = 0.;
    for (const Slic3r::Polyline &pl : paths)
        length += unscale<double>(pl.length());
    const double area      = unscale<double>(unscale<double>(square.area()));
    const double cell_area = (std::sqrt(3.) / 2.) * D * D;
    const double expected  = area / cell_area * 6. * R;
    CHECK_THAT(length, Catch::Matchers::WithinRel(expected, 0.15));
}

namespace {

struct Stroke
{
    Polyline pl;
    double   spacing{0};
};

void collect_strokes(const ExtrusionEntity *e, std::vector<Stroke> &out)
{
    if (const auto *c = dynamic_cast<const ExtrusionEntityCollection *>(e)) {
        for (const ExtrusionEntity *ch : c->entities)
            collect_strokes(ch, out);
    } else if (const auto *p = dynamic_cast<const ExtrusionPath *>(e)) {
        Stroke s;
        s.pl      = p->as_polyline();
        s.spacing = Flow::rounded_rectangle_extrusion_spacing(p->width, p->height);
        if (s.pl.points.size() >= 2)
            out.emplace_back(std::move(s));
    } else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(e)) {
        for (const ExtrusionPath &p : loop->paths) {
            Stroke s;
            s.pl      = p.as_polyline();
            s.spacing = Flow::rounded_rectangle_extrusion_spacing(p.width, p.height);
            if (s.pl.points.size() >= 2)
                out.emplace_back(std::move(s));
        }
    }
}

struct WaveFill
{
    std::unique_ptr<Fill> filler;
    FillParams            params;
    Surface               surface;
    ExtrusionEntitiesPtr  out;

    WaveFill(double spacing, double density, const ExPolygon &poly)
        : surface(stTop, poly)
    {
        filler.reset(Fill::new_from_type("symmetricwave"));
        const float height = 0.2f;
        const float width  = float(spacing) + height * float(1. - 0.25 * PI);
        params.flow              = Flow(width, height, 0.4f);
        params.density           = float(density);
        params.dont_adjust       = true;
        params.anchor_length_max = 0.f;
        params.extrusion_role    = erTopSolidInfill;
        params.resolution        = 0.0125;
        filler->spacing          = params.flow.spacing();
        // _infill_direction adds π/2, so this keeps the wave along +X.
        filler->angle            = -float(PI / 2.0);
        filler->overlap          = 0;
        filler->fill_surface_extrusion(&surface, params, out);
    }

    ~WaveFill()
    {
        for (ExtrusionEntity *e : out)
            delete e;
    }

    std::vector<Stroke> strokes() const
    {
        std::vector<Stroke> s;
        for (const ExtrusionEntity *e : out)
            collect_strokes(e, s);
        return s;
    }
};

} // namespace

// Sample the y of every centreline crossing a vertical line, low to high.
static std::vector<double> crossings_at(const std::vector<Stroke> &strokes, double x_mm)
{
    const coord_t       x = scale_(x_mm);
    std::vector<double> ys;
    for (const Stroke &s : strokes) {
        for (size_t i = 1; i < s.pl.points.size(); ++i) {
            const Point &a = s.pl.points[i - 1];
            const Point &b = s.pl.points[i];
            if (double(a.x() - x) * double(b.x() - x) > 0)
                continue;
            const double den = double(b.x() - a.x());
            if (std::abs(den) < 1)
                continue;
            const double t = double(x - a.x()) / den;
            if (t < -1e-6 || t > 1. + 1e-6)
                continue;
            ys.push_back(unscale<double>(double(a.y()) + t * double(b.y() - a.y())));
        }
    }
    std::sort(ys.begin(), ys.end());
    return ys;
}

struct WaveCrossing
{
    double y{};
    double spacing{};
};

// Like crossings_at, but keeps the path spacing with each hit.
static std::vector<WaveCrossing> crossings_with_width(const std::vector<Stroke> &strokes, double x_mm)
{
    const coord_t             x = scale_(x_mm);
    std::vector<WaveCrossing> out;
    for (const Stroke &s : strokes) {
        for (size_t i = 1; i < s.pl.points.size(); ++i) {
            const Point &a = s.pl.points[i - 1];
            const Point &b = s.pl.points[i];
            if (double(a.x() - x) * double(b.x() - x) > 0)
                continue;
            const double den = double(b.x() - a.x());
            if (std::abs(den) < 1)
                continue;
            const double t = double(x - a.x()) / den;
            if (t < -1e-6 || t > 1. + 1e-6)
                continue;
            WaveCrossing c;
            c.y       = unscale<double>(double(a.y()) + t * double(b.y() - a.y()));
            c.spacing = s.spacing;
            out.push_back(c);
        }
    }
    std::sort(out.begin(), out.end(), [](const WaveCrossing &a, const WaveCrossing &b) { return a.y < b.y; });
    return out;
}

TEST_CASE("Symmetric Wave covers the plane at density 1", "[Fill]")
{
    // Paired even+odd widths sum to 2 H1 cos(theta), so their combined vertical
    // extent is 2 H1 and rows at pitch H = H1 cover the plane. Measured along
    // the path, that is the integral of w ds over the area.
    const double              spacing = 0.4;
    const ExPolygon           square(Polygon::new_scale({Vec2d(0, 0), Vec2d(80, 0), Vec2d(80, 80), Vec2d(0, 80)}));
    WaveFill                  fill(spacing, 1.0, square);
    const std::vector<Stroke> strokes = fill.strokes();
    REQUIRE(!strokes.empty());

    double w_ds = 0;
    for (const Stroke &s : strokes)
        w_ds += s.spacing * unscale<double>(s.pl.length());
    const double area = unscale<double>(unscale<double>(square.area()));
    REQUIRE(w_ds > 0);
    CHECK_THAT(w_ds / area, Catch::Matchers::WithinRel(1.0, 0.05));
}

TEST_CASE("Symmetric Wave rows share one phase at a constant pitch", "[Fill]")
{
    // Every row is the same curve translated in y. A half-period shift between
    // neighbours would pinch the channel shut at each peak and tile the surface
    // with closed diamonds instead, so the pitch has to be the same at a turn
    // as it is on a leg.
    const double              spacing = 0.4;
    const ExPolygon           square(Polygon::new_scale({Vec2d(0, 0), Vec2d(80, 0), Vec2d(80, 80), Vec2d(0, 80)}));
    WaveFill                  fill(spacing, 1.0, square);
    const std::vector<Stroke> strokes = fill.strokes();
    REQUIRE(!strokes.empty());

    std::vector<double> pitches;
    for (double x_mm = 16; x_mm <= 64; x_mm += 0.13) {
        const std::vector<double> ys = crossings_at(strokes, x_mm);
        for (size_t i = 1; i < ys.size(); ++i)
            pitches.push_back(ys[i] - ys[i - 1]);
    }
    REQUIRE(pitches.size() > 10);
    const auto mm = std::minmax_element(pitches.begin(), pitches.end());
    CHECK(*mm.second - *mm.first < 0.25 * spacing);
}

TEST_CASE("Symmetric Wave alternate rows swell on opposite legs", "[Fill]")
{
    // Even rows swell on the descending leg, odd rows on the ascending leg, so
    // neighbours trade width while their sum keeps the coverage identity. A
    // single shared apex-centred law, or a half-period nest, would either put
    // the swell back on the turns or close diamonds — both are regressions.
    const double              spacing = 0.4;
    const ExPolygon           square(Polygon::new_scale({Vec2d(0, 0), Vec2d(80, 0), Vec2d(80, 80), Vec2d(0, 80)}));
    WaveFill                  fill(spacing, 1.0, square);
    const std::vector<Stroke> strokes = fill.strokes();
    REQUIRE(!strokes.empty());

    // variable_width splits each row into constant-width fragments, so classify
    // fragments by the sign of their slope rather than trying to average a whole
    // row. Both legs must carry fat fragments and thin fragments.
    double w_min = 1e9, w_max = 0;
    double fat_asc = 0, fat_desc = 0, thin_asc = 0, thin_desc = 0;
    for (const Stroke &s : strokes) {
        w_min = std::min(w_min, s.spacing);
        w_max = std::max(w_max, s.spacing);
    }
    REQUIRE(w_min > 0);
    CHECK(w_max / w_min > 1.4);
    const double mid = 0.5 * (w_min + w_max);
    for (const Stroke &s : strokes) {
        for (size_t i = 1; i < s.pl.points.size(); ++i) {
            const double dx = unscale<double>(s.pl.points[i].x() - s.pl.points[i - 1].x());
            const double dy = unscale<double>(s.pl.points[i].y() - s.pl.points[i - 1].y());
            if (std::abs(dx) < 1e-6)
                continue;
            const double len = std::hypot(dx, dy);
            const bool   asc = dy / dx > 0;
            if (s.spacing > mid) {
                (asc ? fat_asc : fat_desc) += len;
            } else if (s.spacing < mid) {
                (asc ? thin_asc : thin_desc) += len;
            }
        }
    }
    CHECK(fat_asc > 1);
    CHECK(fat_desc > 1);
    CHECK(thin_asc > 1);
    CHECK(thin_desc > 1);
    // Roughly balanced — not a shared apex law dumping all fat onto the flats.
    CHECK_THAT(fat_asc, Catch::Matchers::WithinRel(fat_desc, 0.4));
    CHECK_THAT(thin_asc, Catch::Matchers::WithinRel(thin_desc, 0.4));

    // At a fixed x, neighbouring rows are complementary: when one is wide the
    // other is narrow, and the pair sum still tracks 2 H1 cos(theta).
    std::vector<double> pair_sums;
    int                 complementary = 0, total_pairs = 0;
    for (double x_mm = 16; x_mm <= 64; x_mm += 0.26) {
        const std::vector<WaveCrossing> xs = crossings_with_width(strokes, x_mm);
        for (size_t i = 1; i < xs.size(); ++i) {
            const double a = xs[i - 1].spacing;
            const double b = xs[i].spacing;
            pair_sums.push_back(a + b);
            ++total_pairs;
            if (std::abs(a - b) > 0.05 * spacing)
                ++complementary;
        }
    }
    REQUIRE(pair_sums.size() > 20);
    REQUIRE(total_pairs > 0);
    CHECK(complementary * 2 > total_pairs);
    const auto mm = std::minmax_element(pair_sums.begin(), pair_sums.end());
    // Pair sum follows cos(theta): higher at turns than at the steep legs, and
    // never collapses below a filled-channel floor.
    CHECK(*mm.second - *mm.first > 0.15 * spacing);
    CHECK(*mm.first > 1.5 * spacing);
}

TEST_CASE("Symmetric Wave clip keeps every vertex inside the surface", "[Fill]")
{
    const double    spacing = 0.4;
    const ExPolygon square(Polygon::new_scale({Vec2d(0, 0), Vec2d(8, 0), Vec2d(8, 8), Vec2d(0, 8)}));
    WaveFill        fill(spacing, 1.0, square);
    const std::vector<Stroke> strokes = fill.strokes();
    REQUIRE(!strokes.empty());

    Polylines paths;
    for (const Stroke &s : strokes)
        paths.push_back(s.pl);
    CHECK(diff_pl(paths, offset(square, float(SCALED_EPSILON * 10))).empty());
}

TEST_CASE("Symmetric Wave paths are monotonic bottom to top, left to right", "[Fill]")
{
    // Clipper's PolyTree does not preserve input order, and the G-code planner
    // would re-chain paths unless no_sort is set. Rows must stay bottom-to-top
    // and each fragment left-to-right so neighbouring beads start on the same side.
    const double              spacing = 0.4;
    const ExPolygon           square(Polygon::new_scale({Vec2d(0, 0), Vec2d(40, 0), Vec2d(40, 40), Vec2d(0, 40)}));
    WaveFill                  fill(spacing, 1.0, square);
    const std::vector<Stroke> strokes = fill.strokes();
    REQUIRE(strokes.size() > 4);

    for (const Stroke &s : strokes) {
        CHECK(s.pl.points.front().x() <= s.pl.points.back().x());
    }

    // Walk strokes in emission order; at a fixed x each row contributes at most
    // one crossing, so those y values must be non-decreasing.
    std::vector<double> ys_in_order;
    for (double x_mm : {10.0, 20.0, 30.0}) {
        ys_in_order.clear();
        const coord_t x = scale_(x_mm);
        for (const Stroke &s : strokes) {
            for (size_t i = 1; i < s.pl.points.size(); ++i) {
                const Point &a = s.pl.points[i - 1];
                const Point &b = s.pl.points[i];
                if (double(a.x() - x) * double(b.x() - x) > 0)
                    continue;
                const double den = double(b.x() - a.x());
                if (std::abs(den) < 1)
                    continue;
                const double t = double(x - a.x()) / den;
                if (t < -1e-6 || t > 1. + 1e-6)
                    continue;
                ys_in_order.push_back(unscale<double>(double(a.y()) + t * double(b.y() - a.y())));
                break; // one crossing per stroke is enough
            }
        }
        REQUIRE(ys_in_order.size() > 3);
        for (size_t i = 1; i < ys_in_order.size(); ++i)
            CHECK(ys_in_order[i] + 1e-6 >= ys_in_order[i - 1]);
    }
}

TEST_CASE("Symmetric Wave can be constructed without crashing", "[Fill]")
{
    std::unique_ptr<Fill> filler(Fill::new_from_type("symmetricwave"));
    REQUIRE(filler);
    CHECK_FALSE(filler->is_self_crossing());
    CHECK(filler->no_sort());
    filler->spacing = 0.4;
    filler->angle   = 0.f;
    FillParams params;
    params.density = 1.f;
    const ExPolygon square(Polygon::new_scale({Vec2d(0, 0), Vec2d(10, 0), Vec2d(10, 10), Vec2d(0, 10)}));
    Surface         surface(stInternal, square);
    CHECK(filler->fill_surface(&surface, params).empty());
}

static double stroke_travel_sum(const std::vector<Stroke> &strokes)
{
    double sum = 0.;
    for (size_t i = 1; i < strokes.size(); ++i)
        sum += (strokes[i].pl.first_point() - strokes[i - 1].pl.last_point()).cast<double>().norm();
    return sum;
}

// Reconstruct the old row-major emission order: bottom→top rows, L→R within a row.
static std::vector<Stroke> naive_row_major(std::vector<Stroke> strokes)
{
    std::stable_sort(strokes.begin(), strokes.end(), [](const Stroke &a, const Stroke &b) {
        const double ay = 0.5 * (unscale<double>(a.pl.first_point().y()) + unscale<double>(a.pl.last_point().y()));
        const double by = 0.5 * (unscale<double>(b.pl.first_point().y()) + unscale<double>(b.pl.last_point().y()));
        if (std::abs(ay - by) > 0.15)
            return ay < by;
        return a.pl.first_point().x() < b.pl.first_point().x();
    });
    return strokes;
}

static size_t long_hole_travels(const std::vector<Stroke> &strokes, const Polygon &hole, double min_mm)
{
    const double min_sc = scale_(min_mm);
    size_t       n      = 0;
    for (size_t i = 1; i < strokes.size(); ++i) {
        const Point a = strokes[i - 1].pl.last_point();
        const Point b = strokes[i].pl.first_point();
        if ((b - a).cast<double>().norm() < min_sc)
            continue;
        const Point mid = ((a.cast<double>() + b.cast<double>()) * 0.5).cast<coord_t>();
        if (hole.contains(mid))
            ++n;
    }
    return n;
}

TEST_CASE("Symmetric Wave region chain cuts travel across a hole", "[Fill]")
{
    // A central hole splits each row into left and right fragments. Row-major emission
    // crosses the hole once per row; the oriented region chain should finish one side
    // before the other and cross far fewer times.
    const double spacing = 0.4;
    Polygon      hole_poly = Polygon::new_scale({Vec2d(20, 20), Vec2d(40, 20), Vec2d(40, 40), Vec2d(20, 40)});
    if (hole_poly.is_counter_clockwise())
        hole_poly.reverse();
    const ExPolygon holed(Polygon::new_scale({Vec2d(0, 0), Vec2d(60, 0), Vec2d(60, 60), Vec2d(0, 60)}), hole_poly);

    WaveFill                  fill(spacing, 1.0, holed);
    const std::vector<Stroke> strokes = fill.strokes();
    REQUIRE(strokes.size() > 20);


    for (const Stroke &s : strokes)
        CHECK(s.pl.points.front().x() <= s.pl.points.back().x());

    // At a fixed x on the left channel, crossings stay bottom→top in emission order.
    for (double x_mm : {8.0, 52.0}) {
        std::vector<double> ys;
        const coord_t       x = scale_(x_mm);
        for (const Stroke &s : strokes) {
            for (size_t i = 1; i < s.pl.points.size(); ++i) {
                const Point &a = s.pl.points[i - 1];
                const Point &b = s.pl.points[i];
                if (double(a.x() - x) * double(b.x() - x) > 0)
                    continue;
                const double den = double(b.x() - a.x());
                if (std::abs(den) < 1)
                    continue;
                const double t = double(x - a.x()) / den;
                if (t < -1e-6 || t > 1. + 1e-6)
                    continue;
                ys.push_back(unscale<double>(double(a.y()) + t * double(b.y() - a.y())));
                break;
            }
        }
        REQUIRE(ys.size() > 3);
        for (size_t i = 1; i < ys.size(); ++i)
            CHECK(ys[i] + 1e-6 >= ys[i - 1]);
    }

    const std::vector<Stroke> naive = naive_row_major(strokes);
    const double              chained_travel = stroke_travel_sum(strokes);
    const double              naive_travel   = stroke_travel_sum(naive);
    CAPTURE(chained_travel, naive_travel);
    CHECK(chained_travel < naive_travel);

    const Polygon &hole = holed.holes.front();
    const size_t   chained_cross = long_hole_travels(strokes, hole, 8.);
    const size_t   naive_cross   = long_hole_travels(naive, hole, 8.);
    CAPTURE(chained_cross, naive_cross);
    CHECK(chained_cross < naive_cross);
    CHECK(chained_cross <= 2);
}

TEST_CASE("Scales forced fill order region chain cuts travel on a holed surface", "[Fill]")
{
    const auto [pattern, arcs] = GENERATE(table<std::string, int>({{"scales4", 4}, {"scales6", 6}}));
    const auto fill_order = GENERATE(SurfaceFillOrder::Inward, SurfaceFillOrder::Outward);

    std::unique_ptr<Slic3r::Fill> filler(Slic3r::Fill::new_from_type(pattern));
    filler->spacing = 0.5;
    filler->angle   = 0.f;

    FillParams fill_params;
    fill_params.density           = 1.f;
    fill_params.dont_adjust       = true;
    fill_params.anchor_length_max = 0.f;
    fill_params.fill_order        = fill_order;

    ExPolygon square(Polygon::new_scale({Vec2d(0, 0), Vec2d(80, 0), Vec2d(80, 80), Vec2d(0, 80)}));
    {
        Polygon hole = Polygon::new_scale({Vec2d(30, 30), Vec2d(50, 30), Vec2d(50, 50), Vec2d(30, 50)});
        if (hole.is_counter_clockwise())
            hole.reverse();
        square.holes.emplace_back(std::move(hole));
    }

    Slic3r::Surface         surface(stTop, square);
    const Slic3r::Polylines paths = filler->fill_surface(&surface, fill_params);
    REQUIRE(paths.size() > size_t(arcs));

    // Same sweep direction as the solid-square forced-order test.
    int ccw = 0, cw = 0;
    std::vector<int> rings;
    const double     distance = filler->spacing / fill_params.density;
    for (const Slic3r::Polyline &pl : paths) {
        if (pl.points.size() < 3)
            continue;
        const Vec2d a = unscale(pl.points.front());
        const Vec2d b = unscale(pl.points[pl.points.size() / 2]);
        const Vec2d c = unscale(pl.points.back());
        const double area2 = (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
        if (std::abs(area2) < EPSILON)
            continue;
        const double radius = (b - a).norm() * (c - b).norm() * (c - a).norm() / (2. * std::abs(area2));
        const int    ring   = int(std::lround(radius / distance - 0.5));
        if (ring < 0 || ring >= arcs || std::abs(radius - (double(ring) + 0.5) * distance) > 0.25 * distance)
            continue;
        ++(area2 > 0. ? ccw : cw);
        rings.push_back(ring);
    }
    CAPTURE(pattern, int(fill_order), ccw, cw);
    CHECK(std::min(ccw, cw) == 0);
    REQUIRE(rings.size() > 30);
    const size_t third = rings.size() / 3;
    std::vector<int> head(rings.begin(), rings.begin() + third);
    std::vector<int> tail(rings.end() - third, rings.end());
    std::nth_element(head.begin(), head.begin() + head.size() / 2, head.end());
    std::nth_element(tail.begin(), tail.begin() + tail.size() / 2, tail.end());
    if (fill_order == SurfaceFillOrder::Inward)
        CHECK(head[head.size() / 2] > tail[tail.size() / 2]);
    else
        CHECK(head[head.size() / 2] < tail[tail.size() / 2]);

    // Chained travel must beat a random-ish Clipper append order proxy: the same paths
    // sorted by start point only (ignores end→start), which is what a naïve lattice dump
    // after clip roughly looks like when Clipper reorders.
    double chained = 0.;
    for (size_t i = 1; i < paths.size(); ++i)
        chained += (paths[i].first_point() - paths[i - 1].last_point()).cast<double>().norm();

    Slic3r::Polylines by_start = paths;
    std::stable_sort(by_start.begin(), by_start.end(), [](const Polyline &a, const Polyline &b) {
        if (a.first_point().y() != b.first_point().y())
            return a.first_point().y() < b.first_point().y();
        return a.first_point().x() < b.first_point().x();
    });
    double sorted_travel = 0.;
    for (size_t i = 1; i < by_start.size(); ++i)
        sorted_travel += (by_start[i].first_point() - by_start[i - 1].last_point()).cast<double>().norm();

    CAPTURE(chained, sorted_travel);
    CHECK(chained <= sorted_travel * 1.05);
}

