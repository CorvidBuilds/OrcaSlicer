#ifndef slic3r_SawtoothInterface_hpp_
#define slic3r_SawtoothInterface_hpp_

#include "../ExtrusionEntity.hpp"
#include "../Polyline.hpp"

namespace Slic3r {

class PrintObjectConfig;
class Flow;

// Parameters of the sawtooth post-process applied to support interface extrusions.
// All lengths are scaled (coord_t).
struct SawtoothParams {
    coord_t tooth_height = 0;   // Z rise at the tip. 0 disables the whole feature.
    coord_t spacing_min  = 0;   // distance between teeth, along the path
    coord_t spacing_max  = 0;
    coord_t ramp         = 0;   // run of each up/down ramp and of the flat tip
    coord_t clearance    = 0;   // segments shorter than this get no tooth
};

// Derive the parameters from the object config. tooth_height == 0 disables the feature.
SawtoothParams sawtooth_params(const PrintObjectConfig &object_config, const Flow &interface_flow);

// Rewrite `pl` into a Polyline3 with teeth. Pure geometry — unit testable.
// Returns true if at least one tooth was inserted.
bool add_sawtooth_teeth(const Polyline &pl, const SawtoothParams &p, Polyline3 &out);

// Walk entities appended at or after `first_entity` and apply teeth in place to
// erSupportMaterialInterface paths.
void apply_sawtooth_teeth(ExtrusionEntitiesPtr &entities, size_t first_entity, const SawtoothParams &p);

} // namespace Slic3r

#endif /* slic3r_SawtoothInterface_hpp_ */
