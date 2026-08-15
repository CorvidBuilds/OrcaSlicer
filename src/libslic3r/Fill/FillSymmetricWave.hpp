#ifndef slic3r_FillSymmetricWave_hpp_
#define slic3r_FillSymmetricWave_hpp_

#include "FillBase.hpp"

namespace Slic3r {

// Decorative top/bottom sine wallpaper. Rows share one phase and a constant
// pitch; alternate rows swell on opposite legs so paired coverage stays full
// while the visible expansion sits mid-leg rather than at the turns. Paths are
// emitted bottom-to-top, left-to-right (monotonic) and not re-sorted. Variable
// width is emitted through fill_surface_extrusion; the Polylines overload is a
// no-op.
class FillSymmetricWave : public Fill
{
public:
    bool is_self_crossing() override { return false; }
    bool no_sort() const override { return true; }
    void fill_surface_extrusion(const Surface *surface, const FillParams &params,
                                ExtrusionEntitiesPtr &out) override;

protected:
    Fill *clone() const override { return new FillSymmetricWave(*this); }
    float _layer_angle(size_t) const override { return 0.f; }
};

} // namespace Slic3r

#endif // slic3r_FillSymmetricWave_hpp_
