// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/keyway_din6885_parallel_hub.hpp"

#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <cmath>

using namespace koocadcam;
namespace pr = koocadcam::engine::prim;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

// Make a hub: 60 mm OD × 40 mm tall - bore_dia bore down Z axis.
std::shared_ptr<skill::Workpiece> makeHub(double od_mm, double bore_dia_mm,
                                          double height_mm)
{
    auto outer = skill::createCylindricalStock(od_mm, height_mm);
    const gp_Ax2 bore_ax(gp_Pnt(0, 0, -1.0), gp_Dir(0, 0, 1));
    const TopoDS_Shape bore = pr::cylinder(bore_ax, bore_dia_mm / 2.0,
                                            height_mm + 2.0);
    const TopoDS_Shape hubShape = pr::cut(outer->shape(), bore);
    return std::make_shared<skill::Workpiece>(hubShape, outer->material());
}
}

TEST(SkillKeywayDin6885ParallelHub, ApplyRemovesMaterial)
{
    auto hub = makeHub(60.0, 20.0, 40.0);   // 20 mm bore = 17–22 band

    skill::keyway_din6885_parallel_hub::Input in;
    in.bore_face_id      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.bore_axis         = gp_Dir(0, 0, 1);
    in.bore_center_x_mm  = 0.0;
    in.bore_center_y_mm  = 0.0;
    in.bore_dia_mm       = 20.0;
    in.key_position_z_mm = 20.0;
    in.key_length_mm     = 20.0;
    in.angle_rad         = 0.0;

    const double v0 = volumeOf(hub->shape());
    auto out = skill::keyway_din6885_parallel_hub::apply(*hub, in);
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_LT(v1, v0);
    const double removed = v0 - v1;
    EXPECT_GT(removed, 0.0);
}

TEST(SkillKeywayDin6885ParallelHub, ValidateRejectsTooShortLength)
{
    auto hub = makeHub(60.0, 20.0, 40.0);

    skill::keyway_din6885_parallel_hub::Input in;
    in.bore_face_id  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.bore_dia_mm   = 20.0;
    in.key_length_mm = 5.0;            // < 1.5 * 6 = 9 mm

    auto r = skill::keyway_din6885_parallel_hub::validate(*hub, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DIN6885-LEN") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillKeywayDin6885ParallelHub, SignatureRecordsDinSpec)
{
    auto hub = makeHub(60.0, 20.0, 40.0);

    skill::keyway_din6885_parallel_hub::Input in;
    in.bore_face_id      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.bore_dia_mm       = 20.0;
    in.key_position_z_mm = 20.0;
    in.key_length_mm     = 20.0;

    auto out = skill::keyway_din6885_parallel_hub::apply(*hub, in);

    EXPECT_EQ(out.signature.pattern["kind"],
              "keyway_din6885_parallel_hub");
    EXPECT_EQ(out.signature.pattern["is_compound"], true);
    EXPECT_EQ(out.signature.pattern["keyway_standard"],
              "DIN_6885_A_parallel_hub");
    EXPECT_NEAR(out.signature.pattern["key_width_mm"].get<double>(), 6.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["key_depth_mm"].get<double>(), 2.8, 1e-9);
    EXPECT_EQ  (out.signature.pattern["end_geometry"],            "square");
}

TEST(SkillKeywayDin6885ParallelHub, RecognizeMetadataReplay)
{
    auto hub = makeHub(60.0, 20.0, 40.0);

    skill::keyway_din6885_parallel_hub::Input in;
    in.bore_face_id      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.bore_dia_mm       = 20.0;
    in.key_position_z_mm = 20.0;
    in.key_length_mm     = 18.0;

    auto out = skill::keyway_din6885_parallel_hub::apply(*hub, in);
    auto cands = skill::keyway_din6885_parallel_hub::recognize(
        *out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["bore_dia_mm"].get<double>(),
                20.0, 1e-9);
}
