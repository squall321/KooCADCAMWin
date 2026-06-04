// @lat: [[process/test-strategy#compound piston_oil_gallery_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/piston_oil_gallery_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::piston_oil_gallery_compound::Input goodInput()
{
    skill::piston_oil_gallery_compound::Input in;
    in.face_id            = 0;     // cylinder face
    in.piston_axis_dir    = gp_Dir(0.0, 0.0, 1.0);
    in.piston_axis_origin = gp_Pnt(0.0, 0.0, 2.0);
    in.gallery_OD_mm      = 60.0;
    in.gallery_ID_mm      = 40.0;
    in.gallery_height_mm  = 6.0;
    in.inlet_hole_dia_mm  = 3.0;
    in.drain_hole_dia_mm  = 2.5;
    return in;
}

}  // namespace

// ─── 1. apply removes volume (annular + 2 holes) ─────────────────────────
TEST(SkillPistonOilGallery, ApplyCutsAnnularAndHoles)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0);
    auto in    = goodInput();
    ASSERT_GE(in.face_id, 0);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::piston_oil_gallery_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
}

// ─── 2. validate rejects OD <= ID ────────────────────────────────────────
TEST(SkillPistonOilGallery, ValidateRejectsODNotGreaterThanID)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0);
    auto in    = goodInput();
    in.gallery_OD_mm = in.gallery_ID_mm;

    auto r = skill::piston_oil_gallery_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GALLERY-RING") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::piston_oil_gallery_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. signature records engine_feature_type ────────────────────────────
TEST(SkillPistonOilGallery, SignatureRecordsEngineFeature)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::piston_oil_gallery_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("piston_oil_gallery_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("engine_feature_type")
              .get<std::string>(),
              std::string("piston_oil_gallery"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
    EXPECT_NEAR(out.signature.pattern.at("wall_thickness_mm").get<double>(),
                10.0, 1e-6);
}

// ─── 4. recognize via metadata replay ────────────────────────────────────
TEST(SkillPistonOilGallery, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::piston_oil_gallery_compound::apply(*stock, in);

    auto cands = skill::piston_oil_gallery_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("gallery_OD_mm").get<double>(), 60.0);
}

// ─── 5. SPECIFIC: derived volume removed > 0 ─────────────────────────────
TEST(SkillPistonOilGallery, DerivedVolumeRemovedRecorded)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0);
    auto in    = goodInput();
    auto out   = skill::piston_oil_gallery_compound::apply(*stock, in);

    const double removed =
        out.signature.pattern.at("derived_volume_removed_mm3").get<double>();
    EXPECT_GT(removed, 0.0);
}
