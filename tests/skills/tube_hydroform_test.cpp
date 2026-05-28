// @lat: [[process/test-strategy#skill round-trip]]
//
// tube_hydroform — internal-pressure expansion to die profile; metadata-only.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tube_hydroform.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. ApplyHandlesInput — metadata-only ────────────────────────────────
TEST(SkillTubeHydroform, ApplyHandlesInput)
{
    auto stock = skill::createCylindricalStock(40.0, 100.0);
    const double volBefore = volumeOf(stock->shape());

    skill::tube_hydroform::Input in;
    in.target_profile_polyline = {
        { 0.0,  20.0 },
        { 30.0, 25.0 },
        { 60.0, 25.0 },
        { 100.0,20.0 },
    };
    in.pressure_mpa = 250.0;
    auto out = skill::tube_hydroform::apply(*stock, in);

    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.signature.skill_id, std::string("tube_hydroform"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. ValidateRejectsBadInput — polyline with < 2 vertices ─────────────
TEST(SkillTubeHydroform, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(40.0, 100.0);

    skill::tube_hydroform::Input in;
    in.target_profile_polyline = { { 0.0, 20.0 } };   // only 1 vertex
    in.pressure_mpa = 200.0;

    auto r = skill::tube_hydroform::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::tube_hydroform::apply(*stock, in), skill::SkillError);
}

// ─── 3. SignatureRecordsKind + key params (incl. polyline) ───────────────
TEST(SkillTubeHydroform, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(40.0, 100.0);

    skill::tube_hydroform::Input in;
    in.target_profile_polyline = {
        { 10.0, 22.0 },
        { 50.0, 26.0 },
        { 90.0, 22.0 },
    };
    in.pressure_mpa = 180.0;
    auto out = skill::tube_hydroform::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("tube_hydroform"));
    EXPECT_NEAR(out.signature.pattern["pressure_mpa"].get<double>(),
                180.0, 1e-6);
    ASSERT_TRUE(out.signature.pattern["target_profile_polyline"].is_array());
    EXPECT_EQ(out.signature.pattern["target_profile_polyline"].size(), 3u);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("hydroform_die"));
}

// ─── 4. RecognizeMetadataReplay ──────────────────────────────────────────
TEST(SkillTubeHydroform, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 100.0);

    skill::tube_hydroform::Input in;
    in.target_profile_polyline = {
        { 0.0,  20.0 },
        { 50.0, 25.0 },
        { 100.0,20.0 },
    };
    in.pressure_mpa = 220.0;
    auto out = skill::tube_hydroform::apply(*stock, in);

    auto cands = skill::tube_hydroform::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["pressure_mpa"].get<double>(),
                220.0, 1e-6);

    // Stripped of metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::tube_hydroform::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: non-monotone Z in polyline → DFM-INPUT error ───────────
TEST(SkillTubeHydroform, ValidateRejectsNonMonotonePolyline)
{
    auto stock = skill::createCylindricalStock(40.0, 100.0);

    skill::tube_hydroform::Input in;
    in.target_profile_polyline = {
        { 10.0, 20.0 },
        { 50.0, 25.0 },
        { 30.0, 25.0 },   // Z decreases — not monotone
    };
    in.pressure_mpa = 200.0;

    auto r = skill::tube_hydroform::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);
}
