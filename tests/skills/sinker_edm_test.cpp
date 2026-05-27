// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sinker_edm.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply: rect_pocket electrode sinks a rectangular cavity ───────────
TEST(SkillSinkerEdm, ApplyRectPocket)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 15.0);

    skill::sinker_edm::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.electrode_shape = "rect_pocket";
    in.dims = nlohmann::json{ { "length_mm", 20.0 }, { "width_mm", 12.0 } };
    in.depth_mm       = 4.0;

    auto out = skill::sinker_edm::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expected = 20.0 * 12.0 * 4.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("sinker_edm"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("sinker_edm"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("edm_electrode"));
}

// ── 2. Apply: cylindrical_cavity electrode sinks a round cavity ──────────
TEST(SkillSinkerEdm, ApplyCylindricalCavity)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 15.0);

    skill::sinker_edm::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 30.0;
    in.electrode_shape = "cylindrical_cavity";
    in.dims = nlohmann::json{ { "dia_mm", 14.0 } };
    in.depth_mm       = 5.0;

    auto out = skill::sinker_edm::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expected = M_PI * 7.0 * 7.0 * 5.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.05);

    EXPECT_EQ(out.signature.pattern["electrode_shape"].get<std::string>(),
              std::string("cylindrical_cavity"));
}

// ── 3. Validate: unknown electrode_shape rejected ────────────────────────
TEST(SkillSinkerEdm, ValidateRejectsUnknownShape)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 15.0);
    skill::sinker_edm::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0; in.position_y_mm = 30.0;
    in.electrode_shape = "freeform_blob";
    in.depth_mm = 3.0;

    auto r = skill::sinker_edm::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::sinker_edm::apply(*stock, in), skill::SkillError);
}

// ── 4. Validate: deep stroke emits warning ───────────────────────────────
TEST(SkillSinkerEdm, ValidateWarnsOnDeepStroke)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 80.0);
    skill::sinker_edm::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0; in.position_y_mm = 40.0;
    in.electrode_shape = "cylindrical_cavity";
    in.dims = nlohmann::json{ { "dia_mm", 10.0 } };
    in.depth_mm = 60.0;     // > 50 → warning

    auto r = skill::sinker_edm::validate(*stock, in);
    EXPECT_TRUE(r.passed);    // warnings don't fail
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-EDM-DEPTH" && f.severity == "warning") {
            found = true; break;
        }
    EXPECT_TRUE(found);
}

// ── 5. Recognize: zero match without metadata; full match with metadata ──
TEST(SkillSinkerEdm, RecognizeReplaysMetadata)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 15.0);
    skill::sinker_edm::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0; in.position_y_mm = 30.0;
    in.electrode_shape = "rect_pocket";
    in.dims = nlohmann::json{ { "length_mm", 20.0 }, { "width_mm", 12.0 } };
    in.depth_mm = 4.0;

    auto out = skill::sinker_edm::apply(*stock, in);
    auto cands = skill::sinker_edm::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_GT(cands[0].confidence, 0.9);
    EXPECT_EQ(cands[0].recovered_params["electrode_shape"].get<std::string>(),
              std::string("rect_pocket"));
}
