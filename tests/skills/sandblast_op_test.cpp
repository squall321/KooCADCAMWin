// @lat: [[process/test-strategy#skill round-trip]]
//
// sandblast_op skill — 5-case sweep covering identity-geometry synthesis,
// DFM pressure clamp, metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sandblast_op.hpp"

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

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillSandblastOp, ApplyDoesNotChangeGeometry)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore   = volumeOf(stock->shape());
    const int    faceBefore  = stock->faceCount();
    const int    edgeBefore  = stock->edgeCount();

    skill::sandblast_op::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.grit_size    = "120_grit";
    in.pressure_psi = 60.0;
    in.target_ra    = "ra_3.2";

    auto out = skill::sandblast_op::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("sandblast_op"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("sandblaster"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: pressure > 100 psi rejected ──────────────────────────────────
TEST(SkillSandblastOp, ValidateRejectsHighPressure)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::sandblast_op::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.grit_size    = "120_grit";
    in.pressure_psi = 150.0;       // > 100
    in.target_ra    = "ra_3.2";

    auto r = skill::sandblast_op::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SANDBLAST-PRESSURE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::sandblast_op::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature carries face_id + grit + Ra metadata ────────────────────
TEST(SkillSandblastOp, SignatureRecordsMetadata)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::sandblast_op::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.grit_size    = "220_grit";
    in.pressure_psi = 45.0;
    in.target_ra    = "ra_1.6";

    auto out = skill::sandblast_op::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("sandblast_op"));
    EXPECT_EQ(out.signature.pattern["grit_size"].get<std::string>(),
              std::string("220_grit"));
    EXPECT_EQ(out.signature.pattern["target_ra"].get<std::string>(),
              std::string("ra_1.6"));
    EXPECT_NEAR(out.signature.pattern["pressure_psi"].get<double>(), 45.0, 1e-6);
    EXPECT_TRUE(out.signature.pattern.contains("target_face_id"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSandblastOp, RecognizeFromMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::sandblast_op::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.grit_size    = "120_grit";
    in.pressure_psi = 60.0;
    in.target_ra    = "ra_3.2";

    auto out = skill::sandblast_op::apply(*stock, in);

    auto cands = skill::sandblast_op::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["grit_size"].get<std::string>(),
              std::string("120_grit"));
}

// ─── 5. Recognize on metadata-free workpiece returns EMPTY ────────────────
TEST(SkillSandblastOp, RecognizeEmptyWithoutMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::sandblast_op::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.grit_size    = "120_grit";
    in.pressure_psi = 60.0;
    in.target_ra    = "ra_3.2";
    auto out = skill::sandblast_op::apply(*stock, in);

    // Build a metadata-free Workpiece from the raw shape.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands = skill::sandblast_op::recognize(raw);
    EXPECT_TRUE(cands.empty())
        << "sandblast_op cannot be recognized geometrically";
}

// ─── 6. Multiple sandblast passes leave 1 signature per pass ──────────────
TEST(SkillSandblastOp, MultiplePassesStackSignatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::sandblast_op::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.grit_size    = "60_grit";
    in.pressure_psi = 80.0;
    in.target_ra    = "ra_6.3";

    auto w1 = skill::sandblast_op::apply(*stock, in);

    in.grit_size = "220_grit";
    in.target_ra = "ra_1.6";
    auto w2 = skill::sandblast_op::apply(*w1.workpiece, in);
    EXPECT_EQ(w2.workpiece->features().size(), 2u);

    auto cands = skill::sandblast_op::recognize(*w2.workpiece);
    EXPECT_EQ(cands.size(), 2u);
}
