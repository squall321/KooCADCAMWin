// @lat: [[process/test-strategy#skill round-trip]]
//
// glued_joint — metadata-only adhesive bond.  Produces NO geometric change.
//
// Cases (6):
//   1. Apply does not alter shape topology or volume.
//   2. Signature carries adhesive_type, bead_width, cure_time, bond_area.
//   3. DFM rejects empty adhesive_type.
//   4. DFM rejects bead_width outside [0.1, 5.0].
//   5. Recognize replays only from history (no geometric fallback).
//   6. DFM rejects a tiny bond face area (< 25 mm²).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/glued_joint.hpp"

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

// ─── 1. Apply leaves shape topology unchanged ───────────────────────────
TEST(SkillGluedJoint, ApplyDoesNotChangeShape)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::glued_joint::Input in;
    in.bond_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.partner_part       = "lid";
    in.partner_face_name  = "lid_bottom";
    in.adhesive_type      = "epoxy";
    in.bead_width_mm      = 1.5;
    in.cure_time_min      = 60.0;
    in.shear_strength_MPa = 18.0;

    auto out = skill::glued_joint::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_EQ(out.workpiece->faceCount(),   stock->faceCount());
    EXPECT_EQ(out.workpiece->edgeCount(),   stock->edgeCount());
    EXPECT_EQ(out.workpiece->vertexCount(), stock->vertexCount());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()),
                volumeOf(stock->shape()), 1e-6);
}

// ─── 2. Signature carries adhesive metadata ──────────────────────────────
TEST(SkillGluedJoint, SignatureCarriesMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::glued_joint::Input in;
    in.bond_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.partner_part       = "cap";
    in.adhesive_type      = "uv_cure";
    in.bead_width_mm      = 0.8;
    in.cure_time_min      = 5.0;
    in.shear_strength_MPa = 20.0;

    auto out = skill::glued_joint::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("glued_joint"));
    EXPECT_EQ(out.signature.params["adhesive_type"].get<std::string>(), std::string("uv_cure"));
    EXPECT_NEAR(out.signature.params["bead_width_mm"].get<double>(),   0.8, 1e-9);
    EXPECT_NEAR(out.signature.params["cure_time_min"].get<double>(),   5.0, 1e-9);
    EXPECT_EQ(out.signature.params["partner_part"].get<std::string>(), std::string("cap"));
    // Bond area on a 50 × 50 top face = 2500 mm²
    EXPECT_NEAR(out.signature.params["bond_area_mm2"].get<double>(), 2500.0, 1.0);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 3. DFM rejects empty adhesive_type ──────────────────────────────────
TEST(SkillGluedJoint, ValidateRejectsEmptyAdhesive)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::glued_joint::Input in;
    in.bond_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.partner_part       = "lid";
    in.adhesive_type      = "";          // invalid
    in.bead_width_mm      = 1.0;
    in.cure_time_min      = 30.0;

    auto r = skill::glued_joint::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GLUE-ADHESIVE") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
    EXPECT_THROW(skill::glued_joint::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects out-of-range bead width ──────────────────────────────
TEST(SkillGluedJoint, ValidateRejectsBeadWidth)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::glued_joint::Input in;
    in.bond_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.adhesive_type   = "epoxy";
    in.bead_width_mm   = 8.0;            // > 5.0
    in.cure_time_min   = 60.0;

    auto r = skill::glued_joint::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GLUE-WIDTH") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 5. Recognize replays only from history (no geometric fallback) ──────
TEST(SkillGluedJoint, RecognizeOnlyFromHistory)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Without any features applied, recognize() must return empty —
    // glued_joint cannot be recovered from B-rep alone.
    auto empty = skill::glued_joint::recognize(*stock);
    EXPECT_TRUE(empty.empty());

    // After apply, it must replay.
    skill::glued_joint::Input in;
    in.bond_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.partner_part    = "lid";
    in.adhesive_type   = "epoxy";
    in.bead_width_mm   = 1.0;
    in.cure_time_min   = 45.0;

    auto out = skill::glued_joint::apply(*stock, in);
    auto cands = skill::glued_joint::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_NEAR(c.confidence, 1.0, 1e-9);
    EXPECT_EQ(c.recovered_params["partner_part"].get<std::string>(), std::string("lid"));
    EXPECT_EQ(c.recovered_params["adhesive_type"].get<std::string>(), std::string("epoxy"));
}

// ─── 6. DFM flags a tiny bond face area ──────────────────────────────────
TEST(SkillGluedJoint, ValidateRejectsTinyBondArea)
{
    // 4 × 4 × 4 cube → top face area = 16 mm², below the 25 mm² minimum.
    auto stock = skill::createCuboidStock(4.0, 4.0, 4.0);

    skill::glued_joint::Input in;
    in.bond_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.adhesive_type   = "epoxy";
    in.bead_width_mm   = 0.5;
    in.cure_time_min   = 30.0;

    auto r = skill::glued_joint::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GLUE-AREA") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}
