// @lat: [[process/test-strategy#skill round-trip]]
//
// glue_laminate skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of out-of-range ply thickness, metadata-only recognition,
// and the specific assertion that total_thickness_mm = ply_count ×
// ply_thickness_mm and that PUR has shorter cure time than MUF.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/glue_laminate.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

skill::glue_laminate::Input goodInput()
{
    skill::glue_laminate::Input in;
    in.ply_count        = 6;
    in.ply_thickness_mm = 38.0;
    in.adhesive_type    = "MUF";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillGlueLaminate, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::glue_laminate::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("glue_laminate"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (ply too thick) ───────────────────────
TEST(SkillGlueLaminate, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.ply_thickness_mm = 75.0;                // > 50 mm EN 14080 cap
    auto r = skill::glue_laminate::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-GLULAM-PLY"));
    EXPECT_THROW(skill::glue_laminate::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.adhesive_type = "epoxy";               // not in {MUF, PRF, PUR}
    auto r2 = skill::glue_laminate::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-GLULAM-ADHESIVE"));

    auto in3 = goodInput();
    in3.ply_count = 1;                         // < 2
    auto r3 = skill::glue_laminate::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + plies + thickness + adhesive ────────────
TEST(SkillGlueLaminate, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.ply_count        = 8;
    in.ply_thickness_mm = 45.0;
    in.adhesive_type    = "PRF";
    auto out = skill::glue_laminate::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("glue_laminate"));
    EXPECT_EQ(out.signature.pattern["ply_count"].get<int>(), 8);
    EXPECT_NEAR(out.signature.pattern["ply_thickness_mm"].get<double>(),
                45.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["adhesive_type"].get<std::string>(),
              std::string("PRF"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillGlueLaminate, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::glue_laminate::apply(*stock, goodInput());

    auto cands = skill::glue_laminate::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["adhesive_type"].get<std::string>(),
              std::string("MUF"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::glue_laminate::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "glue_laminate cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: total_thickness_mm = ply_count × ply_thickness_mm; ──────
//      and PUR cures faster than MUF for the same lay-up
TEST(SkillGlueLaminate, TotalThicknessAndCureRanking)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.ply_count        = 10;
    in.ply_thickness_mm = 40.0;                // total = 400 mm
    auto out = skill::glue_laminate::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["total_thickness_mm"].get<double>(),
                400.0, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("hydraulic_press"));

    auto mufIn = goodInput();
    mufIn.adhesive_type = "MUF";
    auto mufOut = skill::glue_laminate::apply(*stock, mufIn);

    auto purIn = goodInput();
    purIn.adhesive_type = "PUR";
    auto purOut = skill::glue_laminate::apply(*stock, purIn);

    // Same ply lay-up → PUR (4 h) must cure faster than MUF (8 h).
    EXPECT_LT(purOut.signature.tooling.est_cycle_time_s,
              mufOut.signature.tooling.est_cycle_time_s);
}
