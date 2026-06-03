// @lat: [[process/test-strategy#skill round-trip]]
//
// surface_knit_to_shell — sew ≥ 3 faces of the workpiece into a Shell.
//
// Cases (5):
//   1. ApplyProducesNonNull — compound has a sub-shell sub-shape.
//   2. ValidateRejectsTooFewFaces (< 3).
//   3. ValidateRejectsBadTolerance (≤ 0).
//   4. SignatureRecordsKindAndFaceCount.
//   5. RecognizeMetadataReplay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/surface_knit_to_shell.hpp"

#include <TopExp_Explorer.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

// ─── 1. apply produces non-null compound containing a Shell ───────────────
TEST(SkillSurfaceKnitToShell, ApplyProducesNonNull)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 20.0);
    // A box has 6 faces (ids 0..5).  Sew the first 4 into a shell.
    skill::surface_knit_to_shell::Input in;
    in.face_ids     = { 0, 1, 2, 3 };
    in.tolerance_mm = 1e-3;

    auto out = skill::surface_knit_to_shell::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // A new (free) shell exists in the compound.
    int freeShells = 0;
    for (TopExp_Explorer e(out.workpiece->shape(), TopAbs_SHELL, TopAbs_SOLID);
         e.More(); e.Next()) {
        ++freeShells;
    }
    EXPECT_GE(freeShells, 1);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. DFM rejects fewer than 3 faces ────────────────────────────────────
TEST(SkillSurfaceKnitToShell, ValidateRejectsTooFewFaces)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    skill::surface_knit_to_shell::Input in;
    in.face_ids     = { 0, 1 };
    in.tolerance_mm = 1e-3;
    auto r = skill::surface_knit_to_shell::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FACE-COUNT") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
    EXPECT_THROW(skill::surface_knit_to_shell::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects tolerance ≤ 0 ─────────────────────────────────────────
TEST(SkillSurfaceKnitToShell, ValidateRejectsBadTolerance)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    skill::surface_knit_to_shell::Input in;
    in.face_ids     = { 0, 1, 2, 3 };
    in.tolerance_mm = -1.0;
    auto r = skill::surface_knit_to_shell::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TOLERANCE") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 4. signature records kind & face count ───────────────────────────────
TEST(SkillSurfaceKnitToShell, SignatureRecordsKindAndFaceCount)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);
    skill::surface_knit_to_shell::Input in;
    in.face_ids     = { 0, 1, 2 };
    in.tolerance_mm = 1e-3;
    auto out = skill::surface_knit_to_shell::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("surface_knit_to_shell"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["source_face_count"].get<int>(), 3);
}

// ─── 5. recognize replay ──────────────────────────────────────────────────
TEST(SkillSurfaceKnitToShell, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);
    skill::surface_knit_to_shell::Input in;
    in.face_ids     = { 0, 1, 2, 3 };
    in.tolerance_mm = 1e-3;
    auto out = skill::surface_knit_to_shell::apply(*stock, in);
    auto cands = skill::surface_knit_to_shell::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id,
              std::string("surface_knit_to_shell"));
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
}
