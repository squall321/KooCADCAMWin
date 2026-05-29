// @lat: [[process/test-strategy#skill round-trip]]
//
// screen_print — silk-screen printing.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. CoarseMeshEmitsInfoFinding

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/screen_print.hpp"

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

}  // namespace

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillScreenPrint, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(150.0, 100.0, 3.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::screen_print::Input in;
    in.mesh_count  = 156;
    in.ink_color   = "PMS-186C";
    in.impressions = 2;

    auto out = skill::screen_print::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("screen_print"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillScreenPrint, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 3.0);

    skill::screen_print::Input badMesh;
    badMesh.mesh_count  = 0;
    badMesh.ink_color   = "black";
    badMesh.impressions = 1;
    {
        auto r = skill::screen_print::validate(*stock, badMesh);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::screen_print::apply(*stock, badMesh), skill::SkillError);

    skill::screen_print::Input emptyColor;
    emptyColor.mesh_count  = 156;
    emptyColor.ink_color   = "";
    emptyColor.impressions = 1;
    {
        auto r = skill::screen_print::validate(*stock, emptyColor);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::screen_print::apply(*stock, emptyColor), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillScreenPrint, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 4.0);

    skill::screen_print::Input in;
    in.mesh_count  = 230;
    in.ink_color   = "white";
    in.impressions = 3;

    auto out = skill::screen_print::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("screen_print"));
    EXPECT_EQ(out.signature.pattern["mesh_count"].get<int>(), 230);
    EXPECT_EQ(out.signature.pattern["ink_color"].get<std::string>(),
              std::string("white"));
    EXPECT_EQ(out.signature.pattern["impressions"].get<int>(), 3);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillScreenPrint, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 70.0, 3.0);

    skill::screen_print::Input in;
    in.mesh_count  = 156;
    in.ink_color   = "black";
    in.impressions = 1;

    auto out   = skill::screen_print::apply(*stock, in);
    auto cands = skill::screen_print::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["ink_color"].get<std::string>(),
              std::string("black"));
    EXPECT_EQ(cands[0].recovered_params["mesh_count"].get<int>(), 156);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::screen_print::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: coarse mesh emits info finding (still passes) ───────────
TEST(SkillScreenPrint, CoarseMeshEmitsInfoFinding)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 4.0);

    skill::screen_print::Input in;
    in.mesh_count  = 40;             // < 60 → coarse
    in.ink_color   = "red";
    in.impressions = 1;

    auto r = skill::screen_print::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-MESH-COARSE"));
    EXPECT_NO_THROW(skill::screen_print::apply(*stock, in));
}
