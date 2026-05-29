// @lat: [[process/test-strategy#skill round-trip]]
//
// flexo_print — flexographic printing.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. UvInkSelectsUvPressTooling

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/flexo_print.hpp"

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
TEST(SkillFlexoPrint, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(800.0, 400.0, 0.2);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::flexo_print::Input in;
    in.anilox_lpi  = 800;
    in.dot_size_um = 40.0;
    in.ink_type    = "water";

    auto out = skill::flexo_print::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("flexo_print"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillFlexoPrint, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(400.0, 200.0, 0.2);

    skill::flexo_print::Input badLpi;
    badLpi.anilox_lpi  = -1;
    badLpi.dot_size_um = 40.0;
    badLpi.ink_type    = "water";
    {
        auto r = skill::flexo_print::validate(*stock, badLpi);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::flexo_print::apply(*stock, badLpi), skill::SkillError);

    skill::flexo_print::Input badInk;
    badInk.anilox_lpi  = 800;
    badInk.dot_size_um = 40.0;
    badInk.ink_type    = "magic";          // unknown
    {
        auto r = skill::flexo_print::validate(*stock, badInk);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::flexo_print::apply(*stock, badInk), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillFlexoPrint, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(800.0, 400.0, 0.2);

    skill::flexo_print::Input in;
    in.anilox_lpi  = 1200;
    in.dot_size_um = 25.0;
    in.ink_type    = "solvent";

    auto out = skill::flexo_print::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("flexo_print"));
    EXPECT_EQ(out.signature.pattern["anilox_lpi"].get<int>(), 1200);
    EXPECT_NEAR(out.signature.pattern["dot_size_um"].get<double>(),
                25.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["ink_type"].get<std::string>(),
              std::string("solvent"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillFlexoPrint, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(600.0, 300.0, 0.2);

    skill::flexo_print::Input in;
    in.anilox_lpi  = 800;
    in.dot_size_um = 40.0;
    in.ink_type    = "water";

    auto out   = skill::flexo_print::apply(*stock, in);
    auto cands = skill::flexo_print::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["anilox_lpi"].get<int>(), 800);
    EXPECT_EQ(cands[0].recovered_params["ink_type"].get<std::string>(),
              std::string("water"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::flexo_print::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: UV ink selects UV press tooling ─────────────────────────
TEST(SkillFlexoPrint, UvInkSelectsUvPressTooling)
{
    auto stock = skill::createCuboidStock(400.0, 200.0, 0.1);

    skill::flexo_print::Input in;
    in.anilox_lpi  = 1000;
    in.dot_size_um = 30.0;
    in.ink_type    = "uv";

    auto out = skill::flexo_print::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("flexo_uv_press"));
}
