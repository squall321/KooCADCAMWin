// @lat: [[process/test-strategy#skill round-trip]]
//
// tool_recoat skill — 5-case sweep covering identity-geometry synthesis,
// DFM error on empty tool_id / empty new_coating, signature metadata
// round-trip, recognition by metadata replay, and the specific
// same-coat / unknown-strip-method assertions.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tool_recoat.hpp"

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

skill::tool_recoat::Input goodInput()
{
    skill::tool_recoat::Input in;
    in.tool_id          = "EM-6MM-4F-001";
    in.old_coating      = "TiN";
    in.new_coating      = "TiAlN";
    in.stripping_method = "chemical";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ─────────────────────────────────
TEST(SkillToolRecoat, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::tool_recoat::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("tool_recoat"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (empty tool_id, empty new_coating) ────
TEST(SkillToolRecoat, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");

    auto in = goodInput();
    in.tool_id = "";                                  // empty
    auto r = skill::tool_recoat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::tool_recoat::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.new_coating = "";                             // must not be empty
    auto r2 = skill::tool_recoat::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + coating transition + strip method ───────
TEST(SkillToolRecoat, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");

    auto in = goodInput();
    in.tool_id          = "IN-CNMG120408-002";
    in.old_coating      = "AlTiN";
    in.new_coating      = "AlCrN";
    in.stripping_method = "electrochemical";

    auto out = skill::tool_recoat::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("tool_recoat"));
    EXPECT_EQ(out.signature.pattern["tool_id"].get<std::string>(),
              std::string("IN-CNMG120408-002"));
    EXPECT_EQ(out.signature.pattern["old_coating"].get<std::string>(),
              std::string("AlTiN"));
    EXPECT_EQ(out.signature.pattern["new_coating"].get<std::string>(),
              std::string("AlCrN"));
    EXPECT_EQ(out.signature.pattern["stripping_method"].get<std::string>(),
              std::string("electrochemical"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillToolRecoat, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");
    auto out = skill::tool_recoat::apply(*stock, goodInput());

    auto cands = skill::tool_recoat::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["new_coating"].get<std::string>(),
              std::string("TiAlN"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::tool_recoat::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "tool_recoat cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: same-coat warning + unknown strip method info ──────────
TEST(SkillToolRecoat, SameCoatAndUnknownStripEmitInfoButPasses)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");

    auto inSame = goodInput();
    inSame.old_coating = "TiAlN";
    inSame.new_coating = "TiAlN";                     // identical
    auto r = skill::tool_recoat::validate(*stock, inSame);
    EXPECT_TRUE(r.passed) << "info must not block apply";
    EXPECT_TRUE(hasFinding(r, "DFM-RC-SAMECOAT"));
    EXPECT_NO_THROW(skill::tool_recoat::apply(*stock, inSame));

    auto inStrip = goodInput();
    inStrip.stripping_method = "laser_ablation";      // unknown
    auto r2 = skill::tool_recoat::validate(*stock, inStrip);
    EXPECT_TRUE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-RC-STRIP"));
}
