// @lat: [[process/test-strategy#skill round-trip]]
//
// chromatograph — analytical chromatography run record.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndKeyParams
//   4. RecognizeMetadataReplay
//   5. MethodSelectsCorrectInstrumentTool

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/chromatograph.hpp"

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

skill::chromatograph::Input goodInput()
{
    skill::chromatograph::Input in;
    in.method      = "HPLC";
    in.column_id   = "C18-150x4.6";
    in.runtime_min = 20.0;
    return in;
}

}  // namespace

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillChromatograph, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 50.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    auto out = skill::chromatograph::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("chromatograph"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillChromatograph, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 50.0);

    // Unknown method.
    auto badMethod = goodInput();
    badMethod.method = "TLC";
    {
        auto r = skill::chromatograph::validate(*stock, badMethod);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::chromatograph::apply(*stock, badMethod),
                 skill::SkillError);

    // Empty column id.
    auto noCol = goodInput();
    noCol.column_id = "";
    {
        auto r = skill::chromatograph::validate(*stock, noCol);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::chromatograph::apply(*stock, noCol),
                 skill::SkillError);

    // Zero runtime.
    auto noTime = goodInput();
    noTime.runtime_min = 0.0;
    {
        auto r = skill::chromatograph::validate(*stock, noTime);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-CHRO-TIME"));
    }
    EXPECT_THROW(skill::chromatograph::apply(*stock, noTime),
                 skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillChromatograph, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 50.0);

    auto in = goodInput();
    in.method      = "LC-MS";
    in.column_id   = "BEH-C18-100x2.1";
    in.runtime_min = 8.5;

    auto out = skill::chromatograph::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("chromatograph"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("chromatograph"));
    EXPECT_EQ(out.signature.pattern["method"].get<std::string>(),
              std::string("LC-MS"));
    EXPECT_EQ(out.signature.pattern["column_id"].get<std::string>(),
              std::string("BEH-C18-100x2.1"));
    EXPECT_NEAR(out.signature.pattern["runtime_min"].get<double>(),
                8.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillChromatograph, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 50.0);

    auto out   = skill::chromatograph::apply(*stock, goodInput());
    auto cands = skill::chromatograph::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["method"].get<std::string>(),
              std::string("HPLC"));
    EXPECT_NEAR(cands[0].recovered_params["runtime_min"].get<double>(),
                20.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::chromatograph::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: method choice drives instrument tool_type ───────────────
TEST(SkillChromatograph, MethodSelectsCorrectInstrumentTool)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 50.0);

    auto hplcIn = goodInput();
    hplcIn.method = "HPLC";
    auto hplcOut = skill::chromatograph::apply(*stock, hplcIn);
    EXPECT_EQ(hplcOut.signature.tooling.tool_type,
              std::string("hplc_system"));

    auto gcIn = goodInput();
    gcIn.method = "GC";
    auto gcOut = skill::chromatograph::apply(*stock, gcIn);
    EXPECT_EQ(gcOut.signature.tooling.tool_type,
              std::string("gc_system"));

    auto lcmsIn = goodInput();
    lcmsIn.method = "LC-MS";
    auto lcmsOut = skill::chromatograph::apply(*stock, lcmsIn);
    EXPECT_EQ(lcmsOut.signature.tooling.tool_type,
              std::string("lcms_system"));
}
