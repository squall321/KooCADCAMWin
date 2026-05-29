// @lat: [[process/test-strategy#skill round-trip]]
//
// digital_print — short-run digital printing.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. UvInkSelectsUvInkjetPressTooling

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/digital_print.hpp"

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
TEST(SkillDigitalPrint, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(297.0, 210.0, 0.1);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::digital_print::Input in;
    in.dpi      = 1200;
    in.ink_type = "toner";
    in.sheets   = 500;

    auto out = skill::digital_print::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("digital_print"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillDigitalPrint, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(297.0, 210.0, 0.1);

    skill::digital_print::Input badDpi;
    badDpi.dpi      = 0;
    badDpi.ink_type = "toner";
    badDpi.sheets   = 100;
    {
        auto r = skill::digital_print::validate(*stock, badDpi);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::digital_print::apply(*stock, badDpi), skill::SkillError);

    skill::digital_print::Input badInk;
    badInk.dpi      = 1200;
    badInk.ink_type = "plasma";       // unknown
    badInk.sheets   = 100;
    {
        auto r = skill::digital_print::validate(*stock, badInk);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::digital_print::apply(*stock, badInk), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillDigitalPrint, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(297.0, 210.0, 0.1);

    skill::digital_print::Input in;
    in.dpi      = 1800;
    in.ink_type = "latex";
    in.sheets   = 250;

    auto out = skill::digital_print::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("digital_print"));
    EXPECT_EQ(out.signature.pattern["dpi"].get<int>(), 1800);
    EXPECT_EQ(out.signature.pattern["ink_type"].get<std::string>(),
              std::string("latex"));
    EXPECT_EQ(out.signature.pattern["sheets"].get<int>(), 250);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillDigitalPrint, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(297.0, 210.0, 0.1);

    skill::digital_print::Input in;
    in.dpi      = 1200;
    in.ink_type = "aqueous";
    in.sheets   = 100;

    auto out   = skill::digital_print::apply(*stock, in);
    auto cands = skill::digital_print::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["dpi"].get<int>(), 1200);
    EXPECT_EQ(cands[0].recovered_params["ink_type"].get<std::string>(),
              std::string("aqueous"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::digital_print::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: UV ink selects UV-inkjet press tooling ──────────────────
TEST(SkillDigitalPrint, UvInkSelectsUvInkjetPressTooling)
{
    auto stock = skill::createCuboidStock(297.0, 210.0, 0.1);

    skill::digital_print::Input in;
    in.dpi      = 1200;
    in.ink_type = "uv";
    in.sheets   = 50;

    auto out = skill::digital_print::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("uv_inkjet_press"));
}
