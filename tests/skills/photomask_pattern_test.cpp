// @lat: [[process/test-strategy#skill round-trip]]
//
// photomask_pattern — lithography metadata-only skill.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/photomask_pattern.hpp"

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

// ── 1. Apply: geometry unchanged (clone, volume preserved) ───────────────
TEST(SkillPhotomaskPattern, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::photomask_pattern::Input in;
    in.mask_id              = "M-FIN-01";
    in.exposure_dose_mj_cm2 = 25.0;
    in.alignment_class      = "fine";

    auto out = skill::photomask_pattern::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. ValidateRejectsBadInput: empty mask, zero dose ────────────────────
TEST(SkillPhotomaskPattern, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");

    skill::photomask_pattern::Input emptyMask;
    emptyMask.mask_id              = "";
    emptyMask.exposure_dose_mj_cm2 = 25.0;
    auto r1 = skill::photomask_pattern::validate(*stock, emptyMask);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::photomask_pattern::apply(*stock, emptyMask),
                 skill::SkillError);

    skill::photomask_pattern::Input zeroDose;
    zeroDose.mask_id              = "M-OK";
    zeroDose.exposure_dose_mj_cm2 = 0.0;
    auto r2 = skill::photomask_pattern::validate(*stock, zeroDose);
    EXPECT_FALSE(r2.passed);

    skill::photomask_pattern::Input negDose;
    negDose.mask_id              = "M-OK";
    negDose.exposure_dose_mj_cm2 = -5.0;
    auto r3 = skill::photomask_pattern::validate(*stock, negDose);
    EXPECT_FALSE(r3.passed);
}

// ── 3. SignatureRecordsKind + key params ─────────────────────────────────
TEST(SkillPhotomaskPattern, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(200.0, 0.775, "silicon_wafer");

    skill::photomask_pattern::Input in;
    in.mask_id              = "M-CRIT-42";
    in.exposure_dose_mj_cm2 = 35.0;
    in.alignment_class      = "critical";

    auto out = skill::photomask_pattern::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("photomask_pattern"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("photomask_pattern"));
    EXPECT_EQ(out.signature.pattern["mask_id"].get<std::string>(),
              std::string("M-CRIT-42"));
    EXPECT_NEAR(out.signature.pattern["exposure_dose_mj_cm2"].get<double>(),
                35.0, 1e-12);
    EXPECT_EQ(out.signature.pattern["alignment_class"].get<std::string>(),
              std::string("critical"));
    EXPECT_TRUE(out.signature.pattern["is_metadata_only"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("stepper_scanner"));
}

// ── 4. RecognizeMetadataReplay ───────────────────────────────────────────
TEST(SkillPhotomaskPattern, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");

    skill::photomask_pattern::Input in;
    in.mask_id              = "M-RECG-01";
    in.exposure_dose_mj_cm2 = 50.0;
    in.alignment_class      = "fine";
    auto out = skill::photomask_pattern::apply(*stock, in);

    auto cands = skill::photomask_pattern::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["mask_id"].get<std::string>(),
              std::string("M-RECG-01"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::photomask_pattern::recognize(raw).empty());
}

// ── 5. SPECIFIC: unknown alignment_class emits info but does NOT block ──
TEST(SkillPhotomaskPattern, UnknownAlignmentEmitsInfoNotBlocking)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");

    skill::photomask_pattern::Input in;
    in.mask_id              = "M-ALIGN-X";
    in.exposure_dose_mj_cm2 = 30.0;
    in.alignment_class      = "ultra-fine";   // not in known set

    auto r = skill::photomask_pattern::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-LITHO-ALIGN"));
    EXPECT_NO_THROW(skill::photomask_pattern::apply(*stock, in));
}
