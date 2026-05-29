// @lat: [[process/test-strategy#skill round-trip]]
//
// qc_auto — 5-case sweep covering identity-geometry synthesis, VIN /
// inspection / pass-count DFM gates, signature with computed pass_rate,
// metadata replay, and pass-count > inspection-count rejection.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/qc_auto.hpp"

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

}  // namespace

// ── 1. Apply: geometry COMPLETELY unchanged ──────────────────────────────
TEST(SkillQcAuto, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::qc_auto::Input in;
    in.vin              = "KMHHN65F12U000123";    // 17 chars
    in.inspection_count = 100;
    in.pass_count       = 100;

    auto out = skill::qc_auto::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("qc_auto"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: empty VIN, bad counts rejected ───────────────────────────────
TEST(SkillQcAuto, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::qc_auto::Input bad;
    bad.vin              = "";                     // invalid
    bad.inspection_count = 50;
    bad.pass_count       = 50;
    EXPECT_FALSE(skill::qc_auto::validate(*stock, bad).passed);
    EXPECT_THROW(skill::qc_auto::apply(*stock, bad), skill::SkillError);

    bad.vin              = "KMHHN65F12U000123";
    bad.inspection_count = 0;                      // invalid
    EXPECT_FALSE(skill::qc_auto::validate(*stock, bad).passed);

    bad.inspection_count = 50;
    bad.pass_count       = -1;                     // invalid
    EXPECT_FALSE(skill::qc_auto::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + key params + pass_rate ───────────────────
TEST(SkillQcAuto, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::qc_auto::Input in;
    in.vin              = "KMHHN65F12U000456";
    in.inspection_count = 200;
    in.pass_count       = 190;

    auto out = skill::qc_auto::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("qc_auto"));
    EXPECT_EQ(out.signature.pattern["vin"].get<std::string>(),
              std::string("KMHHN65F12U000456"));
    EXPECT_EQ(out.signature.pattern["inspection_count"].get<int>(), 200);
    EXPECT_EQ(out.signature.pattern["pass_count"].get<int>(), 190);
    EXPECT_NEAR(out.signature.pattern["pass_rate"].get<double>(),
                190.0 / 200.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("qc_auto_station"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillQcAuto, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::qc_auto::Input in;
    in.vin              = "KMHHN65F12U000789";
    in.inspection_count = 150;
    in.pass_count       = 148;

    auto out = skill::qc_auto::apply(*stock, in);
    auto cands = skill::qc_auto::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["vin"].get<std::string>(),
              std::string("KMHHN65F12U000789"));
    EXPECT_EQ(cands[0].recovered_params["inspection_count"].get<int>(), 150);
    EXPECT_EQ(cands[0].recovered_params["pass_count"].get<int>(), 148);

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::qc_auto::recognize(raw).empty());
}

// ── 5. Skill-specific: pass_count > inspection_count → DFM-QC-COUNT error ─
TEST(SkillQcAuto, PassCountExceedsInspectionRejected)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::qc_auto::Input over;
    over.vin              = "KMHHN65F12U000ABC";
    over.inspection_count = 50;
    over.pass_count       = 60;                    // > inspection_count
    auto r = skill::qc_auto::validate(*stock, over);
    EXPECT_FALSE(r.passed);
    bool sawCount = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-QC-COUNT" && f.severity == "error") {
            sawCount = true; break;
        }
    }
    EXPECT_TRUE(sawCount);
    EXPECT_THROW(skill::qc_auto::apply(*stock, over), skill::SkillError);

    // Failures present (info-only) but operation proceeds.
    skill::qc_auto::Input withFails;
    withFails.vin              = "KMHHN65F12U000DEF";
    withFails.inspection_count = 100;
    withFails.pass_count       = 95;               // 5 failures
    auto r2 = skill::qc_auto::validate(*stock, withFails);
    EXPECT_TRUE(r2.passed);
    bool sawFails = false;
    for (const auto& f : r2.findings) {
        if (f.code == "DFM-QC-FAILS" && f.severity == "info") {
            sawFails = true; break;
        }
    }
    EXPECT_TRUE(sawFails);
    EXPECT_NO_THROW(skill::qc_auto::apply(*stock, withFails));
}
