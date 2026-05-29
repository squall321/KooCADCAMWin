// @lat: [[process/test-strategy#skill round-trip]]
//
// lyophilize — freeze-drying metadata stamp.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. HighResidualMoistureEmitsInfo

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lyophilize.hpp"

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
TEST(SkillLyophilize, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::lyophilize::Input in;
    in.freeze_temp_c             = -60.0;
    in.primary_dry_pressure_mbar = 0.1;
    in.residual_moisture_pct     = 2.0;

    auto out = skill::lyophilize::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillLyophilize, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 3.0);

    // Positive freeze temperature.
    skill::lyophilize::Input warm;
    warm.freeze_temp_c             = 5.0;       // ≥ 0 °C invalid
    warm.primary_dry_pressure_mbar = 0.1;
    warm.residual_moisture_pct     = 2.0;
    {
        auto r = skill::lyophilize::validate(*stock, warm);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::lyophilize::apply(*stock, warm), skill::SkillError);

    // Zero pressure.
    skill::lyophilize::Input zeroP;
    zeroP.freeze_temp_c             = -40.0;
    zeroP.primary_dry_pressure_mbar = 0.0;
    zeroP.residual_moisture_pct     = 2.0;
    {
        auto r = skill::lyophilize::validate(*stock, zeroP);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-LYO-PRESSURE"));
    }

    // Moisture out of range.
    skill::lyophilize::Input badM;
    badM.freeze_temp_c             = -40.0;
    badM.primary_dry_pressure_mbar = 0.1;
    badM.residual_moisture_pct     = 25.0;      // > 20
    {
        auto r = skill::lyophilize::validate(*stock, badM);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-LYO-MOISTURE"));
    }
}

// ─── 3. Signature records kind + key params ────────────────────────────────
TEST(SkillLyophilize, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::lyophilize::Input in;
    in.freeze_temp_c             = -50.0;
    in.primary_dry_pressure_mbar = 0.2;
    in.residual_moisture_pct     = 1.5;

    auto out = skill::lyophilize::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("lyophilize"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("lyophilize"));
    EXPECT_NEAR(out.signature.pattern["freeze_temp_c"].get<double>(),
                -50.0, 1e-9);
    EXPECT_NEAR(
        out.signature.pattern["primary_dry_pressure_mbar"].get<double>(),
        0.2, 1e-9);
    EXPECT_NEAR(out.signature.pattern["residual_moisture_pct"].get<double>(),
                1.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ──────────────────────────────────────────
TEST(SkillLyophilize, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::lyophilize::Input in;
    in.freeze_temp_c             = -70.0;
    in.primary_dry_pressure_mbar = 0.05;
    in.residual_moisture_pct     = 1.0;

    auto out   = skill::lyophilize::apply(*stock, in);
    auto cands = skill::lyophilize::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["freeze_temp_c"].get<double>(),
                -70.0, 1e-9);
    EXPECT_NEAR(
        cands[0].recovered_params["primary_dry_pressure_mbar"].get<double>(),
        0.05, 1e-9);

    // Stripped of metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::lyophilize::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: high residual moisture emits info, still proceeds ────────
TEST(SkillLyophilize, HighResidualMoistureEmitsInfo)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 2.0);

    skill::lyophilize::Input in;
    in.freeze_temp_c             = -40.0;
    in.primary_dry_pressure_mbar = 0.1;
    in.residual_moisture_pct     = 8.0;      // > 5 → info

    auto r = skill::lyophilize::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-LYO-MOISTURE-HIGH"));

    auto out = skill::lyophilize::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("freeze_dryer"));
}
