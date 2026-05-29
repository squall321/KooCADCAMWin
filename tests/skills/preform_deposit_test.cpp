// @lat: [[process/test-strategy#skill round-trip]]
//
// preform_deposit skill — MCVD/OVD preform deposition.  Covers:
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput  (core_dia ≥ clad_dia → DFM error)
//   3. SignatureRecordsKind+params (kind + core_dia + clad_dia + dopant)
//   4. RecognizeMetadataReplay
//   5. Specific assertion — clad-to-core ratio computed in pattern

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/preform_deposit.hpp"

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

// ─── 1. Apply: clones geometry unchanged ──────────────────────────────────
TEST(SkillPreformDeposit, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(125.0, 1000.0, "aluminum_6061");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::preform_deposit::Input in;
    in.method                   = "MCVD";
    in.core_dia_mm              = 8.0;
    in.clad_dia_mm              = 125.0;
    in.dopant_concentration_pct = 5.0;

    auto out = skill::preform_deposit::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("preform_deposit"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input (core ≥ clad) ──────────────────────────
TEST(SkillPreformDeposit, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(125.0, 500.0);

    skill::preform_deposit::Input in;
    in.method                   = "MCVD";
    in.core_dia_mm              = 130.0;     // larger than clad — invalid
    in.clad_dia_mm              = 125.0;
    in.dopant_concentration_pct = 5.0;

    auto r = skill::preform_deposit::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PREFORM-GEOMETRY"));
    EXPECT_THROW(skill::preform_deposit::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + params ───────────────────────────────────
TEST(SkillPreformDeposit, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(125.0, 500.0);

    skill::preform_deposit::Input in;
    in.method                   = "OVD";
    in.core_dia_mm              = 10.0;
    in.clad_dia_mm              = 120.0;
    in.dopant_concentration_pct = 7.5;
    in.dopant_species           = "GeO2";

    auto out = skill::preform_deposit::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("preform_deposit"));
    EXPECT_EQ(out.signature.pattern["method"].get<std::string>(),
              std::string("OVD"));
    EXPECT_NEAR(out.signature.pattern["core_dia_mm"].get<double>(), 10.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["clad_dia_mm"].get<double>(), 120.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["dopant_concentration_pct"].get<double>(),
                7.5, 1e-9);
    EXPECT_EQ(out.signature.pattern["dopant_species"].get<std::string>(),
              std::string("GeO2"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("mcvd_lathe"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPreformDeposit, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(125.0, 500.0);

    skill::preform_deposit::Input in;
    in.method                   = "MCVD";
    in.core_dia_mm              = 8.0;
    in.clad_dia_mm              = 125.0;
    in.dopant_concentration_pct = 5.0;
    auto out = skill::preform_deposit::apply(*stock, in);

    auto cands = skill::preform_deposit::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["method"].get<std::string>(),
              std::string("MCVD"));
    EXPECT_NEAR(cands[0].recovered_params["core_dia_mm"].get<double>(),
                8.0, 1e-9);

    // Same workpiece stripped of metadata → recognize is empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::preform_deposit::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "preform_deposit cannot be recognized geometrically";
}

// ─── 5. Specific assertion — clad/core ratio recorded in pattern ──────────
TEST(SkillPreformDeposit, PatternRecordsCladToCoreRatio)
{
    auto stock = skill::createCylindricalStock(125.0, 500.0);

    skill::preform_deposit::Input in;
    in.method                   = "MCVD";
    in.core_dia_mm              = 10.0;
    in.clad_dia_mm              = 125.0;     // ratio = 12.5 (well over 5)
    in.dopant_concentration_pct = 4.0;

    auto out = skill::preform_deposit::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["clad_to_core_ratio"].get<double>(),
                12.5, 1e-9);
}
