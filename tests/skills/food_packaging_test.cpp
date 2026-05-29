// @lat: [[process/test-strategy#skill round-trip]]
//
// food_packaging — vacuum / MAP food packaging metadata stamp.
// Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. VacuumWithHighO2EmitsInfo

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/food_packaging.hpp"

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
TEST(SkillFoodPackaging, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::food_packaging::Input in;
    in.package_type = "vacuum";
    in.gas_mix      = "vacuum";
    in.oxygen_pct   = 0.5;

    auto out = skill::food_packaging::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillFoodPackaging, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 3.0);

    // Unknown package type.
    skill::food_packaging::Input badPkg;
    badPkg.package_type = "shrink_band";
    badPkg.gas_mix      = "n2";
    badPkg.oxygen_pct   = 1.0;
    {
        auto r = skill::food_packaging::validate(*stock, badPkg);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::food_packaging::apply(*stock, badPkg),
                 skill::SkillError);

    // Empty gas mix.
    skill::food_packaging::Input emptyGas;
    emptyGas.package_type = "map_tray";
    emptyGas.gas_mix      = "";
    emptyGas.oxygen_pct   = 1.0;
    {
        auto r = skill::food_packaging::validate(*stock, emptyGas);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-PKG-GAS"));
    }

    // Oxygen pct out of range.
    skill::food_packaging::Input badO2;
    badO2.package_type = "map_pouch";
    badO2.gas_mix      = "co2_n2_30_70";
    badO2.oxygen_pct   = 50.0;        // > 21
    {
        auto r = skill::food_packaging::validate(*stock, badO2);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-PKG-OXYGEN"));
    }
}

// ─── 3. Signature records kind + key params ────────────────────────────────
TEST(SkillFoodPackaging, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::food_packaging::Input in;
    in.package_type = "map_tray";
    in.gas_mix      = "co2_n2_o2_30_65_5";
    in.oxygen_pct   = 5.0;

    auto out = skill::food_packaging::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("food_packaging"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("food_packaging"));
    EXPECT_EQ(out.signature.pattern["package_type"].get<std::string>(),
              std::string("map_tray"));
    EXPECT_EQ(out.signature.pattern["gas_mix"].get<std::string>(),
              std::string("co2_n2_o2_30_65_5"));
    EXPECT_NEAR(out.signature.pattern["oxygen_pct"].get<double>(),
                5.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ──────────────────────────────────────────
TEST(SkillFoodPackaging, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::food_packaging::Input in;
    in.package_type = "thermoform";
    in.gas_mix      = "n2";
    in.oxygen_pct   = 0.3;

    auto out   = skill::food_packaging::apply(*stock, in);
    auto cands = skill::food_packaging::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["package_type"].get<std::string>(),
              std::string("thermoform"));
    EXPECT_NEAR(cands[0].recovered_params["oxygen_pct"].get<double>(),
                0.3, 1e-9);

    // Stripped of metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::food_packaging::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: vacuum pack with high O2 emits info, still proceeds ─────
TEST(SkillFoodPackaging, VacuumWithHighO2EmitsInfo)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 2.0);

    skill::food_packaging::Input in;
    in.package_type = "vacuum";
    in.gas_mix      = "vacuum";
    in.oxygen_pct   = 3.0;       // > 1 % under vacuum → info

    auto r = skill::food_packaging::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PKG-VACUUM"));

    auto out = skill::food_packaging::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("food_packaging_vacuum"));
}
