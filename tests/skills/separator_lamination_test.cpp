// @lat: [[process/test-strategy#skill round-trip]]
//
// separator_lamination skill — battery-manufacturing metadata-only sweep.
// Covers:
//   1. apply clones the workpiece without touching geometry
//   2. validate rejects thickness / temperature bad input
//   3. signature records kind + key lamination params
//   4. recognize replays metadata only
//   5. unknown film material emits info but proceeds

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/separator_lamination.hpp"

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

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillSeparatorLamination, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 0.1, "coated_electrode");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::separator_lamination::Input in;
    in.film_thickness_um  = 20.0;
    in.lamination_temp_c  = 90.0;
    in.film_material      = "PE";
    in.roll_pressure_bar  = 5.0;

    auto out = skill::separator_lamination::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("separator_lamination"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: rejects bad inputs ──────────────────────────────────────────
TEST(SkillSeparatorLamination, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 0.1);

    // 2a. film too thin
    skill::separator_lamination::Input tooThin;
    tooThin.film_thickness_um = 2.0;
    tooThin.lamination_temp_c = 90.0;
    auto r1 = skill::separator_lamination::validate(*stock, tooThin);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-SEP-THK"));
    EXPECT_THROW(skill::separator_lamination::apply(*stock, tooThin),
                 skill::SkillError);

    // 2b. temperature too low for adhesion
    skill::separator_lamination::Input tooCold;
    tooCold.film_thickness_um = 20.0;
    tooCold.lamination_temp_c = 30.0;
    auto r2 = skill::separator_lamination::validate(*stock, tooCold);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-SEP-TEMP-LOW"));

    // 2c. temperature too high — PE pore collapse risk
    skill::separator_lamination::Input tooHot;
    tooHot.film_thickness_um = 20.0;
    tooHot.lamination_temp_c = 160.0;
    auto r3 = skill::separator_lamination::validate(*stock, tooHot);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-SEP-TEMP-HIGH"));
    EXPECT_THROW(skill::separator_lamination::apply(*stock, tooHot),
                 skill::SkillError);
}

// ─── 3. Signature records kind + key parameters ──────────────────────────
TEST(SkillSeparatorLamination, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 0.1);

    skill::separator_lamination::Input in;
    in.film_thickness_um  = 16.0;
    in.lamination_temp_c  = 100.0;
    in.film_material      = "ceramic_coated_PE";
    in.roll_pressure_bar  = 8.0;

    auto out = skill::separator_lamination::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("separator_lamination"));
    EXPECT_NEAR(out.signature.pattern["film_thickness_um"].get<double>(),
                16.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["lamination_temp_c"].get<double>(),
                100.0, 1e-6);
    EXPECT_EQ(out.signature.pattern["film_material"].get<std::string>(),
              std::string("ceramic_coated_PE"));
    EXPECT_NEAR(out.signature.pattern["roll_pressure_bar"].get<double>(),
                8.0, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("heated_lamination_roll"));
}

// ─── 4. Recognize replays metadata only ──────────────────────────────────
TEST(SkillSeparatorLamination, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 0.1);

    skill::separator_lamination::Input in;
    in.film_thickness_um  = 25.0;
    in.lamination_temp_c  = 95.0;
    in.film_material      = "PE_PP_PE";
    in.roll_pressure_bar  = 6.0;

    auto out = skill::separator_lamination::apply(*stock, in);

    auto cands = skill::separator_lamination::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["film_material"].get<std::string>(),
              std::string("PE_PP_PE"));
    EXPECT_NEAR(cands[0].recovered_params["film_thickness_um"].get<double>(),
                25.0, 1e-6);

    skill::Workpiece raw(out.workpiece->shape());
    auto empty = skill::separator_lamination::recognize(raw);
    EXPECT_TRUE(empty.empty())
        << "separator_lamination cannot be recognized geometrically";
}

// ─── 5. Specific: unknown film material is INFO-only, does not block ────
TEST(SkillSeparatorLamination, UnknownFilmMaterialEmitsInfoAndProceeds)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 0.1);

    skill::separator_lamination::Input exotic;
    exotic.film_thickness_um = 20.0;
    exotic.lamination_temp_c = 90.0;
    exotic.film_material     = "PVDF_HFP_gel";       // research-grade chemistry
    exotic.roll_pressure_bar = 5.0;

    auto r = skill::separator_lamination::validate(*stock, exotic);
    EXPECT_TRUE(r.passed)
        << "unknown film material is info-only, must not block";
    EXPECT_TRUE(hasFinding(r, "DFM-SEP-MATERIAL"));
    EXPECT_NO_THROW(skill::separator_lamination::apply(*stock, exotic));
}
