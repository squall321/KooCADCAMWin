// @lat: [[process/test-strategy#skill round-trip]]
//
// electrode_coat skill — battery-manufacturing metadata-only sweep.
// Covers:
//   1. apply clones the workpiece without touching geometry
//   2. validate rejects thickness / capacity / electrode_side bad input
//   3. signature records kind + key chemistry/loading params
//   4. recognize replays metadata only
//   5. cathode-on-Cu mismatch is INFO-only (not blocking)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/electrode_coat.hpp"

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
TEST(SkillElectrodeCoat, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 0.02, "aluminum_foil");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::electrode_coat::Input in;
    in.active_material         = "NMC811";
    in.electrode_side          = "cathode";
    in.thickness_um            = 80.0;
    in.areal_capacity_mAh_cm2  = 3.5;
    in.collector_foil_material = "aluminum";
    in.coating_method          = "slot_die";

    auto out = skill::electrode_coat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("electrode_coat"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: rejects bad inputs ──────────────────────────────────────────
TEST(SkillElectrodeCoat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 0.02);

    // 2a. thickness below industrial window
    skill::electrode_coat::Input tooThin;
    tooThin.thickness_um            = 10.0;
    tooThin.areal_capacity_mAh_cm2  = 3.0;
    tooThin.electrode_side          = "cathode";
    tooThin.collector_foil_material = "aluminum";
    auto r1 = skill::electrode_coat::validate(*stock, tooThin);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-COAT-THK"));
    EXPECT_THROW(skill::electrode_coat::apply(*stock, tooThin),
                 skill::SkillError);

    // 2b. unknown electrode_side
    skill::electrode_coat::Input badSide;
    badSide.thickness_um            = 80.0;
    badSide.areal_capacity_mAh_cm2  = 3.0;
    badSide.electrode_side          = "middle";
    badSide.collector_foil_material = "aluminum";
    auto r2 = skill::electrode_coat::validate(*stock, badSide);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-COAT-SIDE"));

    // 2c. non-positive capacity
    skill::electrode_coat::Input zeroCap;
    zeroCap.thickness_um            = 80.0;
    zeroCap.areal_capacity_mAh_cm2  = 0.0;
    zeroCap.electrode_side          = "cathode";
    zeroCap.collector_foil_material = "aluminum";
    auto r3 = skill::electrode_coat::validate(*stock, zeroCap);
    EXPECT_FALSE(r3.passed);
    EXPECT_THROW(skill::electrode_coat::apply(*stock, zeroCap),
                 skill::SkillError);
}

// ─── 3. Signature records kind + key parameters ──────────────────────────
TEST(SkillElectrodeCoat, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 0.02, "copper_foil");

    skill::electrode_coat::Input in;
    in.active_material         = "graphite";
    in.electrode_side          = "anode";
    in.thickness_um            = 100.0;
    in.areal_capacity_mAh_cm2  = 3.2;
    in.collector_foil_material = "copper";
    in.coating_method          = "comma_bar";

    auto out = skill::electrode_coat::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("electrode_coat"));
    EXPECT_EQ(out.signature.pattern["active_material"].get<std::string>(),
              std::string("graphite"));
    EXPECT_EQ(out.signature.pattern["electrode_side"].get<std::string>(),
              std::string("anode"));
    EXPECT_NEAR(out.signature.pattern["thickness_um"].get<double>(),
                100.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["areal_capacity_mAh_cm2"].get<double>(),
                3.2, 1e-6);
    EXPECT_EQ(out.signature.pattern["coating_method"].get<std::string>(),
              std::string("comma_bar"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("slot_die_coater"));
}

// ─── 4. Recognize replays metadata only ──────────────────────────────────
TEST(SkillElectrodeCoat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 0.02);

    skill::electrode_coat::Input in;
    in.active_material         = "LFP";
    in.electrode_side          = "cathode";
    in.thickness_um            = 120.0;
    in.areal_capacity_mAh_cm2  = 2.5;
    in.collector_foil_material = "aluminum";

    auto out = skill::electrode_coat::apply(*stock, in);

    auto cands = skill::electrode_coat::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["active_material"].get<std::string>(),
              std::string("LFP"));
    EXPECT_NEAR(
        cands[0].recovered_params["areal_capacity_mAh_cm2"].get<double>(),
        2.5, 1e-6);

    // Strip metadata → recognize returns empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto empty = skill::electrode_coat::recognize(raw);
    EXPECT_TRUE(empty.empty())
        << "electrode_coat cannot be recognized geometrically";
}

// ─── 5. Specific: cathode-on-Cu mismatch is INFO-only (proceeds) ────────
TEST(SkillElectrodeCoat, FoilMismatchEmitsInfoAndProceeds)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 0.02);

    skill::electrode_coat::Input mismatch;
    mismatch.active_material         = "NMC811";       // cathode chemistry
    mismatch.electrode_side          = "cathode";
    mismatch.thickness_um            = 80.0;
    mismatch.areal_capacity_mAh_cm2  = 3.5;
    mismatch.collector_foil_material = "copper";       // unusual pairing
    mismatch.coating_method          = "slot_die";

    auto r = skill::electrode_coat::validate(*stock, mismatch);
    EXPECT_TRUE(r.passed) << "foil mismatch is info-only, must not block";
    EXPECT_TRUE(hasFinding(r, "DFM-COAT-FOIL-MISMATCH"));
    EXPECT_NO_THROW(skill::electrode_coat::apply(*stock, mismatch));
}
