// @lat: [[process/test-strategy#skill round-trip]]
//
// package_medical — medical-grade barrier packaging.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. UnknownLabelEmitsInfoButProceeds

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/package_medical.hpp"

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
TEST(SkillPackageMedical, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::package_medical::Input in;
    in.barrier_class     = "tyvek_pouch";
    in.label_compliance  = "ISO 11607";
    in.shelf_life_months = 36.0;

    auto out = skill::package_medical::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillPackageMedical, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 3.0);

    // Unknown barrier.
    skill::package_medical::Input badBarrier;
    badBarrier.barrier_class     = "ziplock_bag";
    badBarrier.label_compliance  = "ISO 11607";
    badBarrier.shelf_life_months = 24.0;
    {
        auto r = skill::package_medical::validate(*stock, badBarrier);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::package_medical::apply(*stock, badBarrier),
                 skill::SkillError);

    // Shelf life > 120.
    skill::package_medical::Input longShelf;
    longShelf.barrier_class     = "tyvek_pouch";
    longShelf.label_compliance  = "ISO 11607";
    longShelf.shelf_life_months = 200.0;
    {
        auto r = skill::package_medical::validate(*stock, longShelf);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-PACKAGE-SHELF"));
    }

    // Empty label.
    skill::package_medical::Input noLabel;
    noLabel.barrier_class     = "tyvek_pouch";
    noLabel.label_compliance  = "";
    noLabel.shelf_life_months = 24.0;
    {
        auto r = skill::package_medical::validate(*stock, noLabel);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
}

// ─── 3. Signature records kind + key params ────────────────────────────────
TEST(SkillPackageMedical, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::package_medical::Input in;
    in.barrier_class     = "rigid_tray";
    in.label_compliance  = "ISO 11607-1";
    in.shelf_life_months = 60.0;

    auto out = skill::package_medical::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("package_medical"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("package_medical"));
    EXPECT_EQ(out.signature.pattern["barrier_class"].get<std::string>(),
              std::string("rigid_tray"));
    EXPECT_EQ(out.signature.pattern["label_compliance"].get<std::string>(),
              std::string("ISO 11607-1"));
    EXPECT_NEAR(out.signature.pattern["shelf_life_months"].get<double>(),
                60.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ──────────────────────────────────────────
TEST(SkillPackageMedical, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::package_medical::Input in;
    in.barrier_class     = "double_pouch";
    in.label_compliance  = "ISO 11607-2";
    in.shelf_life_months = 48.0;

    auto out   = skill::package_medical::apply(*stock, in);
    auto cands = skill::package_medical::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["barrier_class"].get<std::string>(),
              std::string("double_pouch"));
    EXPECT_NEAR(cands[0].recovered_params["shelf_life_months"].get<double>(),
                48.0, 1e-9);

    // Stripped of metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::package_medical::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: unknown label compliance emits info but proceeds ─────────
TEST(SkillPackageMedical, UnknownLabelEmitsInfoButProceeds)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 2.0);

    skill::package_medical::Input in;
    in.barrier_class     = "tyvek_pouch";
    in.label_compliance  = "MIL-STD-2073";    // not in regulatory catalog
    in.shelf_life_months = 36.0;

    auto r = skill::package_medical::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PACKAGE-LABEL"));

    auto out = skill::package_medical::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("heat_sealer"));
}
