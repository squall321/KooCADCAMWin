// @lat: [[process/test-strategy#skill round-trip]]
//
// antireflective_coat skill — multilayer dielectric AR coat (metadata-only).
// 5 cases:
//   1. apply clones geometry unchanged
//   2. validate rejects inverted wavelength band
//   3. signature records kind + band + reflectance + layer_count
//   4. recognize via metadata replay
//   5. specific: out-of-practical-range reflectance rejected

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/antireflective_coat.hpp"

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

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillAntireflectiveCoat, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::antireflective_coat::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.wavelength_min_nm = 450.0;
    in.wavelength_max_nm = 650.0;
    in.reflectance_pct   = 0.5;
    in.layer_count       = 7;

    auto out = skill::antireflective_coat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("antireflective_coat"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. ValidateRejectsBadInput — inverted band ──────────────────────────
TEST(SkillAntireflectiveCoat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::antireflective_coat::Input bad;
    bad.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.wavelength_min_nm = 700.0;        // inverted
    bad.wavelength_max_nm = 400.0;
    bad.reflectance_pct   = 0.5;
    bad.layer_count       = 5;

    auto r = skill::antireflective_coat::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::antireflective_coat::apply(*stock, bad),
                 skill::SkillError);
}

// ─── 3. SignatureRecordsKind + key params ────────────────────────────────
TEST(SkillAntireflectiveCoat, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::antireflective_coat::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.wavelength_min_nm = 1000.0;
    in.wavelength_max_nm = 1600.0;
    in.reflectance_pct   = 0.25;
    in.layer_count       = 11;

    auto out = skill::antireflective_coat::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("antireflective_coat"));
    EXPECT_NEAR(out.signature.pattern["reflectance_pct"].get<double>(),
                0.25, 1e-9);
    EXPECT_EQ(out.signature.pattern["layer_count"].get<int>(), 11);
    EXPECT_EQ(out.signature.pattern["ar_class"].get<std::string>(),
              std::string("super_broadband_ar"));

    ASSERT_TRUE(out.signature.pattern.contains("wavelength_band_nm"));
    auto band = out.signature.pattern["wavelength_band_nm"];
    ASSERT_EQ(band.size(), 2u);
    EXPECT_NEAR(band[0].get<double>(), 1000.0, 1e-9);
    EXPECT_NEAR(band[1].get<double>(), 1600.0, 1e-9);
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillAntireflectiveCoat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::antireflective_coat::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.wavelength_min_nm = 450.0;
    in.wavelength_max_nm = 650.0;
    in.reflectance_pct   = 0.5;
    in.layer_count       = 7;

    auto out = skill::antireflective_coat::apply(*stock, in);

    auto cands = skill::antireflective_coat::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["layer_count"].get<int>(), 7);

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::antireflective_coat::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "antireflective_coat cannot be recognized geometrically";
}

// ─── 5. Specific: reflectance > 10 % is rejected ─────────────────────────
TEST(SkillAntireflectiveCoat, OutOfRangeReflectanceRejected)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::antireflective_coat::Input bad;
    bad.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.wavelength_min_nm = 450.0;
    bad.wavelength_max_nm = 650.0;
    bad.reflectance_pct   = 25.0;         // way too high
    bad.layer_count       = 7;

    auto r = skill::antireflective_coat::validate(*stock, bad);
    EXPECT_FALSE(r.passed);

    bool foundR = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-AR-REFL" && f.severity == "error") {
            foundR = true; break;
        }
    EXPECT_TRUE(foundR);
    EXPECT_THROW(skill::antireflective_coat::apply(*stock, bad),
                 skill::SkillError);
}
