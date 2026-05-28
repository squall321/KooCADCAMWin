// @lat: [[process/test-strategy#skill round-trip]]
//
// mirror_finish skill — high-reflectivity mirror coating (metadata-only).
// 5 cases:
//   1. apply clones geometry unchanged
//   2. validate rejects reflectivity out of (0, 100]
//   3. signature records kind + reflectivity + wavelength_band
//   4. recognize via metadata replay
//   5. specific: reflectivity ≥ 99.5 → dielectric_high_reflector coating_class

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mirror_finish.hpp"

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
TEST(SkillMirrorFinish, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::mirror_finish::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.reflectivity_pct  = 95.0;
    in.wavelength_min_nm = 400.0;
    in.wavelength_max_nm = 700.0;

    auto out = skill::mirror_finish::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("mirror_finish"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. ValidateRejectsBadInput — reflectivity > 100 ─────────────────────
TEST(SkillMirrorFinish, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::mirror_finish::Input bad;
    bad.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.reflectivity_pct  = 150.0;        // > 100 — impossible
    bad.wavelength_min_nm = 400.0;
    bad.wavelength_max_nm = 700.0;

    auto r = skill::mirror_finish::validate(*stock, bad);
    EXPECT_FALSE(r.passed);

    bool foundR = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MIRROR-REFL" && f.severity == "error") {
            foundR = true; break;
        }
    EXPECT_TRUE(foundR);
    EXPECT_THROW(skill::mirror_finish::apply(*stock, bad), skill::SkillError);
}

// ─── 3. SignatureRecordsKind + key params ────────────────────────────────
TEST(SkillMirrorFinish, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::mirror_finish::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.reflectivity_pct  = 96.5;
    in.wavelength_min_nm = 600.0;
    in.wavelength_max_nm = 800.0;

    auto out = skill::mirror_finish::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("mirror_finish"));
    EXPECT_NEAR(out.signature.pattern["reflectivity_pct"].get<double>(),
                96.5, 1e-9);

    ASSERT_TRUE(out.signature.pattern.contains("wavelength_band_nm"));
    auto band = out.signature.pattern["wavelength_band_nm"];
    ASSERT_EQ(band.size(), 2u);
    EXPECT_NEAR(band[0].get<double>(), 600.0, 1e-9);
    EXPECT_NEAR(band[1].get<double>(), 800.0, 1e-9);

    EXPECT_EQ(out.signature.pattern["coating_class"].get<std::string>(),
              std::string("protected_metal"));
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillMirrorFinish, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::mirror_finish::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.reflectivity_pct  = 92.0;
    in.wavelength_min_nm = 400.0;
    in.wavelength_max_nm = 700.0;

    auto out = skill::mirror_finish::apply(*stock, in);

    auto cands = skill::mirror_finish::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["reflectivity_pct"].get<double>(),
                92.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::mirror_finish::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "mirror_finish cannot be recognized geometrically";
}

// ─── 5. Specific: ≥ 99.5 % reflectivity → dielectric_high_reflector ──────
TEST(SkillMirrorFinish, DielectricHighReflectorClassified)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::mirror_finish::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.reflectivity_pct  = 99.7;       // requires dielectric HR stack
    in.wavelength_min_nm = 1030.0;
    in.wavelength_max_nm = 1080.0;     // Yb:YAG laser line

    auto out = skill::mirror_finish::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["coating_class"].get<std::string>(),
              std::string("dielectric_high_reflector"));
    EXPECT_EQ(out.signature.tooling.tool_material,
              std::string("SiO2_TiO2_HR_stack"));
}
