// @lat: [[process/test-strategy#skill round-trip]]
//
// shot_peen_surface skill — 5-case sweep covering identity-geometry
// synthesis, intensity / coverage DFM rejection, signature metadata,
// metadata recognition, and target-face-area capture.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/shot_peen_surface.hpp"

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
TEST(SkillShotPeenSurface, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::shot_peen_surface::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.intensity_almen = 0.012;
    in.coverage_pct    = 200.0;
    in.shot_size_mm    = 0.8;

    auto out = skill::shot_peen_surface::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("shot_peen_surface"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects out-of-range intensity + coverage ───────────────
TEST(SkillShotPeenSurface, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::shot_peen_surface::Input weak;
    weak.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    weak.intensity_almen = 0.0005;        // < 0.002
    weak.coverage_pct    = 200.0;
    weak.shot_size_mm    = 0.8;
    auto r1 = skill::shot_peen_surface::validate(*stock, weak);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-PEEN-INTENSITY"));
    EXPECT_THROW(skill::shot_peen_surface::apply(*stock, weak),
                 skill::SkillError);

    skill::shot_peen_surface::Input lowCov;
    lowCov.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    lowCov.intensity_almen = 0.012;
    lowCov.coverage_pct    = 25.0;        // < 50
    lowCov.shot_size_mm    = 0.8;
    auto r2 = skill::shot_peen_surface::validate(*stock, lowCov);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-PEEN-COVERAGE"));

    skill::shot_peen_surface::Input zeroShot;
    zeroShot.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    zeroShot.intensity_almen = 0.012;
    zeroShot.coverage_pct    = 200.0;
    zeroShot.shot_size_mm    = 0.0;       // ≤ 0
    auto r3 = skill::shot_peen_surface::validate(*stock, zeroShot);
    EXPECT_FALSE(r3.passed);
    EXPECT_THROW(skill::shot_peen_surface::apply(*stock, zeroShot),
                 skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillShotPeenSurface, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::shot_peen_surface::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.intensity_almen = 0.020;
    in.coverage_pct    = 150.0;
    in.shot_size_mm    = 1.2;

    auto out = skill::shot_peen_surface::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("shot_peen_surface"));
    EXPECT_NEAR(out.signature.pattern["intensity_almen"].get<double>(),
                0.020, 1e-6);
    EXPECT_NEAR(out.signature.pattern["coverage_pct"].get<double>(),
                150.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["shot_size_mm"].get<double>(), 1.2, 1e-6);
    EXPECT_TRUE(out.signature.pattern.contains("target_face_id"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("shot_peening_machine"));
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillShotPeenSurface, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::shot_peen_surface::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.intensity_almen = 0.012;
    in.coverage_pct    = 200.0;
    in.shot_size_mm    = 0.8;
    auto out = skill::shot_peen_surface::apply(*stock, in);

    auto cands = skill::shot_peen_surface::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["intensity_almen"].get<double>(),
                0.012, 1e-6);

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::shot_peen_surface::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "shot_peen_surface cannot be recognized geometrically";
}

// ─── 5. Signature captures target face area (50×50 = 2500 mm²) ───────────
TEST(SkillShotPeenSurface, SignatureRecordsFaceArea)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::shot_peen_surface::Input in;
    in.entry_face      = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.intensity_almen = 0.012;
    in.coverage_pct    = 200.0;
    in.shot_size_mm    = 0.8;

    auto out = skill::shot_peen_surface::apply(*stock, in);
    EXPECT_NEAR(
        out.signature.pattern["target_face_area_mm2"].get<double>(),
        50.0 * 50.0, 1e-6);
}
