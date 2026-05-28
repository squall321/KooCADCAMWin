// @lat: [[process/test-strategy#skill round-trip]]
//
// lens_grind skill — metadata-only grind step.  5 cases covering:
//   1. identity geometry on apply
//   2. validate rejects bad input (aperture ≤ 0)
//   3. signature records kind + radius + aperture + surface_quality
//   4. recognize via metadata replay
//   5. impossible spherical-cap geometry (|R| < aperture/2) is rejected

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lens_grind.hpp"

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
TEST(SkillLensGrind, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::lens_grind::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.radius_curvature_mm = 100.0;
    in.aperture_mm         = 20.0;
    in.surface_quality     = "5/3x0.10";

    auto out = skill::lens_grind::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("lens_grind"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. ValidateRejectsBadInput — aperture ≤ 0 ───────────────────────────
TEST(SkillLensGrind, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::lens_grind::Input bad;
    bad.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.radius_curvature_mm = 100.0;
    bad.aperture_mm         = 0.0;        // invalid
    bad.surface_quality     = "5/3x0.10";

    auto r = skill::lens_grind::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::lens_grind::apply(*stock, bad), skill::SkillError);

    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. SignatureRecordsKind + key params ────────────────────────────────
TEST(SkillLensGrind, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::lens_grind::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.radius_curvature_mm = -50.0;       // concave
    in.aperture_mm         = 25.0;
    in.surface_quality     = "3/2x0.05";

    auto out = skill::lens_grind::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("lens_grind"));
    EXPECT_NEAR(out.signature.pattern["radius_curvature_mm"].get<double>(),
                -50.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["aperture_mm"].get<double>(),
                25.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["surface_quality"].get<std::string>(),
              std::string("3/2x0.05"));
    EXPECT_EQ(out.signature.pattern["curvature_kind"].get<std::string>(),
              std::string("concave"));
    EXPECT_TRUE(out.signature.pattern.contains("target_face_id"));
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillLensGrind, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::lens_grind::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.radius_curvature_mm = 75.0;
    in.aperture_mm         = 30.0;
    in.surface_quality     = "5/3x0.10";

    auto out = skill::lens_grind::apply(*stock, in);

    auto cands = skill::lens_grind::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["radius_curvature_mm"].get<double>(),
                75.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["aperture_mm"].get<double>(),
                30.0, 1e-9);

    // Stripped → empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::lens_grind::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "lens_grind cannot be recognized geometrically in phase-1";
}

// ─── 5. Specific assertion: |R| < aperture/2 is geometrically impossible ──
TEST(SkillLensGrind, ImpossibleSphericalCapRejected)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::lens_grind::Input bad;
    bad.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.radius_curvature_mm = 5.0;        // |R| = 5
    bad.aperture_mm         = 20.0;       // aperture/2 = 10 > |R|
    bad.surface_quality     = "5/3x0.10";

    auto r = skill::lens_grind::validate(*stock, bad);
    EXPECT_FALSE(r.passed);

    bool foundGeom = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LENS-GEOM" && f.severity == "error") {
            foundGeom = true; break;
        }
    EXPECT_TRUE(foundGeom);
    EXPECT_THROW(skill::lens_grind::apply(*stock, bad), skill::SkillError);
}
