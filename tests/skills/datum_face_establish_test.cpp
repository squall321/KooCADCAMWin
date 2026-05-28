// @lat: [[process/test-strategy#skill round-trip]]
//
// datum_face_establish skill — 5-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/datum_face_establish.hpp"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
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

double topZ(const TopoDS_Shape& s)
{
    Bnd_Box b;
    BRepBndLib::AddOptimal(s, b);
    double xMin, yMin, zMin, xMax, yMax, zMax;
    b.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    return zMax;
}

}  // namespace

// ─── 1. Apply: light skim + datum-stamp signature ─────────────────────────
TEST(SkillDatumFaceEstablish, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());
    const double topBefore = topZ(stock->shape());

    skill::datum_face_establish::Input in;
    in.datum_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.datum_class          = "A";
    in.required_flatness_um = 10.0;
    in.depth_mm             = 0.5;

    auto out = skill::datum_face_establish::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    const double topAfter = topZ(out.workpiece->shape());

    // Removed ≈ 50 × 50 × 0.5 = 1250 mm³, top down by 0.5 mm.
    EXPECT_NEAR(volBefore - volAfter, 1250.0, 1250.0 * 0.02);
    EXPECT_NEAR(topBefore - topAfter, 0.5, 1e-3);

    EXPECT_EQ(out.signature.skill_id, std::string("datum_face_establish"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate: bad datum_class rejected ────────────────────────────────
TEST(SkillDatumFaceEstablish, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::datum_face_establish::Input in;
    in.datum_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.datum_class          = "Z";          // invalid
    in.required_flatness_um = 10.0;
    in.depth_mm             = 0.5;

    auto r = skill::datum_face_establish::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::datum_face_establish::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records datum_class + required_flatness_um ──────────────
TEST(SkillDatumFaceEstablish, SignatureRecordsKindAndKeys)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::datum_face_establish::Input in;
    in.datum_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.datum_class          = "B";
    in.required_flatness_um = 15.0;
    in.depth_mm             = 0.5;

    auto out = skill::datum_face_establish::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("datum_face_establish"));
    EXPECT_EQ(out.signature.pattern["datum_class"].get<std::string>(),
              std::string("B"));
    EXPECT_NEAR(out.signature.pattern["required_flatness_um"].get<double>(),
                15.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["depth_mm"].get<double>(),
                0.5, 1e-6);
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillDatumFaceEstablish, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::datum_face_establish::Input in;
    in.datum_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.datum_class          = "A";
    in.required_flatness_um = 8.0;
    in.depth_mm             = 0.3;
    auto out = skill::datum_face_establish::apply(*stock, in);

    auto cands = skill::datum_face_establish::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["datum_class"].get<std::string>(),
              std::string("A"));

    // Stripped workpiece → no metadata → empty recognize.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::datum_face_establish::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific assertion: loose flatness emits warning, still passes ────
TEST(SkillDatumFaceEstablish, LooseFlatnessIsWarningNotError)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::datum_face_establish::Input in;
    in.datum_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.datum_class          = "C";
    in.required_flatness_um = 80.0;        // > 50 µm → warning
    in.depth_mm             = 0.5;

    auto r = skill::datum_face_establish::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool foundWarn = false;
    for (const auto& f : r.findings)
        if (f.severity == "warning" && f.code == "DF-FLAT-LOOSE") {
            foundWarn = true; break;
        }
    EXPECT_TRUE(foundWarn);

    EXPECT_NO_THROW(skill::datum_face_establish::apply(*stock, in));
}
