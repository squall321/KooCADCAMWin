// @lat: [[process/test-strategy#skill round-trip]]
//
// lens_polish skill — metadata-only final-polish step.  5 cases:
//   1. apply clones geometry unchanged
//   2. validate rejects target_roughness_nm ≤ 0
//   3. signature records kind + target_roughness_nm + compound
//   4. recognize via metadata replay
//   5. specific: roughness below 0.2 nm pitch-lap floor is rejected as error

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lens_polish.hpp"

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
TEST(SkillLensPolish, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::lens_polish::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.target_roughness_nm = 2.0;
    in.polishing_compound  = "cerium_oxide";

    auto out = skill::lens_polish::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("lens_polish"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. ValidateRejectsBadInput — non-positive roughness ─────────────────
TEST(SkillLensPolish, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::lens_polish::Input bad;
    bad.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.target_roughness_nm = 0.0;        // invalid
    bad.polishing_compound  = "cerium_oxide";

    auto r = skill::lens_polish::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::lens_polish::apply(*stock, bad), skill::SkillError);

    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT" && f.severity == "error") {
            found = true; break;
        }
    EXPECT_TRUE(found);
}

// ─── 3. SignatureRecordsKind + key params ────────────────────────────────
TEST(SkillLensPolish, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::lens_polish::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.target_roughness_nm = 0.8;         // super-polish class
    in.polishing_compound  = "colloidal_silica";

    auto out = skill::lens_polish::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("lens_polish"));
    EXPECT_NEAR(out.signature.pattern["target_roughness_nm"].get<double>(),
                0.8, 1e-9);
    EXPECT_EQ(out.signature.pattern["polishing_compound"].get<std::string>(),
              std::string("colloidal_silica"));
    EXPECT_EQ(out.signature.pattern["polish_class"].get<std::string>(),
              std::string("super_polish"));
    EXPECT_TRUE(out.signature.pattern.contains("target_face_id"));
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillLensPolish, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::lens_polish::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.target_roughness_nm = 3.5;
    in.polishing_compound  = "diamond_slurry";

    auto out = skill::lens_polish::apply(*stock, in);

    auto cands = skill::lens_polish::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["target_roughness_nm"].get<double>(),
                3.5, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["polishing_compound"].get<std::string>(),
              std::string("diamond_slurry"));

    // Stripped → empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::lens_polish::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "lens_polish cannot be recognized geometrically in phase-1";
}

// ─── 5. Specific: 0.1 nm is below pitch-lap floor → error ────────────────
TEST(SkillLensPolish, BelowPitchLapFloorIsError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::lens_polish::Input bad;
    bad.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.target_roughness_nm = 0.1;        // < 0.2 nm floor
    bad.polishing_compound  = "cerium_oxide";

    auto r = skill::lens_polish::validate(*stock, bad);
    EXPECT_FALSE(r.passed);

    bool foundFloor = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LENS-POLISH-ROUGH" && f.severity == "error") {
            foundFloor = true; break;
        }
    EXPECT_TRUE(foundFloor);
    EXPECT_THROW(skill::lens_polish::apply(*stock, bad), skill::SkillError);
}
