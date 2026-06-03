// @lat: [[process/test-strategy#skill round-trip]]
//
// face_fillet_two_faces — apply + DFM + recognize sanity checks.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/face_fillet_two_faces.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <BRepAdaptor_Surface.hxx>
#include <gp_Dir.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

// Pick two planar faces of a cuboid that share an edge.  On a cuboid the
// face enumeration order from TopExp_Explorer typically lists adjacent
// sides first; pairs (0,2), (0,3), (1,2), (1,3) tend to share edges.
bool findAdjacentPair(const skill::Workpiece& wp, int& a, int& b)
{
    const int n = wp.faceCount();
    if (n < 2) return false;
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            // Skip parallel-face pairs — they don't share an edge.
            try {
                const gp_Dir ni = wp.faceNormal(i);
                const gp_Dir nj = wp.faceNormal(j);
                if (std::abs(ni.Dot(nj)) > 0.999) continue;   // (anti-)parallel
            } catch (...) { continue; }
            a = i; b = j;
            return true;
        }
    }
    return false;
}

}  // namespace

// ─── 1. Apply: fillet a shared edge between two non-parallel faces ────────
TEST(SkillFaceFilletTwoFaces, ApplyOnCuboidSucceeds)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    const double volBefore = volumeOf(stock->shape());

    int a = -1, b = -1;
    ASSERT_TRUE(findAdjacentPair(*stock, a, b));

    skill::face_fillet_two_faces::Input in;
    in.face_a_id = a;
    in.face_b_id = b;
    in.radius_mm = 1.0;

    auto out = skill::face_fillet_two_faces::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const int filleted =
        out.signature.pattern["edge_count_filleted"].get<int>();
    EXPECT_GT(filleted, 0);

    // Volume decreased — material removed.
    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_LT(volAfter, volBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("face_fillet_two_faces"));
}

// ─── 2. Validate rejects same face IDs ────────────────────────────────────
TEST(SkillFaceFilletTwoFaces, ValidateRejectsSameFace)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    skill::face_fillet_two_faces::Input in;
    in.face_a_id = 0;
    in.face_b_id = 0;
    in.radius_mm = 0.5;

    auto r = skill::face_fillet_two_faces::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::face_fillet_two_faces::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Parallel-face pair shares no edges → throws ──────────────────────
TEST(SkillFaceFilletTwoFaces, NoSharedEdgesThrows)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    // Find two faces with (anti-)parallel normals (e.g., top + bottom of cube).
    int a = -1, b = -1;
    for (int i = 0; i < stock->faceCount() && a < 0; ++i) {
        for (int j = i + 1; j < stock->faceCount(); ++j) {
            try {
                const gp_Dir ni = stock->faceNormal(i);
                const gp_Dir nj = stock->faceNormal(j);
                if (std::abs(ni.Dot(nj)) > 0.999) { a = i; b = j; break; }
            } catch (...) { /* skip */ }
        }
    }
    if (a < 0 || b < 0) {
        GTEST_SKIP() << "No parallel face pair found";
    }

    skill::face_fillet_two_faces::Input in;
    in.face_a_id = a;
    in.face_b_id = b;
    in.radius_mm = 0.5;

    EXPECT_THROW(skill::face_fillet_two_faces::apply(*stock, in),
                 skill::SkillError);
}
