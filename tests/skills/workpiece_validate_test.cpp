// @lat: [[engine/skills#Workpiece]]
//
// Skill-synthesis output gate (modeling roadmap #1).  The 250+-skill path
// historically wrapped raw Boolean output in a Workpiece with NO validity
// check, so a null / empty / invalid BRep silently became a "valid" Workpiece.
// validateOrHeal (routed through finalizeOutput) + the ctor null-guard close
// that hole: null/empty/unhealable input is rejected as a SkillError, valid
// input passes through unchanged.  drill_hole_test proves the integration on a
// real skill (its epilogue now goes through validateOrHeal).

#include <gtest/gtest.h>

#include "skills/Skill.hpp"
#include "skills/Workpiece.hpp"

#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

using namespace koocadcam;
using namespace koocadcam::skill;

namespace {

TopoDS_Shape makeBox()
{
    return BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 10.0, 10.0, 10.0).Shape();
}

TopoDS_Shape emptyCompound()
{
    TopoDS_Compound c;
    BRep_Builder().MakeCompound(c);
    return c;   // valid handle, but no faces
}

}  // namespace

// A valid solid passes through validateOrHeal unchanged and stays valid.
TEST(WorkpieceValidate, ValidShapePassesThrough)
{
    const TopoDS_Shape box = makeBox();
    TopoDS_Shape out;
    ASSERT_NO_THROW(out = validateOrHeal(box, "test"));
    EXPECT_FALSE(out.IsNull());
    EXPECT_TRUE(BRepCheck_Analyzer(out).IsValid());
    EXPECT_TRUE(TopExp_Explorer(out, TopAbs_FACE).More());
}

// A null shape is silent corruption — it must throw, never return.
TEST(WorkpieceValidate, NullShapeThrows)
{
    EXPECT_THROW(validateOrHeal(TopoDS_Shape(), "test"), SkillError);
}

// An empty compound (a cut that consumed everything / a failed Boolean) has no
// faces — reject it rather than let an empty Workpiece propagate.
TEST(WorkpieceValidate, EmptyCompoundThrows)
{
    EXPECT_THROW(validateOrHeal(emptyCompound(), "test"), SkillError);
}

// The ctor's baseline invariant: a Workpiece is never built from a null shape,
// protecting the not-yet-migrated construction sites.
TEST(WorkpieceValidate, CtorRejectsNull)
{
    const TopoDS_Shape nullShape;
    EXPECT_THROW(Workpiece{ nullShape }, SkillError);
}

// A valid shape still constructs normally (no false rejection).
TEST(WorkpieceValidate, ValidShapeConstructs)
{
    ASSERT_NO_THROW({
        Workpiece wp(makeBox());
        EXPECT_EQ(wp.faceCount(), 6);
    });
}
