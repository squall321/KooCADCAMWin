// @lat: [[process/test-strategy#bb_shell_thread_bsa]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bb_shell_thread_bsa.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::bb_shell_thread_bsa::Input goodInput()
{
    skill::bb_shell_thread_bsa::Input in;
    in.face_id                = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_origin            = gp_Pnt(0.0, 0.0, 0.0);
    in.shell_width_mm         = 68.0;
    in.shell_bore_dia_mm      = 34.8;
    in.thread_relief_width_mm = 3.0;
    return in;
}
}  // namespace

TEST(SkillBbShellThreadBsa, ApplyRemovesMaterial)
{
    // Cylindrical shell stock: Ø48 x 70 mm tall (axis along Z).
    auto stock = skill::createCylindricalStock(48.0, 70.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::bb_shell_thread_bsa::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // bore + 2 relief grooves removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillBbShellThreadBsa, ValidateRejectsNonStandardShell)
{
    auto stock = skill::createCylindricalStock(48.0, 70.0);
    auto in = goodInput();
    in.shell_width_mm = 100.0;   // not 68/73

    auto r = skill::bb_shell_thread_bsa::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SHELL") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::bb_shell_thread_bsa::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillBbShellThreadBsa, ValidateRejectsBadBore)
{
    auto stock = skill::createCylindricalStock(48.0, 70.0);
    auto in = goodInput();
    in.shell_bore_dia_mm = 30.0;   // far from BSA 34.8 major

    auto r = skill::bb_shell_thread_bsa::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BORE") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillBbShellThreadBsa, SignatureCompoundBsaShell)
{
    auto stock = skill::createCylindricalStock(48.0, 70.0);
    auto in    = goodInput();
    auto out   = skill::bb_shell_thread_bsa::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("bb_shell_thread_bsa"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("bicycle_feature_type", std::string()),
              std::string("bsa_threaded_bb_shell"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillBbShellThreadBsa, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(48.0, 70.0);
    auto in    = goodInput();
    auto out   = skill::bb_shell_thread_bsa::apply(*stock, in);
    auto cands = skill::bb_shell_thread_bsa::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
