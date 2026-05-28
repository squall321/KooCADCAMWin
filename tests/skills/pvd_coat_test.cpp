// @lat: [[process/test-strategy#skill round-trip]]
//
// pvd_coat skill — 6-case sweep covering identity-geometry synthesis,
// thickness clamps (very thin), compound table, hardness range info,
// metadata recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pvd_coat.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillPvdCoat, ApplyDoesNotChangeGeometry)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "carbide_K20");
    const double volBefore = volumeOf(stock->shape());

    skill::pvd_coat::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_compound = "TiN";
    in.thickness_um     = 3.0;
    in.hardness_hv      = 2200.0;

    auto out = skill::pvd_coat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);

    EXPECT_EQ(out.signature.skill_id, std::string("pvd_coat"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("pvd_chamber"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: thickness [0.5, 10] μm enforced ─────────────────────────────
TEST(SkillPvdCoat, ValidateRejectsOutOfRangeThickness)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::pvd_coat::Input tooThin;
    tooThin.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tooThin.coating_compound = "TiN";
    tooThin.thickness_um     = 0.2;
    EXPECT_FALSE(skill::pvd_coat::validate(*stock, tooThin).passed);
    EXPECT_THROW(skill::pvd_coat::apply(*stock, tooThin), skill::SkillError);

    skill::pvd_coat::Input tooThick = tooThin;
    tooThick.thickness_um = 25.0;
    EXPECT_FALSE(skill::pvd_coat::validate(*stock, tooThick).passed);
    EXPECT_THROW(skill::pvd_coat::apply(*stock, tooThick), skill::SkillError);
}

// ─── 3. Unknown compound → info, but apply succeeds ──────────────────────
TEST(SkillPvdCoat, UnknownCompoundInfoOnly)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::pvd_coat::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_compound = "ZrN";   // not in {TiN, TiAlN, CrN, DLC}
    in.thickness_um     = 2.0;
    in.hardness_hv      = 2500.0;

    auto r = skill::pvd_coat::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PVD-COMPOUND") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_NO_THROW(skill::pvd_coat::apply(*stock, in));
}

// ─── 4. Hardness outside compound's range emits info ─────────────────────
TEST(SkillPvdCoat, HardnessOutsideRangeInfo)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::pvd_coat::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_compound = "TiN";
    in.thickness_um     = 3.0;
    in.hardness_hv      = 800.0;     // far below TiN's 2000-2500 range

    auto r = skill::pvd_coat::validate(*stock, in);
    EXPECT_TRUE(r.passed);   // info, not error
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PVD-HARDNESS") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 5. Signature carries compound + hardness + color hint ───────────────
TEST(SkillPvdCoat, SignatureRecordsMetadata)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::pvd_coat::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_compound = "DLC";
    in.thickness_um     = 2.0;
    // hardness left at 0 → expect to be filled with compound midpoint

    auto out = skill::pvd_coat::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("pvd_coat"));
    EXPECT_EQ(out.signature.pattern["coating_compound"].get<std::string>(),
              std::string("DLC"));
    EXPECT_NEAR(out.signature.pattern["thickness_um"].get<double>(),
                2.0, 1e-6);
    // DLC midpoint = (1500 + 4000) / 2 = 2750 HV
    EXPECT_NEAR(out.signature.pattern["hardness_hv"].get<double>(),
                2750.0, 1.0);
    EXPECT_TRUE(out.signature.pattern.contains("color_rgb_hint"));
    EXPECT_TRUE(out.signature.pattern["hardness_in_compound_range"].get<bool>());
}

// ─── 6. Recognize via metadata replay ────────────────────────────────────
TEST(SkillPvdCoat, RecognizeFromMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::pvd_coat::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_compound = "TiAlN";
    in.thickness_um     = 4.0;
    in.hardness_hv      = 3200.0;

    auto out = skill::pvd_coat::apply(*stock, in);

    auto cands = skill::pvd_coat::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["coating_compound"].get<std::string>(),
              std::string("TiAlN"));

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::pvd_coat::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "pvd_coat cannot be recognized geometrically";
}
