// @lat: [[process/test-strategy#skill round-trip]]
//
// anodize skill — 6-case sweep covering identity-geometry synthesis,
// thickness DFM clamps, color/seal/material info hints, metadata recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/anodize.hpp"

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
TEST(SkillAnodize, ApplyDoesNotChangeGeometry)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::anodize::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_thickness_um = 25.0;
    in.color                = "black";
    in.seal_type            = "hot_water";

    auto out = skill::anodize::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("anodize"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("anodize_bath"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: thickness below 5 μm or above 75 μm rejected ────────────────
TEST(SkillAnodize, ValidateRejectsOutOfRangeThickness)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::anodize::Input tooThin;
    tooThin.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tooThin.coating_thickness_um = 2.0;
    tooThin.color                = "natural";
    auto r1 = skill::anodize::validate(*stock, tooThin);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::anodize::apply(*stock, tooThin), skill::SkillError);

    skill::anodize::Input tooThick = tooThin;
    tooThick.coating_thickness_um = 120.0;
    auto r2 = skill::anodize::validate(*stock, tooThick);
    EXPECT_FALSE(r2.passed);
    EXPECT_THROW(skill::anodize::apply(*stock, tooThick), skill::SkillError);

    bool found = false;
    for (const auto& f : r1.findings)
        if (f.code == "DFM-ANODIZE-THK") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Signature carries face_id + thickness + color + seal metadata ────
TEST(SkillAnodize, SignatureRecordsMetadata)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::anodize::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_thickness_um = 60.0;     // Type III hard
    in.color                = "gold";
    in.seal_type            = "nickel_acetate";

    auto out = skill::anodize::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("anodize"));
    EXPECT_EQ(out.signature.pattern["color"].get<std::string>(),
              std::string("gold"));
    EXPECT_EQ(out.signature.pattern["seal_type"].get<std::string>(),
              std::string("nickel_acetate"));
    EXPECT_NEAR(out.signature.pattern["coating_thickness_um"].get<double>(),
                60.0, 1e-6);
    EXPECT_TRUE(out.signature.pattern.contains("target_face_id"));
    EXPECT_EQ(out.signature.pattern["anodize_class"].get<std::string>(),
              std::string("Type_III_hard"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Material-family info hint on non-aluminum substrate ──────────────
TEST(SkillAnodize, NonAluminumInfoHint)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "steel_1018");

    skill::anodize::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_thickness_um = 25.0;
    in.color                = "black";

    auto r = skill::anodize::validate(*stock, in);
    EXPECT_TRUE(r.passed);   // info, not error
    bool foundMat = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ANODIZE-MATERIAL") { foundMat = true; break; }
    EXPECT_TRUE(foundMat);
    EXPECT_NO_THROW(skill::anodize::apply(*stock, in));
}

// ─── 5. Recognize via metadata replay returns exactly one candidate ──────
TEST(SkillAnodize, RecognizeFromMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::anodize::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_thickness_um = 25.0;
    in.color                = "red";
    in.seal_type            = "hot_water";

    auto out = skill::anodize::apply(*stock, in);

    auto cands = skill::anodize::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["color"].get<std::string>(),
              std::string("red"));

    // Strip features — recognize returns empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::anodize::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "anodize cannot be recognized geometrically";
}

// ─── 6. Multiple passes stack signatures ─────────────────────────────────
TEST(SkillAnodize, MultiplePassesStackSignatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::anodize::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_thickness_um = 20.0;
    in.color                = "natural";
    in.seal_type            = "hot_water";

    auto w1 = skill::anodize::apply(*stock, in);

    in.color = "blue";
    auto w2 = skill::anodize::apply(*w1.workpiece, in);
    EXPECT_EQ(w2.workpiece->features().size(), 2u);

    auto cands = skill::anodize::recognize(*w2.workpiece);
    EXPECT_EQ(cands.size(), 2u);
}
