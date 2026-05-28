// @lat: [[process/test-strategy#skill round-trip]]
//
// inkjet_mark skill — geometric-noop printed surface marking tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/inkjet_mark.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply does NOT change volume (geometric no-op) ────────────────────
TEST(SkillInkjetMark, ApplyIsGeometricNoOp)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::inkjet_mark::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 20.0;
    in.text          = "LOT2026";
    in.font_size_mm  = 2.0;
    in.ink_color     = "black";

    auto out = skill::inkjet_mark::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_NEAR(volAfter, volBefore, 1e-9);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("inkjet_mark"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.stock_removed_mm3, 0.0);
}

// ── 2. DFM-INKJET-FONT rejects sub-droplet font ──────────────────────────
TEST(SkillInkjetMark, ValidateRejectsTinyFont)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);

    skill::inkjet_mark::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.text         = "X";
    in.font_size_mm = 0.2;  // < 0.3
    in.ink_color    = "black";

    auto r = skill::inkjet_mark::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INKJET-FONT") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::inkjet_mark::apply(*stock, in), skill::SkillError);
}

// ── 3. Empty text rejected ───────────────────────────────────────────────
TEST(SkillInkjetMark, ValidateRejectsEmptyText)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);

    skill::inkjet_mark::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.text         = "";
    in.font_size_mm = 2.0;
    in.ink_color    = "black";

    auto r = skill::inkjet_mark::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 4. Empty ink_color emits warning but PASSES (defaulted) ──────────────
TEST(SkillInkjetMark, EmptyColorDefaults)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 10.0);

    skill::inkjet_mark::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 20.0;
    in.text          = "TEST";
    in.font_size_mm  = 2.0;
    in.ink_color     = "";   // → defaults to "black"

    auto r = skill::inkjet_mark::validate(*stock, in);
    EXPECT_TRUE(r.passed);          // warning only
    auto out = skill::inkjet_mark::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["ink_color"].get<std::string>(), "black");
}

// ── 5. Recognize replays metadata ────────────────────────────────────────
TEST(SkillInkjetMark, RecognizeReplaysMetadata)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::inkjet_mark::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 20.0;
    in.text          = "BATCH-A1";
    in.font_size_mm  = 1.5;
    in.ink_color     = "white";

    auto out = skill::inkjet_mark::apply(*stock, in);
    auto cands = skill::inkjet_mark::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].skill_id, std::string("inkjet_mark"));
    EXPECT_EQ(cands[0].recovered_params["text"], "BATCH-A1");
    EXPECT_EQ(cands[0].recovered_params["ink_color"], "white");
    EXPECT_NEAR(cands[0].recovered_params["font_size_mm"].get<double>(), 1.5, 1e-6);
    EXPECT_GT(cands[0].confidence, 0.9);
}

// ── 6. Recognize without metadata returns empty ──────────────────────────
TEST(SkillInkjetMark, RecognizeNoMetadataIsEmpty)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    skill::Workpiece raw(stock->shape());
    auto cands = skill::inkjet_mark::recognize(raw);
    EXPECT_TRUE(cands.empty());
}
