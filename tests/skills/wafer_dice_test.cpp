// @lat: [[process/test-strategy#skill round-trip]]
//
// wafer_dice — semiconductor singulation metadata-only skill.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/wafer_dice.hpp"

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

// ── 1. Apply: geometry unchanged (clone, volume preserved) ───────────────
TEST(SkillWaferDice, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::wafer_dice::Input in;
    in.wafer_dia_mm  = 150.0;
    in.die_size_x_mm = 5.0;
    in.die_size_y_mm = 5.0;
    in.kerf_um       = 30.0;

    auto out = skill::wafer_dice::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. ValidateRejectsBadInput: zero diameter, zero die, zero kerf ───────
TEST(SkillWaferDice, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");

    skill::wafer_dice::Input zeroDia;
    zeroDia.wafer_dia_mm = 0.0;
    auto r1 = skill::wafer_dice::validate(*stock, zeroDia);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::wafer_dice::apply(*stock, zeroDia), skill::SkillError);

    skill::wafer_dice::Input zeroDie;
    zeroDie.die_size_x_mm = 0.0;
    auto r2 = skill::wafer_dice::validate(*stock, zeroDie);
    EXPECT_FALSE(r2.passed);

    skill::wafer_dice::Input zeroKerf;
    zeroKerf.kerf_um = 0.0;
    auto r3 = skill::wafer_dice::validate(*stock, zeroKerf);
    EXPECT_FALSE(r3.passed);

    // Kerf >= die dimension → mass-error (whole die would be consumed).
    skill::wafer_dice::Input kerfTooBig;
    kerfTooBig.die_size_x_mm = 1.0;        // 1 mm = 1000 μm
    kerfTooBig.die_size_y_mm = 5.0;
    kerfTooBig.kerf_um       = 2000.0;     // 2 mm — bigger than die
    auto r4 = skill::wafer_dice::validate(*stock, kerfTooBig);
    EXPECT_FALSE(r4.passed);
}

// ── 3. SignatureRecordsKind + key params (wafer_dia, die_size, kerf) ─────
TEST(SkillWaferDice, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(200.0, 0.775, "silicon_wafer");

    skill::wafer_dice::Input in;
    in.wafer_dia_mm  = 200.0;
    in.die_size_x_mm = 10.0;
    in.die_size_y_mm = 8.0;
    in.kerf_um       = 50.0;

    auto out = skill::wafer_dice::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("wafer_dice"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("wafer_dice"));
    EXPECT_NEAR(out.signature.pattern["wafer_dia_mm"].get<double>(),
                200.0, 1e-12);
    EXPECT_NEAR(out.signature.pattern["die_size_x_mm"].get<double>(),
                10.0, 1e-12);
    EXPECT_NEAR(out.signature.pattern["die_size_y_mm"].get<double>(),
                8.0, 1e-12);
    EXPECT_NEAR(out.signature.pattern["kerf_um"].get<double>(),
                50.0, 1e-12);
    EXPECT_TRUE(out.signature.pattern["is_separation"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("dicing_saw"));
}

// ── 4. RecognizeMetadataReplay (confidence 1.0; stripped → none) ─────────
TEST(SkillWaferDice, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");

    skill::wafer_dice::Input in;
    in.wafer_dia_mm  = 150.0;
    in.die_size_x_mm = 5.0;
    in.die_size_y_mm = 5.0;
    in.kerf_um       = 30.0;
    auto out = skill::wafer_dice::apply(*stock, in);

    auto cands = skill::wafer_dice::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].skill_id, std::string("wafer_dice"));
    EXPECT_NEAR(cands[0].recovered_params["wafer_dia_mm"].get<double>(),
                150.0, 1e-12);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::wafer_dice::recognize(raw).empty());
}

// ── 5. SPECIFIC: die_count_est reflects wafer area / die pitch ───────────
TEST(SkillWaferDice, DieCountEstimateScalesWithWafer)
{
    auto stock150 = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");
    auto stock300 = skill::createCylindricalStock(300.0, 0.775, "silicon_wafer");

    skill::wafer_dice::Input in;
    in.die_size_x_mm = 5.0;
    in.die_size_y_mm = 5.0;
    in.kerf_um       = 30.0;

    in.wafer_dia_mm = 150.0;
    auto out150 = skill::wafer_dice::apply(*stock150, in);
    in.wafer_dia_mm = 300.0;
    auto out300 = skill::wafer_dice::apply(*stock300, in);

    const int n150 = out150.signature.pattern["die_count_est"].get<int>();
    const int n300 = out300.signature.pattern["die_count_est"].get<int>();

    EXPECT_GT(n150, 0);
    EXPECT_GT(n300, n150)
        << "300 mm wafer must yield more dies than 150 mm with same die size";
    // 300 mm has ~ 4× the area of 150 mm — die count should be roughly 4× too.
    EXPECT_GT(n300, 3 * n150);
}
