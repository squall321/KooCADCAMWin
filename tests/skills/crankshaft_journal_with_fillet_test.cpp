// @lat: [[process/test-strategy#compound crankshaft_journal_with_fillet]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/crankshaft_journal_with_fillet.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

int findCylFaceId(const skill::Workpiece& wp) {
    for (int i = 0; i < wp.faceCount(); ++i)
        if (wp.isFaceCylinder(i)) return i;
    return -1;
}

skill::crankshaft_journal_with_fillet::Input goodInput(const skill::Workpiece& wp)
{
    skill::crankshaft_journal_with_fillet::Input in;
    in.face_id              = findCylFaceId(wp);
    in.journal_position_z_mm = 30.0;
    in.journal_dia_mm        = 50.0;
    in.journal_length_mm     = 20.0;
    in.fillet_radius_mm      = 3.0;
    return in;
}

}  // namespace

// ─── 1. apply removes annular material ───────────────────────────────────
TEST(SkillCrankshaftJournal, ApplyCutsJournalStep)
{
    // 60 mm OD shaft, 60 mm long.
    auto stock = skill::createCylindricalStock(60.0, 60.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::crankshaft_journal_with_fillet::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
}

// ─── 2. validate rejects fillet radius outside fatigue range ─────────────
TEST(SkillCrankshaftJournal, ValidateRejectsBadFilletRadius)
{
    auto stock = skill::createCylindricalStock(60.0, 60.0);
    auto in    = goodInput(*stock);
    in.fillet_radius_mm = 0.1;  // too small

    auto r = skill::crankshaft_journal_with_fillet::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FILLET-FATIGUE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::crankshaft_journal_with_fillet::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. validate rejects journal dia >= shaft OD ─────────────────────────
TEST(SkillCrankshaftJournal, ValidateRejectsJournalDiaTooLarge)
{
    auto stock = skill::createCylindricalStock(60.0, 60.0);
    auto in    = goodInput(*stock);
    in.journal_dia_mm = 65.0;   // > 60 mm shaft

    auto r = skill::crankshaft_journal_with_fillet::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-JOURNAL-DIA") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. signature records engine_feature_type + fillet edge count ────────
TEST(SkillCrankshaftJournal, SignatureRecordsEngineFeature)
{
    auto stock = skill::createCylindricalStock(60.0, 60.0);
    auto in    = goodInput(*stock);
    auto out   = skill::crankshaft_journal_with_fillet::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("crankshaft_journal_with_fillet"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("engine_feature_type")
              .get<std::string>(),
              std::string("crankshaft_journal"));
    EXPECT_NEAR(out.signature.pattern.at("fillet_radius_mm").get<double>(),
                3.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("journal_dia_mm").get<double>(),
                50.0, 1e-6);
    // We expect at least 1 edge filleted (both ends ideally).
    EXPECT_GE(out.signature.pattern.at("fillet_edges_filleted").get<int>(), 1);
}

// ─── 5. recognize via metadata replay ────────────────────────────────────
TEST(SkillCrankshaftJournal, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(60.0, 60.0);
    auto in    = goodInput(*stock);
    auto out   = skill::crankshaft_journal_with_fillet::apply(*stock, in);

    auto cands = skill::crankshaft_journal_with_fillet::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("journal_dia_mm").get<double>(),
              50.0);
}
