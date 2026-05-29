// @lat: [[process/test-strategy#skill round-trip]]
//
// replace_part skill — 5-case sweep:
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKind+params
//   4. RecognizeMetadataReplay
//   5. Specific: mismatched old/new id list sizes ⇒ DFM-REPL-COUNT error

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/replace_part.hpp"

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

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

}  // namespace

// ─── 1. Apply: geometry COMPLETELY unchanged ─────────────────────────────
TEST(SkillReplacePart, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::replace_part::Input in;
    in.old_part_ids  = { "bearing-7201", "seal-O30x2" };
    in.new_part_ids  = { "bearing-7201",  "seal-O30x2" };
    in.work_time_min = 25.0;

    auto out = skill::replace_part::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("replace_part"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillReplacePart, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::replace_part::Input emptyOld;
    emptyOld.new_part_ids  = { "X" };
    emptyOld.work_time_min = 10.0;
    EXPECT_FALSE(skill::replace_part::validate(*stock, emptyOld).passed);
    EXPECT_THROW(skill::replace_part::apply(*stock, emptyOld),
                 skill::SkillError);

    skill::replace_part::Input emptyNew;
    emptyNew.old_part_ids  = { "X" };
    emptyNew.work_time_min = 10.0;
    EXPECT_FALSE(skill::replace_part::validate(*stock, emptyNew).passed);
    EXPECT_THROW(skill::replace_part::apply(*stock, emptyNew),
                 skill::SkillError);

    skill::replace_part::Input badTime;
    badTime.old_part_ids  = { "X" };
    badTime.new_part_ids  = { "Y" };
    badTime.work_time_min = 0.0;
    EXPECT_FALSE(skill::replace_part::validate(*stock, badTime).passed);
    EXPECT_THROW(skill::replace_part::apply(*stock, badTime),
                 skill::SkillError);
}

// ─── 3. Signature records kind + params ──────────────────────────────────
TEST(SkillReplacePart, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::replace_part::Input in;
    in.old_part_ids  = { "A1", "B2", "C3" };
    in.new_part_ids  = { "A1n", "B2n", "C3n" };
    in.work_time_min = 35.0;

    auto out = skill::replace_part::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("replace_part"));
    ASSERT_TRUE(out.signature.pattern["old_part_ids"].is_array());
    ASSERT_TRUE(out.signature.pattern["new_part_ids"].is_array());
    EXPECT_EQ(out.signature.pattern["old_part_ids"].size(), 3u);
    EXPECT_EQ(out.signature.pattern["new_part_ids"].size(), 3u);
    EXPECT_EQ(out.signature.pattern["old_part_ids"][1].get<std::string>(),
              std::string("B2"));
    EXPECT_EQ(out.signature.pattern["new_part_ids"][2].get<std::string>(),
              std::string("C3n"));
    EXPECT_EQ(out.signature.pattern["part_count"].get<int>(), 3);
    EXPECT_NEAR(out.signature.pattern["work_time_min"].get<double>(), 35.0, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_NEAR(out.signature.params["work_time_min"].get<double>(), 35.0, 1e-6);
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillReplacePart, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::replace_part::Input in;
    in.old_part_ids  = { "gasket-A" };
    in.new_part_ids  = { "gasket-A" };
    in.work_time_min = 12.0;
    auto out = skill::replace_part::apply(*stock, in);

    auto cands = skill::replace_part::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["old_part_ids"][0].get<std::string>(),
              std::string("gasket-A"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::replace_part::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: mismatched id-list sizes ⇒ DFM-REPL-COUNT error ────────
TEST(SkillReplacePart, MismatchedListSizesRejected)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::replace_part::Input in;
    in.old_part_ids  = { "X1", "X2", "X3" };
    in.new_part_ids  = { "Y1", "Y2" };           // 3 vs 2
    in.work_time_min = 15.0;

    auto r = skill::replace_part::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-REPL-COUNT"));
    EXPECT_THROW(skill::replace_part::apply(*stock, in), skill::SkillError);
}
