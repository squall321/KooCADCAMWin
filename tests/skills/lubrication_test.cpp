// @lat: [[process/test-strategy#skill round-trip]]
//
// lubrication skill — 5-case sweep:
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKind+params
//   4. RecognizeMetadataReplay
//   5. Specific: unrecognised lube_type / method emit info-only (no block)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lubrication.hpp"

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
TEST(SkillLubrication, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::lubrication::Input in;
    in.lube_type          = "oil";
    in.volume_ml          = 5.0;
    in.application_method = "drip";

    auto out = skill::lubrication::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("lubrication"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillLubrication, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::lubrication::Input zeroVol;
    zeroVol.lube_type          = "oil";
    zeroVol.volume_ml          = 0.0;
    zeroVol.application_method = "brush";
    EXPECT_FALSE(skill::lubrication::validate(*stock, zeroVol).passed);
    EXPECT_THROW(skill::lubrication::apply(*stock, zeroVol),
                 skill::SkillError);

    skill::lubrication::Input negVol = zeroVol;
    negVol.volume_ml = -2.0;
    EXPECT_FALSE(skill::lubrication::validate(*stock, negVol).passed);
    EXPECT_THROW(skill::lubrication::apply(*stock, negVol),
                 skill::SkillError);
}

// ─── 3. Signature records kind + params ──────────────────────────────────
TEST(SkillLubrication, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::lubrication::Input in;
    in.lube_type          = "grease";
    in.volume_ml          = 12.5;
    in.application_method = "inject";

    auto out = skill::lubrication::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("lubrication"));
    EXPECT_EQ(out.signature.pattern["lube_type"].get<std::string>(),
              std::string("grease"));
    EXPECT_NEAR(out.signature.pattern["volume_ml"].get<double>(), 12.5, 1e-6);
    EXPECT_EQ(out.signature.pattern["application_method"].get<std::string>(),
              std::string("inject"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.params["lube_type"].get<std::string>(),
              std::string("grease"));
    EXPECT_NEAR(out.signature.params["volume_ml"].get<double>(), 12.5, 1e-6);
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillLubrication, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::lubrication::Input in;
    in.lube_type          = "dry-film";
    in.volume_ml          = 2.0;
    in.application_method = "spray";
    auto out = skill::lubrication::apply(*stock, in);

    auto cands = skill::lubrication::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["lube_type"].get<std::string>(),
              std::string("dry-film"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::lubrication::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: unrecognised lube_type / method emit info, no block ────
TEST(SkillLubrication, UnknownTypeAndMethodAreInfoOnly)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::lubrication::Input in;
    in.lube_type          = "moly-paste";       // not in known set
    in.volume_ml          = 5.0;
    in.application_method = "swab";             // not in known set

    auto r = skill::lubrication::validate(*stock, in);
    EXPECT_TRUE(r.passed) << "unknown enums should be info-only, not block";
    EXPECT_TRUE(hasFinding(r, "DFM-LUBE-TYPE"));
    EXPECT_TRUE(hasFinding(r, "DFM-LUBE-METH"));
    EXPECT_NO_THROW(skill::lubrication::apply(*stock, in));
}
