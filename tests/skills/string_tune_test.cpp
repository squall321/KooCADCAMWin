// @lat: [[process/test-strategy#skill round-trip]]
//
// string_tune skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of mismatched freq list size, metadata-only recognition,
// and a skill-specific assertion that the mean frequency is recorded.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/string_tune.hpp"

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

skill::string_tune::Input goodInput()
{
    skill::string_tune::Input in;
    in.string_count = 6;
    in.target_freq_hz_per_string =
        { 82.41, 110.00, 146.83, 196.00, 246.94, 329.63 };
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillStringTune, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::string_tune::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("string_tune"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (size mismatch + out-of-range freq) ───
TEST(SkillStringTune, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    // Size mismatch: 6 strings but only 5 freqs supplied.
    auto in = goodInput();
    in.target_freq_hz_per_string.pop_back();
    auto r = skill::string_tune::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-TUNE-COUNT"));
    EXPECT_THROW(skill::string_tune::apply(*stock, in), skill::SkillError);

    // Out-of-range frequency.
    auto in2 = goodInput();
    in2.target_freq_hz_per_string[0] = 5.0;   // below 16 Hz
    auto r2 = skill::string_tune::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-TUNE-FREQ"));
}

// ─── 3. Signature records kind + key params (list preserved) ─────────────
TEST(SkillStringTune, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.string_count = 4;
    in.target_freq_hz_per_string = { 196.00, 293.66, 440.00, 659.25 };  // violin

    auto out = skill::string_tune::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("string_tune"));
    EXPECT_EQ(out.signature.pattern["string_count"].get<int>(), 4);
    EXPECT_EQ(out.signature.pattern["target_freq_hz_per_string"].size(), 4u);
    EXPECT_NEAR(
        out.signature.pattern["target_freq_hz_per_string"][2].get<double>(),
        440.00, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillStringTune, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::string_tune::apply(*stock, goodInput());

    auto cands = skill::string_tune::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["string_count"].get<int>(), 6);
    EXPECT_EQ(cands[0].recovered_params["target_freq_hz_per_string"].size(),
              6u);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::string_tune::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "string_tune cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: mean_freq_hz pattern field matches arithmetic mean ─────
TEST(SkillStringTune, MeanFrequencyRecorded)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::string_tune::apply(*stock, goodInput());

    const auto& freqs = goodInput().target_freq_hz_per_string;
    double sum = 0.0;
    for (double f : freqs) sum += f;
    const double expectedMean = sum / static_cast<double>(freqs.size());

    EXPECT_NEAR(out.signature.pattern["mean_freq_hz"].get<double>(),
                expectedMean, 1e-6);
}
