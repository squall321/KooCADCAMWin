// @lat: [[process/test-strategy#skill round-trip]]
//
// dichroic_coat skill — dichroic mirror multilayer (metadata-only).
// 5 cases:
//   1. apply clones geometry unchanged
//   2. validate rejects cutoff outside [200, 14000] nm
//   3. signature records kind + cutoff + transmission_band + reflection_band
//   4. recognize via metadata replay
//   5. specific: longpass filter_kind correctly classified

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/dichroic_coat.hpp"

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

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillDichroicCoat, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::dichroic_coat::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cutoff_wavelength_nm  = 500.0;
    in.transmission_min_nm   = 500.0;
    in.transmission_max_nm   = 700.0;
    in.reflection_min_nm     = 380.0;
    in.reflection_max_nm     = 500.0;

    auto out = skill::dichroic_coat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("dichroic_coat"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. ValidateRejectsBadInput — cutoff out of window ───────────────────
TEST(SkillDichroicCoat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::dichroic_coat::Input bad;
    bad.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.cutoff_wavelength_nm = 50.0;          // below 200 nm window
    bad.transmission_min_nm  = 60.0;
    bad.transmission_max_nm  = 100.0;
    bad.reflection_min_nm    = 10.0;
    bad.reflection_max_nm    = 50.0;

    auto r = skill::dichroic_coat::validate(*stock, bad);
    EXPECT_FALSE(r.passed);

    bool foundCutoff = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DICHROIC-CUTOFF" && f.severity == "error") {
            foundCutoff = true; break;
        }
    EXPECT_TRUE(foundCutoff);
    EXPECT_THROW(skill::dichroic_coat::apply(*stock, bad), skill::SkillError);
}

// ─── 3. SignatureRecordsKind + key params ────────────────────────────────
TEST(SkillDichroicCoat, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::dichroic_coat::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cutoff_wavelength_nm  = 650.0;
    in.transmission_min_nm   = 650.0;
    in.transmission_max_nm   = 900.0;
    in.reflection_min_nm     = 400.0;
    in.reflection_max_nm     = 650.0;

    auto out = skill::dichroic_coat::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("dichroic_coat"));
    EXPECT_NEAR(out.signature.pattern["cutoff_wavelength_nm"].get<double>(),
                650.0, 1e-9);

    ASSERT_TRUE(out.signature.pattern.contains("transmission_band_nm"));
    auto tb = out.signature.pattern["transmission_band_nm"];
    ASSERT_EQ(tb.size(), 2u);
    EXPECT_NEAR(tb[0].get<double>(), 650.0, 1e-9);
    EXPECT_NEAR(tb[1].get<double>(), 900.0, 1e-9);

    ASSERT_TRUE(out.signature.pattern.contains("reflection_band_nm"));
    auto rb = out.signature.pattern["reflection_band_nm"];
    ASSERT_EQ(rb.size(), 2u);
    EXPECT_NEAR(rb[0].get<double>(), 400.0, 1e-9);
    EXPECT_NEAR(rb[1].get<double>(), 650.0, 1e-9);
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillDichroicCoat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::dichroic_coat::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cutoff_wavelength_nm  = 500.0;
    in.transmission_min_nm   = 500.0;
    in.transmission_max_nm   = 700.0;
    in.reflection_min_nm     = 380.0;
    in.reflection_max_nm     = 500.0;

    auto out = skill::dichroic_coat::apply(*stock, in);

    auto cands = skill::dichroic_coat::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(
        cands[0].recovered_params["cutoff_wavelength_nm"].get<double>(),
        500.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::dichroic_coat::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "dichroic_coat cannot be recognized geometrically";
}

// ─── 5. Specific: longpass filter_kind ───────────────────────────────────
TEST(SkillDichroicCoat, LongpassFilterKindClassified)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    // Reflect short λ, transmit long λ ⇒ longpass.
    skill::dichroic_coat::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cutoff_wavelength_nm  = 500.0;
    in.transmission_min_nm   = 510.0;
    in.transmission_max_nm   = 900.0;
    in.reflection_min_nm     = 380.0;
    in.reflection_max_nm     = 490.0;     // entirely below transmission band

    auto out = skill::dichroic_coat::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["filter_kind"].get<std::string>(),
              std::string("longpass"));
}
