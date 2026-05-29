// @lat: [[process/test-strategy#skill round-trip]]
//
// spectro_measure — instrumental spectroscopic measurement record.
// Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndKeyParams
//   4. RecognizeMetadataReplay
//   5. IrInstrumentSelectsKbrWindowMaterial

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/spectro_measure.hpp"

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

skill::spectro_measure::Input goodInput()
{
    skill::spectro_measure::Input in;
    in.instrument_type = "UV-Vis";
    in.wavelength_nm   = 254.0;
    in.scan_count      = 8;
    return in;
}

}  // namespace

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillSpectroMeasure, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 45.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    auto out = skill::spectro_measure::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("spectro_measure"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillSpectroMeasure, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 45.0);

    // Unknown instrument.
    auto badInst = goodInput();
    badInst.instrument_type = "NMR";
    {
        auto r = skill::spectro_measure::validate(*stock, badInst);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::spectro_measure::apply(*stock, badInst),
                 skill::SkillError);

    // Non-positive wavelength for an optical modality.
    auto badWl = goodInput();
    badWl.wavelength_nm = 0.0;
    {
        auto r = skill::spectro_measure::validate(*stock, badWl);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-SPECTRO-WL"));
    }
    EXPECT_THROW(skill::spectro_measure::apply(*stock, badWl),
                 skill::SkillError);

    // Zero scans.
    auto noScan = goodInput();
    noScan.scan_count = 0;
    {
        auto r = skill::spectro_measure::validate(*stock, noScan);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-SPECTRO-SCAN"));
    }
    EXPECT_THROW(skill::spectro_measure::apply(*stock, noScan),
                 skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillSpectroMeasure, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 45.0);

    auto in = goodInput();
    in.instrument_type = "Raman";
    in.wavelength_nm   = 785.0;
    in.scan_count      = 16;

    auto out = skill::spectro_measure::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("spectro_measure"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("spectro_measure"));
    EXPECT_EQ(out.signature.pattern["instrument_type"].get<std::string>(),
              std::string("Raman"));
    EXPECT_NEAR(out.signature.pattern["wavelength_nm"].get<double>(),
                785.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["scan_count"].get<int>(), 16);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSpectroMeasure, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 45.0);

    auto out   = skill::spectro_measure::apply(*stock, goodInput());
    auto cands = skill::spectro_measure::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["instrument_type"].get<std::string>(),
              std::string("UV-Vis"));
    EXPECT_NEAR(cands[0].recovered_params["wavelength_nm"].get<double>(),
                254.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::spectro_measure::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: IR modality selects KBr window cuvette material ─────────
TEST(SkillSpectroMeasure, IrInstrumentSelectsKbrWindowMaterial)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 45.0);

    auto irIn = goodInput();
    irIn.instrument_type = "IR";
    irIn.wavelength_nm   = 5000.0;   // mid-IR
    auto irOut = skill::spectro_measure::apply(*stock, irIn);
    EXPECT_EQ(irOut.signature.tooling.tool_type,
              std::string("ftir_spectrometer"));
    EXPECT_EQ(irOut.signature.tooling.tool_material,
              std::string("kbr_window"));

    auto uvIn = goodInput();
    auto uvOut = skill::spectro_measure::apply(*stock, uvIn);
    EXPECT_EQ(uvOut.signature.tooling.tool_material,
              std::string("quartz_cuvette"));
}
