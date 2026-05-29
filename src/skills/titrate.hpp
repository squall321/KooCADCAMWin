#pragma once
// @lat: [[engine/skills#titrate]]
//
// titrate — record a volumetric titration: a known titrant is added to an
// analyte solution until an endpoint (pH, conductivity, indicator colour) is
// reached.  The workpiece (the analyte beaker / flask) is geometrically
// unchanged; the signature records analyte, titrant, target endpoint pH and
// volume of titrant consumed.
//
//        ┌─────────────┐                       ┌─────────────┐
//        │  analyte    │      titrate          │ titrated    │
//        │  (acid/base)│ ─────────────────────▶│ analyte     │
//        │             │  (analyte, titrant,   │  endpoint   │
//        │             │   endpoint, volume)   │  reached    │
//        └─────────────┘                       └─────────────┘
//          shape identical ; titration result stamped
//
// Signature pattern:
//   pattern.kind                = "titrate"
//   pattern.analyte             = string
//   pattern.titrant             = string
//   pattern.target_endpoint_ph  = double
//   pattern.volume_ml           = double
//   pattern.geometry_changed    = false
//
// DFM:
//   DFM-INPUT          analyte / titrant empty (error)
//   DFM-TITR-PH        target_endpoint_ph outside [0, 14] (error)
//   DFM-TITR-VOL       volume_ml <= 0 (error)
//   DFM-TITR-VOL-HIGH  volume_ml > 1000 (info — bench burette typically 50 mL)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace titrate {

constexpr const char* kSkillId = "titrate";

struct Input
{
    std::string  analyte             = "HCl";
    std::string  titrant             = "NaOH_0_1M";
    double       target_endpoint_ph  = 7.0;
    double       volume_ml           = 25.0;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace titrate
}  // namespace koocadcam::skill
