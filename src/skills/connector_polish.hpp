#pragma once
// @lat: [[engine/skills#connector_polish]]
//
// connector_polish — multi-stage abrasive polish of a fiber-optic
// connector ferrule end-face (LC, SC, FC, etc.).  Determines the
// connector's reflection performance:
//
//   PC   (Physical Contact)        ~ −40 dB return loss
//   UPC  (Ultra-Physical Contact)  ~ −50 dB
//   APC  (Angled Physical Contact) ~ −60 dB, 8° endface angle
//
//      ┌──────────────┐
//      │  ferrule     │  ZrO2 cylinder, fiber bonded inside
//      │  ┌────────┐  │
//      │  │  fiber │  │
//      │  └────────┘  │
//      └──────╦───────┘   ← polish surface
//          ▼ ▼ ▼ ▼ ▼      (UPC: flat, APC: 8° tilt)
//          abrasive lapping film (5 µm → 1 µm → 0.3 µm → final)
//
// Signature pattern:
//   - pattern.kind             = "connector_polish"
//   - pattern.polish_class     = "UPC" | "APC" | "PC"
//   - pattern.return_loss_db   = <input> (NEGATIVE number; -60 better than -40)
//   - pattern.endface_angle_deg= 0.0 (UPC/PC) | 8.0 (APC)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT             return_loss_db ≥ 0                       (error)
//   DFM-POLISH-CLASS      polish_class ∉ {UPC, APC, PC}            (error)
//   DFM-POLISH-RL-WEAK    return_loss_db > −20 (too weak)          (error)
//   DFM-POLISH-RL-UPC     UPC + return_loss_db < −55 unrealistic   (info)
//   DFM-POLISH-RL-APC     APC + return_loss_db > −50 weak for APC  (info)
//
// Synthesis:
//   Records polish metadata.  The geometric endface is too fine to model
//   in OCCT B-rep; we keep the workpiece shape and stamp the signature.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace connector_polish {

constexpr const char* kSkillId = "connector_polish";

struct Input
{
    std::string  polish_class    = "UPC";    // "UPC" | "APC" | "PC"
    double       return_loss_db  = -50.0;    // negative: -60 better than -40
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace connector_polish
}  // namespace koocadcam::skill
