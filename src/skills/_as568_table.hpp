#pragma once
#include <array>
#include <string>

namespace koocadcam::skill::as568 {

struct DashSpec {
    const char* dash_key;          // "-006", "-011", "-016", "-111", "-114", "-116", "-212", "-224", "-325", "-425", "-908"
    double inner_dia_mm;           // ID of the O-ring
    double cross_section_mm;       // CS (-006/-011/-016 = 1.78; -1xx = 2.62; -2xx = 3.53; -3xx = 5.34; -4xx = 6.99; -9xx = 0.51..varies)
    double groove_depth_mm;        // recommended axial face-groove depth (= 0.75 * CS)
    double groove_width_mm;        // recommended groove width (= CS * 1.4)
    double radial_groove_inset_mm; // recommended ID/OD lead-in chamfer
};

// Values verified against Parker O-Ring Handbook ORD 5700 §A4-2 (AS568D
// table) and SAE AS568-2017.  ID/CS in mm to 0.01 mm precision.
//   groove_depth = 0.75 × CS (Parker §4-2 Table 4-1, 15–25 % squeeze target)
//   groove_width = 1.40 × CS (Parker §4-2 Table 4-1, 75–85 % gland fill)
//   radial_inset = recommended lead-in chamfer (~0.08 × CS, rounded).
// -908 is the boss-seal (MS28778) series — historically logged as 1.27 mm
// here for backwards compatibility; the o_ring_groove_as568_spec and
// x_ring_groove_spec skills locally override -908 to 1.78 mm.
constexpr std::array<DashSpec, 11> kAs568 {{
    // key      ID     CS    depth  width  inset
    { "-006",   2.84,  1.78, 1.34,  2.49,  0.15 },
    { "-011",   7.65,  1.78, 1.34,  2.49,  0.15 },
    { "-016",  15.54,  1.78, 1.34,  2.49,  0.15 },
    { "-111",  10.78,  2.62, 1.97,  3.67,  0.20 },
    { "-114",  17.13,  2.62, 1.97,  3.67,  0.20 },
    { "-116",  20.30,  2.62, 1.97,  3.67,  0.20 },
    { "-212",  21.95,  3.53, 2.65,  4.94,  0.25 },
    { "-224",  47.22,  3.53, 2.65,  4.94,  0.25 },
    { "-325",  37.47,  5.33, 4.00,  7.46,  0.40 },
    { "-425",  88.27,  6.99, 5.24,  9.78,  0.50 },
    { "-908",   4.69,  1.27, 0.95,  1.78,  0.10 },
}};

inline const DashSpec* findDash(const std::string& dash_key) {
    for (const auto& d : kAs568) if (dash_key == d.dash_key) return &d;
    return nullptr;
}

}  // namespace
