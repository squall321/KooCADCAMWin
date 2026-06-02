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

constexpr std::array<DashSpec, 11> kAs568 {{
    // key      ID     CS    depth  width  inset
    { "-006",   2.90,  1.78, 1.34,  2.49,  0.15 },
    { "-011",   7.65,  1.78, 1.34,  2.49,  0.15 },
    { "-016",  15.60,  1.78, 1.34,  2.49,  0.15 },
    { "-111",  10.78,  2.62, 1.97,  3.67,  0.20 },
    { "-114",  17.13,  2.62, 1.97,  3.67,  0.20 },
    { "-116",  20.30,  2.62, 1.97,  3.67,  0.20 },
    { "-212",  21.95,  3.53, 2.65,  4.94,  0.25 },
    { "-224",  47.22,  3.53, 2.65,  4.94,  0.25 },
    { "-325",  37.47,  5.34, 4.00,  7.48,  0.40 },
    { "-425",  88.27,  6.99, 5.24,  9.78,  0.50 },
    { "-908",   4.69,  1.27, 0.95,  1.78,  0.10 },
}};

inline const DashSpec* findDash(const std::string& dash_key) {
    for (const auto& d : kAs568) if (dash_key == d.dash_key) return &d;
    return nullptr;
}

}  // namespace
