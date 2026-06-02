#pragma once
#include <array>

namespace koocadcam::skill::iso286 {

// Per ISO 286-1: nominal-size bands and tolerance grades.
// Hole-basis (upper-case letter = HOLE) deviations in micrometers.
//
// H grades (H6/H7/H11): upper = +h_dev_um, lower = 0. Tolerance = h_dev_um.
// G6 (hole): lower = +g6_lower_um (positive!), upper = lower + h6_dev_um.
// P7 (hole): upper = -p7_upper_um (negative), lower = upper - h7_dev_um.
struct FitBand {
    double size_max_mm;      // upper bound of band
    int    h6_dev_um;        // H6 IT (also upper-dev when lower=0)
    int    h7_dev_um;        // H7 IT
    int    h11_dev_um;       // H11 IT
    int    g6_lower_um;      // G6 hole EI (LOWER deviation, positive per ISO 286)
    int    p7_upper_um;      // P7 hole ES (UPPER deviation, negative per ISO 286)
};

constexpr std::array<FitBand, 7> kFits {{
    // up_to  H6   H7   H11   G6_EI  P7_ES
    {   6.0,  8,  12,   75,    +4,   -8 },
    {  10.0,  9,  15,   90,    +5,   -9 },
    {  18.0, 11,  18,  110,    +6,  -11 },
    {  30.0, 13,  21,  130,    +7,  -14 },
    {  50.0, 16,  25,  160,    +9,  -17 },
    {  80.0, 19,  30,  190,   +10,  -21 },
    { 100.0, 22,  35,  220,   +12,  -24 },
}};

inline const FitBand* findBand(double nominal_mm) {
    for (const auto& b : kFits) if (nominal_mm <= b.size_max_mm) return &b;
    return nullptr;
}

// H7 hole: lower=0, upper = +IT7.
inline double h7_max_mm(double nominal_mm) {
    const FitBand* b = findBand(nominal_mm);
    return b ? nominal_mm + b->h7_dev_um * 1e-3 : nominal_mm;
}
// H6 hole: lower=0, upper = +IT6.
inline double h6_max_mm(double nominal_mm) {
    const FitBand* b = findBand(nominal_mm);
    return b ? nominal_mm + b->h6_dev_um * 1e-3 : nominal_mm;
}
// P7 hole: upper = -P7_ES (negative), lower = upper - IT7.
inline double p7_max_mm(double nominal_mm) {
    const FitBand* b = findBand(nominal_mm);
    return b ? nominal_mm + b->p7_upper_um * 1e-3 : nominal_mm;
}
inline double p7_min_mm(double nominal_mm) {
    const FitBand* b = findBand(nominal_mm);
    return b ? nominal_mm + (b->p7_upper_um - b->h7_dev_um) * 1e-3 : nominal_mm;
}
// G6 hole: lower = +G6_EI (positive), upper = lower + IT6.
inline double g6_min_mm(double nominal_mm) {
    const FitBand* b = findBand(nominal_mm);
    return b ? nominal_mm + b->g6_lower_um * 1e-3 : nominal_mm;
}
inline double g6_max_mm(double nominal_mm) {
    const FitBand* b = findBand(nominal_mm);
    return b ? nominal_mm + (b->g6_lower_um + b->h6_dev_um) * 1e-3 : nominal_mm;
}

}  // namespace
