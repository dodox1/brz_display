#pragma once

// Piecewise linear interpolation for expected oil pressure [kPa] based on rpm
// 13 points (based on real data 93–94°C)

struct OilPressurePoint {
    int rpm;
    int pressure;
};

static const OilPressurePoint oilPressureTable[] = {
    { 600, 79 },
    { 900, 121 },
    { 1200, 184 },
    { 1500, 203 },
    { 2000, 250 },
    { 2500, 294 },
    { 3000, 369 },
    { 3500, 422 },
    { 4000, 458 },
    { 5000, 484 },
    { 6000, 477 },
    { 7100, 466 },
    { 7400, 460 }
};
static const int oilPressureTableSize = sizeof(oilPressureTable) / sizeof(oilPressureTable[0]);

// Return expected oil pressure [kPa] for given rpm at 93–94°C oil temp
inline int expectedOilPressure(int rpm) {
    // Clamp if out of data bounds
    if (rpm <= oilPressureTable[0].rpm) return oilPressureTable[0].pressure;
    if (rpm >= oilPressureTable[oilPressureTableSize - 1].rpm) return oilPressureTable[oilPressureTableSize - 1].pressure;

    // Find segment for interpolation
    for (int i = 1; i < oilPressureTableSize; ++i) {
        if (rpm < oilPressureTable[i].rpm) {
            int rpm0 = oilPressureTable[i-1].rpm;
            int rpm1 = oilPressureTable[i].rpm;
            int p0 = oilPressureTable[i-1].pressure;
            int p1 = oilPressureTable[i].pressure;
            // Linear interpolation
            int pressure = p0 + ((int64_t)(rpm - rpm0) * (p1 - p0)) / (rpm1 - rpm0);
            return pressure;
        }
    }
    // fallback (should not happen)
    return oilPressureTable[oilPressureTableSize - 1].pressure;
}
