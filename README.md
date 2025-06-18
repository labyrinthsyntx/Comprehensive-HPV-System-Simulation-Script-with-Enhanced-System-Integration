# Comprehensive HPV System Simulation  
**Filename:** `HPV_System_Simulation.m`  
**Authors:** Derek Martinez & Tim Yip  
**Initial release:** 19 September 2024  
**Last update:** 17 June 2025  

---

## Purpose
This MATLAB® script couples mechanical, electrical, and safety models for a Human-Powered Vehicle (HPV). It produces an integrated picture of performance, energy demand, braking capability, and structural stress, supported by interactive plots.

---

## Main Features
| Domain           | Key Capabilities |
|------------------|------------------|
| **Mechanical**   | • Steering‐induced force estimates  <br>• Turn-radius limits vs. frame clearance  <br>• Bending stress check against material limits |
| **Electrical**   | • Peukert-adjusted battery capacity  <br>• Peak and range speeds from combined human + motor power  <br>• Runtime prediction vs. mechanical drag |
| **Safety**       | • Braking force split (front/rear)  <br>• Stopping-distance check vs. available friction  <br>• Regenerative-energy recovery |
| **System Links** | • Mechanical-resistance sweep updates every energy, speed, and stress metric  <br>• Regenerated energy fed back into battery-capacity plots |
| **Visualization**| • Nine auto-generated figures (power curves, speed bands, braking maps, stress envelopes, etc.) with shaded safe/unsafe zones and annotated extrema |

All numeric constants are plain at the top of the script for quick tuning.

---

## Quick-Start
1. **Requirements**  
   * MATLAB R2022a + (core functions only)  
   * No extra toolboxes needed

2. **Run**  
   ```matlab
   >> HPV_System_Simulation
The console prints tabulated results, and figures open automatically.

Typical Runtime
• <1 s on a modern laptop (numerical root-finders dominate).
• Plots render progressively; close any window to skip its creation.

File Structure
HPV_System_Simulation.m         — primary script
README.md                       — this file

No external data files are required.

Customisation Tips
Task	Where
Change geometry, mass, or battery specs	“Configuration Parameters” section
Add a new plot	“Enhanced Figures…” block; append after existing examples
Tighten solver tolerances	Inside the anonymous functions that feed fzero

Output Overview
Console blocks:

• Mechanical Analysis Results – forces, radii, bending stress
• Electrical Analysis Results – battery, runtime, speed
• Safety Analysis Results – braking, friction margin
• Adjusted Results – full sweep vs. Mechanical-Resistance Factor (MRF)

Figures (1–9):

1. Power Required vs. Speed (MRF sweep)
2. MRF vs. Maximum Speed
3. Regenerative Energy vs. MRF
4. Battery Capacity vs. MRF
5. Braking Force vs. MRF (unsafe zone shaded)
6. Bending Stress vs. MRF (limit lines)
7. Runtime vs. MRF (threshold shaded)
8. Efficiency vs. Speed (zones)
9. Total Energy vs. Distance for three representative speeds

Version History
Date (YYYY-MM-DD)	Notes
2024-09-19	First public draft
2025-06-17	Added MRF sweep, expanded visualisations, improved comments

Martinez, D., & Yip, T. (2024, September 19). Comprehensive HPV system simulation script with enhanced system integration (MATLAB code, Version 2025-06-17) [Computer software]
