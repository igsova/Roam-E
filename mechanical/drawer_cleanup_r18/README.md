# R18 — corrected curved battery drawer

**Replace only P05 relative to R17. Keep the R17 P06L/P06R guides, lower panels,
switch, M3×12 lock screws, nuts and washers.** R17 and earlier revisions are preserved.

- [Corrected drawer STL](ROAM_R18_drawer_replacement/01_rigid/P05_curved_battery_drawer_R18_qty1.stl)
- [Replacement ZIP with instructions and checks](ROAM_R18_drawer_replacement.zip)
- [Complete updated Blender assembly](roam_drawer_cleanup_r18.blend)
- [Drawer-only Blender inspection view](roam_drawer_cleanup_r18_inspection.blend)
- [Independent validation report](validation.json)

## Corrections

1. Rebuilt the lock-washer transitions with circular, finely tessellated,
   smoothly blended geometry. This changes the actual STL, not just shading.
   The flat 10 mm washer lands, 3.4 mm bores and bolt axes are retained.
2. Replaced the projecting rectangular finger-pull backing and old flat-strip
   remnants with a rounded, closed cup. Its rear projects 5.1 mm less into the
   drawer. The finger recess remains 34 × 9 mm with rounded corners and a
   nominal 2.4 mm closed back; it is not an opening into the wiring compartment.
3. Closed the unintended long wedge gaps between the tray floor and curved
   front. The new connection is a continuous 3 mm floor at the original height.
4. Added inside-only clearance around the washer transitions for the existing
   R17 guide tips. The washer lands retain 3.6 mm thickness; the locally relieved
   transition retains at least nominal 3.05 mm skin. No guide reprint is needed.

The four small strap slots and shallow underside caster-clearance grooves are
intentional and remain. The grooves retain 1.6 mm of floor, as in R17. The
battery position, liner, rails, overall 174 × 104 × 45 mm size, and switch
interface are not rescaled. All 891 other R17 scene objects remain unchanged.

![Corrected front](drawer_front.png)

![Rounded pull backing and continuous floor](drawer_inside.png)

![Closed front-floor junction from below](drawer_bottom.png)

## Printing and fitting

Import the R18 STL at **100%, millimetres**. Do not use the superseded P05 from
the R17 ZIP. This is a PETG mechanical prototype, not a support-free redesign:
the inherited floor is higher than the front's bottom lip, so inspect/support
the tray underside as well as the pull recess, switch counterbore and other
overhangs in the slicer. Keep the sliding rails and screw/switch interfaces clean.

The isolated K1C 0.4 mm / generic PETG / 0.20 mm slicing test uses normal snug
supports and a 4 mm brim. Its diagnostic project is **not a tuned print job**;
use your verified material profile and inspect the complete layer preview.
The CLI cannot render its OpenGL thumbnail on this host, but it exports
toolpaths and the diagnostic 3MF. That limitation does not certify print quality.

Disconnect the battery and external power before fitting. Test the empty
drawer in the **R17 guides** before installing the battery and wiring. Reuse
the two M3×12 locks; do not force a tight slide or overtighten the plastic.
If coming from R16 or earlier, obtain the matching guides from R17 first.

The switch opening is still the provisional Ø20.5 mm / 2.2 mm snap seat from
R17. Test the purchased switch using Q17C and your PETG profile. Physical
switch tolerances, wiring insulation, service slack and strain relief remain
to verify. This revision makes no electrical changes and does not approve
any switch for battery/motor DC current.

## Verification

The release package is gated on `validation.json` reporting **PASS** for this
exact Blender master and STL. Checks include:

- Single closed solids and STL reimport.
- 705 cross-sections through the old floor-gap region, on both master and STL.
- 305 new-floor thickness measurements, retained strap slots/caster grooves,
  closed pull backing, washer lands, fine blend fidelity and switch snap seat.
- Bidirectional tray/rail surface comparison against R17 (0.005 mm tolerance).
- Unchanged R17 mating objects, static interference and switch service space.
- Full-size drawer withdrawal sampled every 0.5 mm over 150 mm.

These are digital checks, not a physical fit, strength, PETG creep or continuous
collision guarantee. Unmodeled cables and actual printer tolerances still matter.

## Reproduce

From the repository root, run the build, independent validator, and render
scripts with Blender in background mode and `--disable-autoexec`:

1. `tools/cad/create_drawer_cleanup_r18.py`
2. `tools/cad/validate_drawer_cleanup_r18.py`
3. `tools/cad/render_drawer_cleanup_r18.py`

Then run `tools/cad/slice_drawer_cleanup_r18.py` with Python for the isolated
Creality Print check and `tools/cad/package_drawer_cleanup_r18.py` to package.
