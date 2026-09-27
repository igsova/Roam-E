# R17 — lower shell, rounded drawer and power-control mounts

Mechanical prototype based on the preserved R16 assembly. The upper robot is
unchanged. Read the [assembly and fit-test guide](ASSEMBLY_R17.md) before printing.

- [Complete Blender assembly](roam_lower_controls_r17.blend)
- [Replacement parts and fit tests](ROAM_R17_lower_replacement.zip)
- [Complete integrated STL pack](ROAM_STL_R17_Lower_Controls_K1C.zip)
- [Power-switch limitations and Pi 5 auto-boot](POWER_NOTES.md)
- [Geometry validation](validation.json)

The lower shell is now three bolted sections with PETG nut retention. The
curved drawer and fixed right panel each carry a modeled round rocker switch.
Replace P16A/P16B with P16C/P16D/P16B, and replace P05 with both matching P06
guides. Test the small coupons first: **Ø20.5 mm holes and 2.2 mm snap seats
are provisional until checked with the actual purchased switch.**

![Lower assembly and both controls](lower_assembled.png)

![Part separation — illustration, not an assembly trajectory](lower_exploded.png)

The supplied switch ratings are AC-only. The mounting model does not approve
direct DC battery/motor switching or select wiring/fusing. No Pi setting,
firmware or electrical connection was changed. The Pi 5 normally boots when
power is applied; see the linked power notes.
