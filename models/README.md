# 3D / CAD Models

This folder is reserved for the 3D-printable and CAD files of the chassis (WRO convention:
`models/` = mechanical models, not neural networks).

**Status: TODO.** Add STL files (for printing) and STEP / native CAD files (for editing), one
sub-folder per assembly, e.g.

```text
models/
├── chassis/        base plate, top deck
├── steering/       servo mount, linkage parts
├── drivetrain/     motor mount, gear carrier
└── sensor-mounts/  ToF brackets, camera mount, colour-sensor holder
```

If the chassis is built from LEGO Technic, add a parts list (or an LDraw / Studio file)
instead.

The trained neural networks are **not** stored here. They are published as GitHub Release
assets (see [`src/README.md`](../src/README.md#trained-models)).
