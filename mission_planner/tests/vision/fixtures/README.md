# Vision regression fixtures

`panorama.json` and the thirty `panorama-*.jpg` views add photographed full-360-degree
camera rotation and return tests for both Object and Area. Source: Greg Zaal's
[Courtyard](https://polyhaven.com/a/courtyard), photographed 2015-12-09, published
2018-04-04, [CC0](https://polyhaven.com/license). The 8192x4096 tonemapped source is
pinned in `scripts/simulation/assets.json`; SHA256
`b11be77cdeca832d44d5067521444feac410d062c476ba4a95666afb4dfa758e`.
`scripts/simulation/panorama.py` uses inverse equirectangular sampling; independent
forward projection of immutable rays supplies per-frame point/quad truth. Views use
640x480,90-degree horizontal FOV and fixed JPEG quality95. The selection and complete
pose schedule were fixed before testing. Return includes changed yaw/pitch/roll before
the exact original view. Tests check image hashes, wrong acceptance, recovery and
sustained positive coverage without reselection. This is camera rotation about one
center, not moving-object, parallax, motion-blur or capture-latency qualification.

The rendered Area images described below were generated locally by the owned simulation harness, with seed173 and
the unchanged procedural ground texture. They contain no operator camera or telemetry data.
The source ground-pattern generator is first-party; rendered vehicle meshes are outside
the visible image. Keep the original failed evidence under ignored test storage.

Reference capture97 and pan captures221,225,229 come from the same fresh fixed-body Iris
run. Camera motion was driven by actual MP native mount commands, ArduPilot servo outputs
and bounded simulated gimbal joints. The original tracker lost distributed selected-region
support during the pan despite a strong global planar fit.

`area-ground.json` records the source pins, selection, hashes and independently projected
canonical points from actual renderer poses and the horizontal plane. These points belong
only to the test oracle. No reference reseeding or matching-threshold changes are permitted
to rescue this regression; existing ambiguity, foreground and appearance tests also apply.

`area-natural.json` and its five PNGs retain the later natural-grass/compression regression.
Frames 8 and 22 have the original pose, while 40 and 62 have a small native-commanded pan.
The old tracker alternated between verification and Searching even with a correct strong
ground-plane fit. The point oracle uses actual camera poses and bounds equivalent capture
sets; it does not infer exposure time or feed poses into tracking. The full failed run is
retained under ignored evidence. Tests initialize once and never reseed the reference.

The natural scene uses Rob Tuytel's [Aerial Grass Rock](https://polyhaven.com/a/aerial_grass_rock)
texture, licensed [CC0](https://polyhaven.com/license), plus OpenRobotics'
[Hatchback v3](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hatchback/3/files) and
[Construction Barrel v4](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction%20Barrel/4/files),
both CC0. Exact archive/texture hashes are in `scripts/simulation/assets.json`. These
rendered fixtures contain no real operator imagery; the separately downloaded public
TMT video remains ignored because no redistribution grant was located.

`vehicle-chart.json` manually annotates the visible gray car in the existing synthetic
`vehicle.png`. Detector-route calibration renders that image on an emissive display
without a floor intersecting it. This is a transport/inference check, with other cars
unscored; it establishes neither outdoor semantic accuracy nor a real-camera result.
