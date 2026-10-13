# Executed validation report

These are executed CoppeliaSim tests with the delivered MATLAB controller, not a predicted trajectory or illustrative animation. Cylinder and box runs used physical support plates at centre Z=0.060 m. The primary sphere rested directly on the floor at centre Z=0.025 m (radius 0.025 m).

## Results

| Case | Final centre error (mm) | Measured lift (mm) | Peak sealed cups | Sampled hand/support clearance (mm) | Result |
|---|---:|---:|---:|---:|---|
| Floor sphere (recorded) | 7.323 | 100.03 | 20 | 6.443 | Pass |
| Upright cylinder | 0.352 | 99.99 | 7 | 21.854 | Pass |
| Box | 0.466 | 99.98 | 7 | 4.354 | Pass |

Primary coordinates: source [0.42,-0.08,0.025], destination [0.35,0.18,0.025]. Cylinder/box use the same X/Y with Z=0.060. All coordinates are object centres in world metres. Other X/Y pairs are accepted by IK but are not covered by these completed tests.

## Checks applied

- The single scene was reloaded before the three-shape sequence; handles rebound by aliases.
- Every logged payload parent remained world (-1). Runtime code never sets payload pose or parent. Native dynamics integrate gravity, contact and the applied bounded forces/torques.
- Every case processed all 23 finger/thumb flexion joints. Some joints correctly stopped at zero angle when a downstream module already contacted the object; every module need not contact every shape.
- Each case lifted more than 50 mm, completed carry, vented all cups, cleared digits in order 1–5, and finished with centre error below 10 mm and speed below 10 mm/s.
- Setdown confirmed physical payload/support distance at or below 0.15 mm before preload and sequential release; no payload pose correction was used.
- All 100 g masses and analytical inertia tensors were read back through getShapeMass/getShapeInertia and asserted against the requested values. The simulation used a 5 ms step, ten steps per viewport rendering.
- Maximum tensile cup force was checked against 20 kPa × π × (3 mm)² = 0.56549 N. Compressive pad force has its separate 1 N cap.
- Preflight sampled 126 robot configurations against floor/supports and the open approach against the payload. This is sampled verification, not a continuous collision-free planning proof.
- The GIF combines native overview and close-up CoppeliaSim frames from the successful repeated floor-sphere run. Frame times come from the simulation log. It includes the final DONE frame.

## Interpretation and limits

The results validate this demonstrator for the tested shape/pose/coordinate combinations. They do not establish grasping of arbitrary reachable objects. The design uses discrete rigid finger modules and ideal joint servos, with independent payload dynamics.

Vacuum uses a bounded compliant force model, not resolved air flow. The assumed flexible cup lip can bridge up to 1.5 mm and 45° normal mismatch; lip deformation and airtight sealing are not resolved or experimentally validated. Support rolling resistance coefficient 0.02 is also a material assumption. Fragile-object damage, motor sizing, robot load response and medical usability are outside this validation. See README.md for full model limits and run instructions.

Machine-readable evidence is in logs/final_*.json and logs/sphere_recorded.json.

## Preflight coroutine regression

The preflight RPC was replaced with beginPaths/nextPathSample/cancelPaths. Every short Lua call temporarily suppresses automatic yielding and restores its previous setting. The 126-sample sphere preflight passed. Cancellation after seven samples and an injected invalid second-path sample both restored all saved arm/flexion joint positions within 1e-12 rad. The subsequent full sphere run passed and simulation was stopped before handoff. Evidence: logs/bounded_preflight_verified_sphere.json.


## Visible-motion and cylinder progress check

Preflight viewport drawing was disabled while the stopped-scene poses were sampled, then restored after cleanup. A normal visible cylinder run passed: measured lift 99.99 mm, final centre error 0.424 mm, seven peak seals and zero seals at completion. Viewport restoration and the new live-position/lift fields were asserted. The run took 175.4 wall seconds for 47.9 simulation seconds on this machine. Evidence: logs/cylinder_ui_verified.json.

## Invisible-copy preflight regression

The earlier viewport suppression did not prevent the reported empty-hand preview. Preflight now samples an invisible, nonrespondable copy of the arm/hand. All real arm, finger flexion, and thumb CMC positions were checked after every one of the 126 samples and stayed unchanged within 1e-12 rad. Minimum support clearance remained 6.443 mm. Normal completion, cancellation after seven samples, and an injected Lua error all removed every temporary clone. Evidence: logs/invisible_preflight_verified.json.


The subsequent normal visible sphere run passed with 100.3 mm measured lift, 12 peak seals, zero seals at completion, independent world-parent payload, and 4.00 mm final centre error. Evidence: logs/invisible_preflight_sphere_verified.json. The final placement was left visible in CoppeliaSim.


## Existing three-object selection workflow

The single scene now stages sphere [0.42,-0.20,0.025], cylinder [0.42,0,0.06], and box [0.32,0.22,0.06] in different reachable directions. run_octopus asks only for object identity and destination. octo_pick_existing reads the actual world source pose, preserves other objects, and retains completed-task object poses when preparing the next command. The selected payload is not repositioned during preparation.

All three runs passed with the other two objects present. Preparation assertions checked that all payload centres remained unchanged; unselected objects stayed within 2 mm of their initial centres during the run. Cylinder and box stopping tests additionally asserted retained final positions; the helper verifies exact restoration to its captured centre. Invisible preflight also checks proximity to unselected objects and the additional support plates.

| Object | Measured lift (mm) | Final error (mm) | Final seals |
|---|---:|---:|---:|
| Sphere | 100.16 | 1.13 | 0 |
| Cylinder | 100.05 | 0.92 | 0 |
| Box | 99.36 | 0.85 | 0 |

The tested destination was [0.48,0.16,0.025] for sphere and [0.48,0.16,0.06] for cylinder/box, with an explicit initial-layout reset between individual tests. All payloads remained world-parented. Evidence: logs/existing_sphere_verified.json, logs/existing_cylinder_verified.json, logs/existing_box_verified.json, and logs/three_object_summary.json. Scene reload and the reset command passed with all three independent objects at their staged positions; an idle job was saved. The final initial layout is left visible.

Selection uses known scene aliases and simulator poses; camera perception and arbitrary-mesh classification are not implemented. Coordinate-demo octo_pick_place remains an explicit staging API. Normal use should go through run_octopus or octo_pick_existing.


## North / south / east layout

The saved layout now places the sphere north of the base at [0,0.42,0.025], cylinder south at [0,-0.42,0.06], and box east at [0.42,0,0.06]. Directions refer to world +Y, -Y, and +X respectively. All three physical pickup/carry/release tests passed, with the unselected objects remaining within 2 mm of their staged positions. The layout was reset explicitly between tests; object selection itself reads the actual source and preserves the other payloads.

| Direction / object | Lift (mm) | Final error (mm) | Final seals |
|---|---:|---:|---:|
| North / sphere | 100.00 | 4.81 | 0 |
| South / cylinder | 100.28 | 1.37 | 0 |
| East / box | 100.11 | 0.85 | 0 |

Sphere was placed at [0.35,0.20,0.025], cylinder at [0.35,-0.20,0.06], and box at [0.35,0.20,0.06] in the independent trials. Evidence: logs/compass_*_verified.json and logs/compass_summary.json. The scene is saved stopped with all three objects in the compass layout. reset_view now fits the explicitly listed arm/hand shapes plus all payloads. Native viewport inspection confirmed the complete arm and all three objects are visible.


## Interactive function syntax repair

Corrected the split negative literal in run_octopus: exampleY=- followed by .20 on a new line caused Invalid expression at line 12, column 51. The cylinder example now uses an ordinary multiline if block with exampleY=-.20 on one line. MATLAB checkcode reports no syntax errors and nargin('run_octopus') successfully resolves the function. No simulation was started by this syntax verification.


## Per-object start and destination input

run_octopus now accepts an optional start coordinate and a required destination for the selected existing payload. A blank start retains its actual scene location. Direct calls use run_octopus(kind,start,destination,options). octo_set_start stages only the selected payload while stopped, checks endpoint IK/floor heights and separation from other payloads, and saves its destination metadata in the same scene.

MATLAB parsing checks passed for both new functions. Native staging of sphere [0,0.40,0.025], cylinder [0,-0.40,0.06], and box [0.40,0,0.06] asserted that each selected start was applied exactly and unselected payload centres were unchanged. An unreachable start [2,0,0.025] was rejected without changing the sphere. The full direct run_octopus input path was exercised in prepareOnly mode for sphere, destination [0.33,0.20,0.025]: all 126 native clearance samples passed, minimum support clearance 6.443 mm, and job source/destination matched the requested inputs. This check did not repeat a physical grasp; the existing verified physics controller is unchanged. Evidence: logs/custom_start_end_verified.json and logs/custom_start_end_console.txt. The scene is left stopped with the customized starts and an idle job.


## Irregular convex object west of the base

Added TaskIrregular, a green asymmetric convex pebble mesh with 1,224 outward surface facets, initially at [-0.40,0,0.06]. The mesh is an independent dynamic world-parented 100 g body. Native convex mass/inertia computation passed; the full tensor and computed centre of mass are retained. Suction normals use the actual convex mesh support planes transformed with the payload pose, rather than a sphere or box formula. J1 and J6 simulated yaw intervals/planning limits were extended to +/-180 degrees to reach west; pitch limits remain unchanged.

The first physical trial lifted and carried the object but refused release because fixed 0.8 mm setdown preload would violate the 2 mm hand clearance margin. Setdown now limits additional preload once actual support contact has been confirmed, retains the safe pose, checks support contact before venting, and waits 0.3 s for settling. It never moves or reparents the payload. The retest passed: measured lift 100.03 mm, 16 peak seals, zero final seals, placement error 0.703 mm, parent -1, mass 0.1 kg. The limited-preload branch was exercised. Tested destination: [-0.32,0.18,0.06]. Minimum sampled support clearance: 7.304 mm.

Evidence: logs/irregular_west_verified.json, logs/irregular_summary.json, and logs/irregular_release_console.txt. Scene reload verified the fourth dynamic body at its west starting position, mass, independent parent and all original object aliases. The saved single scene embeds the updated runtime; camera framing includes all four payload handles. This validates this convex irregular mesh and tested pose, not arbitrary concave meshes, air flow, or real cup airtightness.



## Concave challenge test � 2026-10-09

Red `extreme` payload: six abutting blocks, asymmetric protrusions and concave recesses,
66 x 64 x 48 mm, 100 g. Source [-0.30,0.30,0.06] m; destination [-0.18,0.32,0.06] m.

- Completed physical pickup, carry, supported setdown, sequential release and retreat.
- Four sealed cups; measured peak lift 100.031 mm.
- Final placement error 0.122 mm; zero seals and world parent -1 at completion.
- 126 preflight samples passed; minimum support clearance 12.443 mm.
- A 2 mm probe in a recess had 11.810 mm surface clearance, confirming the recess
  remains physically empty rather than being filled by a convex hull.
- Initial approach failed the support-clearance check. Raising the open-hand
  approach 12 mm for this shape resolved that collision without relaxing any limit.
- Native component mass/inertia computed before grouping; compound full inertia and
  off-centre COM retained by MATLAB. Forces and limits unchanged from other objects.

This is one successful pose/trajectory for a concave object with flat seal patches.
It does not establish arbitrary-shape handling or physical suction airtightness.
Results: logs/extreme_verified.json and logs/extreme_summary.json.
The saved scene restores this object northwest, preserves the other objects,
and is stopped with no pending job.
