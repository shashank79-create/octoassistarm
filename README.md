# Octopus inspired robotic arm for handicapped person

One scene: `scene/OctoAssist.ttt`. This fresh project combines a six-axis arm with a five-finger, opposable-thumb end effector. Rounded articulated modules wrap independently; fixed miniature suction cups augment the grasp. The appearance is a palm with fingers, rather than free tentacles.

## Run

1. Open `scene/OctoAssist.ttt` in CoppeliaSim Edu. Leave simulation stopped. The ZeroMQ remote API must be available on localhost port 23000.
2. In MATLAB, change to this project's `matlab` directory.
3. Run `run_octopus`. Select `sphere`, `cylinder`, `box`, or `irregular`, then enter its start and destination centres in world metres. Leave start blank to retain its actual scene position. Only the selected payload is staged; other payloads remain in place.

```matlab
clear functions
run_octopus
% Or supply this object's start and end directly:
result = run_octopus('sphere', [0,.40,.025], [.33,.20,.025]);
% Pick from its current scene position:
result = octo_pick_existing('sphere', [.35,.20,.025]);
```

All four independent, dynamic 100 g objects remain visible in north, south, and east of the arm base:

| Object | Initial centre (metres) |
|---|---|
| 50 mm sphere | `[0,.42,.025]` (north / +Y) |
| Upright cylinder, diameter 50 mm, height 80 mm | `[0,-.42,.06]` (south / -Y) |
| 45 mm box | `[.42,0,.06]` (east / +X) |

`setup_three_objects` explicitly resets this initial arrangement. A nonempty start input stages only the selected object before simulation; a blank start reads its existing position. Unselected objects are not reset or parked. The existing-object workflow preserves their actual final poses when stopping a completed simulation to prepare the next task. If you manually click Stop, CoppeliaSim can restore its initial poses; the next command uses whatever positions are then present.

Suggested destinations: `[.35,.20,.025]` for the sphere, `[.35,-.20,.06]` for cylinder, and `[.35,.20,.06]` for box. Destinations must be clear of objects already placed there. Choose at least 0.12 m horizontal separation from other objects. Raised objects have their own physical support plates; a destination plate supports raised placement. Floor centre heights are 0.025 m (sphere), 0.040 m (cylinder), and 0.0225 m (box).

Location lookup uses known scene aliases and simulator world poses, not camera perception or automatic classification of arbitrary meshes. The supported grasp geometry remains sphere, upright cylinder, and box. During a run, gravity, contact, and bounded cup forces move the selected payload.

`octo_pick_place(source,destination,kind)` is retained as an explicit coordinate-driven staging/demo API; it repositions its source and parks unused exemplars. Use `octo_pick_existing` or `run_octopus` for the persistent three-object workflow.

A task finishes at `DONE` and leaves its final placement visible. Inspect `result.status.success`, `result.status.reason`, and `logs/latest_<type>.json`.

After replacing MATLAB files, use `clear functions` before starting `run_octopus` again. Preflight performs one configuration per short remote call and prints progress through all 126 samples. Its test poses use an invisible, nonrespondable temporary copy of the robot. The visible robot stays stationary during these clearance checks. The copy is removed on completion, cancellation, or error. Actual grasp/carry begins only after the checks pass. MATLAB names each real phase and periodically reports measured object centre, peak lift and sealed-cup count.

Each preflight Lua call temporarily disables automatic yielding and restores its previous setting, avoiding the coroutine error from the old long remote call. CoppeliaSim receives control between samples. A failure reports its Lua traceback; cleanup removes the temporary robot copy, script, and collection. Send the full `Preflight Lua error` message if a geometry check fails.

```matlab
% Optional fast verification with viewport drawing temporarily disabled:
result = octo_pick_place([.42,-.08,.025],[.35,.18,.025], ...
    'sphere', struct('fastTest',true));

% Optional actual CoppeliaSim image sequence:
result = octo_pick_place([.42,-.08,.025],[.35,.18,.025], ...
    'sphere', struct('recordPath',fullfile(pwd,'..','media','frames')));
```

`open_project` loads the authoritative scene. `build_project` rebuilds it from the fresh source; it replaces that project scene. `update_runtime` refreshes its controller without making a second scene.

The saved scene embeds its Lua controller and handle resolver. Object handles are rebound from unique aliases after loading; `scene_handles.json` is an informational export rather than a portable identifier list.

## Grasp and release

- Analytic arm IK maintains the horizontal palm orientation; quintic joint trajectories provide smooth arm motion. Six physical arm axes are present, although this task holds wrist roll at zero.
- The hand has 29 articulated DOF: four fingers with basal splay plus five flexion joints, and a thumb with two CMC axes plus three flexion joints. Each object's initial CMC posture stays within the modeled limits; 23 flexion joints then wrap sequentially.
- Wrapping stops joint motion at measured capsule/object surface clearance. Each module participates in the contact search; contact does not require every module to touch every shape.
- A cup seals only near the object surface with compatible surface normal and a settled host-link contact. Its upstream joints must have finished curling. It anchors a material point on the object; there is no payload parenting, welded constraint, or commanded payload trajectory.
- Air vacuum pressure ramps toward 20 kPa. Cup normal force is capped by pressure times cup area; tangential force is limited, and seals break under excess extension/shear. The resulting forces and torques act on the dynamic payload.
- A compressed cup also supplies a bounded elastic normal contact reaction (120 N/m, 1 N maximum), separate from vacuum tension. This supplies the physical pad reaction missing from a pull-only adhesion model. Physical mass/inertia are set and verified through the explicit shape-inertia API.
- Lift requires at least four seals and a modeled vertical force-capacity bound exceeding 1.25 times payload weight. Actual lift is checked before carry. Placement IK uses the measured hanging grasp offset after a short lift settling interval, allowing for compliance and asymmetric contact.
- Cup engagement is limited to one new seal per 0.2-second scan. Release vents and clears one digit at a time while the remaining sealed digits stabilize the supported object. Each digit clears with its proximal joint before its remaining joints open. The object settles under gravity before completion is reported.
- Before venting, setdown lowers the hand at 5 mm/s until measured payload/support contact, followed by 0.8 mm of compliant preload and a settling interval. A missing support within 5 mm or insufficient hand/floor clearance aborts release.

## Practical limits

This is a **reduced simulation model**, not fluid dynamics or a continuous soft-body model. Rounded rigid modules approximate finger flexibility. Cups are nonrespondable compliant force elements; host modules provide collision surfaces. The seal gate assumes a flexible lip accommodating up to 1.5 mm reach and 45 degrees of local normal mismatch. That lip deformation is not rendered or physically resolved. Cup airtightness, leakage, elastomer deformation, pneumatic plumbing, and tissue mechanics require separate validation; a passed simulation does not establish that a manufactured cup will seal.

The arm and hand use ideal kinematic joint servos. Payload dynamics are simulated, but joint motor torque limits and equal/opposite robot load response are not. Connected robot links exclude self-response; no comprehensive self-collision, obstacle, or injury-safe motion planner is provided. Preflight samples hand clearance against floor/support plates and checks the open approach against the selected payload; runtime repeats floor/support checks. It does not establish safety around arbitrary obstacles or users.

Reachability alone does not guarantee a grasp. The present fingers/cup angles favor the 50 mm sphere; seal-count checks refuse unsupported shapes or poses. Smooth release reduces impulsive motion but does not prove that fragile objects cannot be damaged. This is an engineering demonstrator, not a medical assistive device ready for human use.

The simulation time step is 5 ms. Objects start at rest using the physics engine's native sleeping state and wake under nonzero applied forces/contact. No object pose is controlled during the run.

Support contact includes a velocity-opposing rolling-resistance torque with coefficient 0.02. It is capped by a weight/radius bound and angular momentum per step. This approximates a lossy support surface and prevents perpetual ideal rolling after placement. Its material coefficient, like cup compliance, is a model assumption. Ten physics steps are grouped per viewport frame; the 5 ms physics resolution is retained.

## Source map

- `matlab/octo_config.m`: geometry, dimensions, limits and vacuum parameters.
- `matlab/octo_ik.m`: arm inverse kinematics.
- `matlab/octo_pick_place.m`: coordinates, preflight, task execution and logs.
- `runtime/build_scene.lua`: creates the single robot scene from native primitives.
- `runtime/preflight.lua`: sampled hand/support clearance.
- `runtime/octo_runtime.lua`: wrapping, suction forces, carry and sequential release.
- `geometry.json`, `scene_handles.json`: frozen numerical configuration and scene object handles.

The force API applies actual forces to a dynamic shape: [CoppeliaSim sim.addForceAndTorque](https://manual.coppeliarobotics.com/en/sim/simAddForceAndTorque.htm). The vacuum force bound follows [Schmalz holding-force guidance](https://www.schmalz.com/en-us/support/know-how/glossary/holding-force).

`reset_view` frames the complete arm/hand geometry and all five payloads without stopping a task.

`octo_set_start(kind,start,destination)` stages a single payload without picking it. It checks floor heights, IK endpoint limits, and horizontal separation before moving the payload. The task subsequently performs native path clearance checks. All initial staging happens while simulation is stopped; payload motion during pickup/carry/release remains physics-driven.

## Irregular object west of the base

A green, asymmetric convex pebble mesh is now a fourth independent dynamic 100 g payload at `[-.40,0,.06]`. Select `irregular` in `run_octopus`, or:

```matlab
run_octopus('irregular', [-.40,0,.06], [-.32,.18,.06])
```

Its mesh facets supply actual local surface normals for contact-gated suction. Mass, full inertia tensor, and centre of mass are computed from its convex mesh; the payload is never parented to the hand. This test covers an irregular convex surface, not arbitrary concave objects or airtightness of manufactured cups. The same bounded vacuum and flexible-lip assumptions apply.

West reach required extending the simulated base yaw (J1) and wrist yaw (J6) intervals to +/-180 degrees; other arm planning limits stay at 155 degrees. This remains an ideal position-controlled arm, with no motor torque or cable routing validation.

`setup_three_objects` retains its original name for compatibility and now resets all five objects. `add_irregular_object` installs the irregular mesh once and preserves an existing instance. Geometry and outward convex support planes are saved in `irregular_geometry.json`; mesh vertices/faces are in `runtime/irregular_mesh.json`.


## Concave challenge object northwest

Select `extreme` for a red asymmetric stepped solid at `[-.30,.30,.06]`.
It comprises six abutting boxes grouped into one independent 100 g rigid body.
The overall dimensions are 66 x 64 x 48 mm. Its collision representation retains
its recesses; it is not replaced with one convex hull. Component mass properties
are computed before grouping. Suction normals come from the nearest actual
component face. No payload parenting or pose control is used during carry.

```matlab
run_octopus('extreme', [-.30,.30,.06], [-.18,.32,.06])
```

This tests an asymmetric concave rigid object with flat contact patches. It does
not establish reliable grasping of arbitrary irregular objects, porous materials,
or physical cup airtightness. Sharp edges and recesses can prevent seals.
`add_extreme_object` installs it while preserving the other payload positions.

