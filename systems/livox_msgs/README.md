# livox_msgs — Livox message types, not the driver

Two catkin packages, `livox_ros_driver` (gen1) and `livox_ros_driver2` (gen2), each generating
only `CustomMsg` / `CustomPoint`. No driver node, no Livox-SDK — **they cannot talk to a
sensor.** We replay bags, so the types are all any baseline needs, and that is why the image
carries no SDK. To capture from real hardware, restore the upstream driver (commits below).

## One msg/ for both package names

ROS1 computes a message md5 over the canonicalised field types and names; **the package name
is not part of it.** Measured, gen1's `CustomMsg` and the `livox_ros_driver2/CustomMsg` in our
bags are both `e4d6829bdfe657cb6c21a746c86b21a6` — so the two generations agree field for
field, and one `.msg` pair backs both packages. `msg/` here is the source of truth; each
package's `msg` is a relative symlink to it.

The same property is why FAST-LIO and Point-LIO, which subscribe with the gen1 type, read our
gen2 bags at all: ROS1 matches subscriptions on md5, not on the type name. The gen2 package
exists for systems that dispatch on the datatype *string* instead (BIEVR-LIO's `ShapeShifter`).

## Acceptance check

`CustomMsg` must be `e4d6829bdfe657cb6c21a746c86b21a6` and `CustomPoint`
`109a3cc548bb1f96626be89a5008bd6d`, in both packages. A mismatch breaks every downstream
subscription silently.

`build_systems.sh` links only gen1 (into `.ws/fast_lio` and `.ws/point_lio`); gen2 is here
ahead of BIEVR-LIO, so check it in a throwaway workspace:

```bash
source /ws/fast_lio/devel/setup.bash && rosmsg md5 livox_ros_driver/CustomMsg

mkdir -p /ws/_chk/src && ln -sfn /slam-bench/systems/livox_msgs/livox_ros_driver2 /ws/_chk/src/
( cd /ws/_chk && catkin_make -DCMAKE_BUILD_TYPE=Release ) >/dev/null
source /ws/_chk/devel/setup.bash && rosmsg md5 livox_ros_driver2/CustomMsg && rm -rf /ws/_chk
```

## Provenance

- `msg/*` and `LICENSE.txt` are verbatim (MIT) from
  [`Livox-SDK2/livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2) at
  `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` (2026-04-14).
- The `systems/livox_ros_driver` submodule this replaced was at
  [`Livox-SDK/livox_ros_driver`](https://github.com/Livox-SDK/livox_ros_driver)
  `3d240d5666129e1a3052e78ee8487a04b08fdda3` (2023-02-28) — recorded so the driver can be
  restored, not the source of the `.msg` above.
