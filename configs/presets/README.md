# Configuration presets

A **preset** is one named configuration of one system. It is the third level of the
results layout:

```
results/<dataset>/<system>/<preset>/run<NN>/
```

| `PRESET` | Launch file used |
|---|---|
| `default` (the default) | `configs/launch/<system>.launch` — as-shipped upstream parameters, plan §9 |
| anything else | `configs/presets/<system>/<preset>.launch` |

## Why presets are separate files

A variant must never be produced by editing the shared launch file between runs. That
is how findings §4.3 happened: configuration arms and their baselines ended up built
from different binaries, and nothing in the results recorded it. With one file per
preset, a batch sweeps `default`, `cube400`, `cube100` back to back with no human
touching anything in between, and `preset_sha` in each `metrics.json` fingerprints the
exact files that were used.

## Adding one

Copy the default launch and change only what the variant is about:

```bash
mkdir -p configs/presets/fast_lio
cp configs/launch/fast_lio.launch configs/presets/fast_lio/cube400.launch
# then edit the one parameter under test, e.g. cube_side_length -> 400
```

Run it:

```bash
BAGS_DIR=/path/to/bags SYS=fast_lio PRESET=cube400 NAME=mydataset N=5 ./bench.sh run

# or sweep the variant against its baseline in one batch
BAGS_DIR=/path/to/bags SYS=fast_lio PRESET="default cube400" NAME=mydataset N=5 ./bench.sh run
```

Two rules the fingerprints depend on:

- keep the `<arg name="config" default="..."/>` line — the YAML it points at is hashed
  into `preset_sha` alongside the launch file, and `preset_files` in `metrics.json`
  records both paths;
- change algorithm parameters here only. A preset that needs a **source** patch is a
  different binary, which `binary_sha` will expose as a separate subgroup in
  `stats.txt` — that is the intended behaviour, not a workaround.
