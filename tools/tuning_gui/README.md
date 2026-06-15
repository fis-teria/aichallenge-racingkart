# AI Challenge Tuning GUI

Local browser UI for choosing `control_method`, editing the related launch/YAML/CSV files, rebuilding, and launching `make dev`, `tools/evalwrap run`, or quick `make eval`.

## Start

```bash
cd /home/graneple/git/autononous_ai/aichallenge-racingkart
tools/run_tuning_gui.bash --background
```

Open `http://127.0.0.1:8765`.

Foreground mode is also available when you want to keep logs in the terminal:

```bash
tools/run_tuning_gui.bash
```

Useful management commands:

```bash
tools/run_tuning_gui.bash --status
tools/run_tuning_gui.bash --stop
tools/run_tuning_gui.bash --restart
```

## Behavior

- The control method list is read from `aichallenge_submit_launch/launch/reference.launch.xml`.
- `XMLへ保存` updates the `control_method` default in the XML launch chain.
- YAML/XML files open in table-edit mode by default. YAML scalar values are shown as path/value rows.
- The table includes an editable `description` column. Descriptions are GUI metadata stored in `tools/tuning_gui/.state/parameter_descriptions.json`; they are not written into YAML/XML comments.
- XML files are shown as one row per element, with common editable attributes such as `name`, `default`, `value`, `to`, `from`, `args`, and `file` grouped into columns.
- Edit table cells, then save; the GUI reflects table changes back into the file text before validation.
- Text edit mode remains available for CSV files, comments, structural YAML/XML edits, or large manual changes.
- `Path Editor` opens the active MPC reference path on the occupancy-grid map. It can move, add, delete, and smooth path points, then save a recalculated `s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2` CSV.
- Path smoothing keeps the point count stable and applies a neighbor-average pass before save; `undo` restores the points from immediately before the last smoothing operation.
- In `move` mode, drag empty map space to select points with a rectangle. Drag a selected green point or selected segment to move the whole selected range together; use middle-click, Alt-drag, or Shift-drag to pan the map.
- `smooth` affects only the selected rectangle range when points are selected. Use `clear` to return smoothing to the whole path.
- Path Editor saves under `multi_purpose_mpc_ros/env` or `multi_purpose_mpc_ros/maps`; the first save from an original path defaults to `<name>_manual.csv`, and later manual-path saves overwrite the same CSV with a backup before writing.
- Saving a file validates YAML/XML/JSON/CSV before writing and stores a timestamped backup under `tools/tuning_gui/backups/`.
- `保存してビルド` and control-method `保存してビルド` start `make autoware-build` after a successful edit.
- `dev` runs with `CONTROL_METHOD=<selected>` and can run `make autoware-build` first.
- `evalwrap` runs `tools/evalwrap run --label ...` with `CONTROL_METHOD=<selected>`. With the update-build checkbox enabled, it regenerates the submit archive, rebuilds the eval image, runs `make eval`, and collects reports.
- `quick eval` keeps the lighter direct `make eval` path for fast local checks; it can also run `make autoware-build` first.
- The run note is passed to evalwrap as the label/note. If it is empty, the GUI uses `<control_method>-gui-eval`.
- While a command is running, parameter edits are locked.
- Run snapshots and command history are stored under `tools/tuning_gui/history/`.
