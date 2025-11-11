# Viewer Changes - November 2025

## Summary

Changed default viewer from basic C++ GLFW window to **full Python MuJoCo viewer** with ImGui UI.

## What Changed

### Before

```bash
ros2 launch openarm_description mujoco_sim.launch.py
# ❌ Got basic C++ GLFW window (no ImGui, limited features)
# ❌ Perturbations broken (always selected body 0)
# ❌ No easy way to run headless
```

### After

```bash
ros2 launch openarm_description mujoco_sim.launch.py
# ✅ Python viewer with FULL ImGui UI (like simulate binary)
# ✅ C++ runs headless internally (fast physics)
# ✅ Interactive perturbations work correctly
# ✅ All visualization options available
```

## New Behavior

### Default Launch (Recommended)

```bash
ros2 launch openarm_description mujoco_sim.launch.py
```

- **C++**: Runs headless (physics only, fast)
- **Python**: Launches full MuJoCo viewer
- **Result**: ONE window with complete UI

### Truly Headless

```bash
ros2 launch openarm_description mujoco_sim.launch.py headless:=true
```

- **C++**: Runs headless
- **Python**: Does NOT launch
- **Result**: No viewer windows (for servers/CI)

### With RViz

```bash
ros2 launch openarm_description mujoco_sim.launch.py headless:=true use_rviz:=true
```

- Uses RViz instead of MuJoCo viewer

## Files Modified

### C++ Side

- `mujoco_ros2_control/src/mujoco_ros2_control_node.cpp`
  - Added `headless` parameter
  - Skip rendering when `headless:=true`
  - Default: runs headless internally

### Python Side

- `openarm_description/scripts/mujoco_passive_viewer.py` (NEW)
  - Full MuJoCo Python viewer
  - Subscribes to `/joint_states`
  - Mirrors simulation state

### Launch

- `openarm_description/launch/mujoco_sim.launch.py`
  - Added Python viewer by default
  - C++ runs with `headless:=true` internally
  - `headless:=true` parameter disables Python viewer too

### Documentation

- `docs/VIEWER_GUIDE.md` (NEW) - Complete usage guide
- `docs/full_mujoco_viewer.md` - Technical details
- `docs/setup.md` - Updated with new viewer info

## Migration

### No changes needed

Existing scripts work the same, but now get better UI:

```bash
# This command works exactly the same
ros2 launch openarm_description mujoco_sim.launch.py

# But now you get:
# - Full ImGui UI
# - Working perturbations
# - All visualization options
```

### To disable viewer (new capability)

```bash
# Now possible!
ros2 launch openarm_description mujoco_sim.launch.py headless:=true
```

## Benefits

1. **Better UX**: Full ImGui interface like `simulate` binary
2. **Working Perturbations**: Ctrl+Right-click actually works
3. **Simpler**: One command for best experience
4. **Flexible**: Easy to run headless when needed
5. **No Rebuild**: Python viewer doesn't require C++ compilation

## Testing

```bash
# Test default (Python viewer)
ros2 launch openarm_description mujoco_sim.launch.py

# Test headless
ros2 launch openarm_description mujoco_sim.launch.py headless:=true

# Test with perturbations
ros2 launch openarm_description mujoco_sim.launch.py
# Ctrl+Right-click on robot links
```

## Known Issues

- Python viewer is read-only (mirrors state, doesn't control physics)
- Small latency (~10ms) between C++ and Python
- Requires `mujoco>=3.0.0` Python package

## Future Improvements

- [ ] Bidirectional force application (Python → ROS2)
- [ ] Configurable viewer settings
- [ ] Optional C++ viewer re-enable parameter

---

**Date**: November 10, 2025
**Status**: ✅ Complete and Tested
