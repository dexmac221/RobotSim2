# Fix: Gemini --free-joints IK Failures

## Problem

When using `--free-joints` (or `--free-wrist`) flag with Gemini agent, the orientation search would find valid orientations, but then IK would still fail with "Inverse kinematics failed to converge" errors.

**Example errors:**
```
Gemini IK failed for point=[482, 570] mapped to world=(0.471, -0.292, 0.160): Inverse kinematics failed to converge
Gemini IK failed for point=[480, 570] mapped to world=(0.479, -0.292, 0.160): Inverse kinematics failed to converge
Gemini IK failed for point=[579, 606] mapped to world=(0.156, -0.400, 0.160): Inverse kinematics failed to converge
```

## Root Cause

There was a **height inconsistency** between two stages:

1. **Orientation Search** (`find_feasible_orientation` in `reachability.py`):
   - Tests IK at `approach_height = max(TABLE_HEIGHT + hover_padding, tip_position[2] + hover_padding)`
   - This adds 0.05m hover padding **on top of** the tip height

2. **Actual IK Solve** (`compute_next_state` in `gemini_agent.py`):
   - Was using `tip_position[2]` directly without the additional hover padding
   - This created a **different, lower target** than what was validated

## The Fix

Modified `gemini_agent.py` to use the **same approach height** that was tested during orientation search:

```python
if self.allow_free_orientation:
    success, orientation = find_feasible_orientation(...)
    if not success:
        return None
    
    # NOW: Use the same approach height that was tested
    hover_padding = 0.05
    approach_height = max(TABLE_HEIGHT + hover_padding, float(tip_position[2]) + hover_padding)
    hover_tip_position = np.array([tip_position[0], tip_position[1], approach_height], dtype=float)
else:
    orientation = DEFAULT_ORIENTATION
    hover_tip_position = tip_position  # No change for non-free mode

hover_pose = self._hover_pose_for_tip(hover_tip_position, orientation)
```

## Impact

- ✅ **Fixes:** `--free-joints` now works correctly with edge workspace targets
- ✅ **Tests:** All 15 tests still pass
- ✅ **Backwards compatible:** Only affects behavior when `allow_free_orientation=True`
- ✅ **Consistent:** Orientation search and IK now use identical target heights

## Testing

The fix was validated against:
- `test_free_orientation_enables_additional_targets` - Ensures edge targets work with free orientation
- Full test suite - All 15 tests passing

## What Changed

**File:** `robot_sim_py/gemini_agent.py`
**Function:** `compute_next_state()`
**Lines:** ~155-185

The hover pose calculation now matches the approach height used in `find_feasible_orientation()` when free orientation search is enabled.

---

**Date:** October 1, 2025  
**Status:** Fixed and tested ✓
