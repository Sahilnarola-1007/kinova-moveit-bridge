# kinova_moveit_bridge

ROS 2 bridge between MoveIt 2 and the KinovaInterface C++ wrapper. Hosts two action servers in a single process:

- **KinovaMoveitBridge** — `FollowJointTrajectory` action server for arm trajectory execution
- **KinovaGripperBridge** — `GripperCommand` action server for Robotiq 2F-85 control

Both nodes share a single `KinovaInterface` instance via `MultiThreadedExecutor`.

## Launch

```bash
ros2 launch kinova_moveit_bridge kinova_bringup.launch.py
```

Launches: RSP + JSP GUI + move_group + RViz + bridge (trajectory + gripper).

## Testing

### Integration Tests

The bridge includes 7 integration tests that verify the full MoveIt → Bridge → Wrapper pipeline using the mock Kortex SDK. Each test spins up the trajectory and gripper bridge nodes with an action client, simulating exactly how MoveIt dispatches goals in production.

**Run tests:**
```bash
colcon build --packages-select kinova_moveit_bridge
./build/kinova_moveit_bridge/test_bridge_integration
```

**Test coverage:**

| # | Test | What it verifies |
|---|------|-----------------|
| 1 | `ValidGoalSucceeds` | 3-waypoint trajectory accepted and executed successfully |
| 2 | `EmptyPointsRejected` | Goal with zero trajectory points is rejected |
| 3 | `EmptyJointNamesRejected` | Goal with missing joint names is rejected |
| 4 | `EStopRejectsGoal` | Goal is rejected when e-stop is active |
| 5 | `GripperOpenSucceeds` | Gripper open command (0.0 rad) reaches target |
| 6 | `GripperCloseSucceeds` | Gripper close command (0.8 rad) reaches target |
| 7 | `CancelTriggersEStop` | Action cancel propagates to `emergencyStop()` on the wrapper |

All tests use `MultiThreadedExecutor` with the same shared `KinovaInterface` architecture as the production `main.cpp`.