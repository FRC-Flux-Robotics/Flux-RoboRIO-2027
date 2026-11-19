# FLUX 10413 — Student Changelog (`STUDENT-CHANGELOG.md`)

## 2026-11-18 — `v2027.1.1` Pinned Vendordep Bump & Shared Core Parity Sync
* **🔧 What Changed:**
  * **Vendordep Fixes (`vendordeps/`):** Bumped `photonlib` (`v2026.3.1 -> v2026.3.4`, fixing NT4 disconnects & OOB packet decode) and `AdvantageKit` (`26.0.1 -> 26.0.2`).
  * **Real Hardware Protocol, PathPlanner & Control-Law Sync (`frc.lib.*`, `Robot.java`, `RobotIdentity.java`):** Synced `BackTo2026Translator` rules, PWF `ROB-70001` little-endian CAN decoding (`Mfg ID 11`), `8.069 m` field width + `SafeAutoBuilder` `Wall/Blue -> CENTER_RED_2027` (`LocalOrigin` preserved on shop templates), `/sys/firmware/devicetree/base/serial-number` + tag fallback, `CanConfigBudget` (`3.0 s` / `50 ms`), `optimizeBusUtilizationForAll(...)`, `FusedCANcoder` Pro/FD fallback + `0.3°` steer deadband, `MAIN_THREAD_RT_PRIORITY = 1`, `REPLAY` CAN skip, and `Vision` yaw-rate/tilt/finite gates.

## 2026-11-03 — Initial `v2027.1.0` Pre-Kickoff Baseline: 2026 `FUEL` & `CORAL` Robot Code, 3D Autonomous Lab & Student Exercise Kit
* **🔧 What's Included:**
  * **Shared Core Engine (`frc.lib.*`) + `FUEL` 6-Mechanism Superstructure (`frc.robot.*`):** 250 Hz CTRE Phoenix 6 swerve drivetrain (`Drive`, `ModuleIOTalonFX`, `ModuleIOSim`), full-queue AprilTag vision, `Superstructure` (`Intake`, `Tilter`, `Indexer`, `Feeder`, dual `Shooter`, `Hood`), `ShotCalculator` Shoot-on-the-Move solver, `CORAL`/`CHASSIS_3` Ghost Superstructure mode, `VirtualGeofence`, `FaultSupervisor`, `BatteryIO`, and `FluxFig` CSV configuration (`libs/fluxfig-1.0.0.jar`).
  * **Student 3D Autonomous Lab (`StudentAutoRoutines.java` & `./gradlew renderAuto`):** Empty `STUDENT` starter (`createStudentDefaultAuto`) at `(3.10 m, 4.05 m, 0°)` in front of the Blue Hub with inline cheat sheets for **Track A (Fixed Code Moves)** (`driveRelative`, `driveTimedVelocity`, `waitLoopSeconds`, `createAimAndShootHubCommand`) and **Track B (PathPlanner `.path` Moves)** (`resetPoseToPathStart`, `"Hub to Depot"`, `"Collect"`), rendering an interactive 3D browser viewer (`build/auto-sim/student-auto.html`) and MP4 video (`build/auto-sim/student-auto.mp4`) with 8-ball hopper physics across 10 field `FUEL` balls.
  * **4-Level Progressive Student Exercise Kit (`StudentExercises.java` & `./gradlew checkExercises`):** Self-graded JUnit 5 exercises covering `FluxFig` hardware safety limits, `Superstructure` shot-release interlocks, swerve heading alignment, and 15-second multi-volley autonomous routines.
* **🎯 What to Try First:**
  * Run `./gradlew renderAuto -Pauto=list`, `./gradlew renderAuto -Pauto="Fixed Code Moves"`, or `./gradlew renderAuto -Pauto="Hub to Depot + Wing"` and open `build/auto-sim/student-auto.html` in your browser!
  * Run `./gradlew checkExercises` to test your solutions in [`src/main/java/frc/robot/exercises/StudentExercises.java`](src/main/java/frc/robot/exercises/StudentExercises.java).
