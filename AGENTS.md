# AGENTS.md — Flux-RoboRIO-2027 Repository Guide (FRC Team 10413)

> **Standalone Repository Guide:** This `AGENTS.md` governs only `Flux-RoboRIO-2027` and is 100% self-contained for single-repository clones.

---

## 1. Repository Overview & Key Files
`Flux-RoboRIO-2027` is the **WPILib 2026 (`edu.wpi.first.*`, JDK 17) roboRIO 2.0 + CTRE CANivore robot codebase** for **FRC Team 10413 (FLUX Robotics)**, combining the `41`-class `frc.lib.*` core engine with our **2026 `FUEL` 6-mechanism shooting stack and `CORAL`/`CHASSIS_3` Ghost Superstructure mode** in `frc.robot.*`:
- [`README.md`](README.md) & [`STUDENTS.md`](STUDENTS.md): Repository overview, auto-generated class/dependency map ([`docs/class-map.html`](docs/class-map.html)), `FluxFig` mechanism table, and active `TODO(student)` board.
- [`STUDENT-CHANGELOG.md`](STUDENT-CHANGELOG.md): Student-readable digest of every code and CSV change.
- [`src/main/java/frc/robot/exercises/StudentExercises.java`](src/main/java/frc/robot/exercises/StudentExercises.java) & [`src/main/java/frc/robot/autos/StudentAutoRoutines.java`](src/main/java/frc/robot/autos/StudentAutoRoutines.java): 4-level student exercise kit (`./gradlew checkExercises`) and 2026 `FUEL` Hub shooting autonomous sandbox (`./gradlew renderAuto` with [`"Hub to Depot.path"`](src/main/deploy/pathplanner/paths/Hub%20to%20Depot.path)).
- [`src/main/java/frc/robot/Superstructure.java`](src/main/java/frc/robot/Superstructure.java), [`src/main/java/frc/robot/util/ShotCalculator.java`](src/main/java/frc/robot/util/ShotCalculator.java), & [`src/main/java/frc/robot/commands/ShotCalibrationCmd.java`](src/main/java/frc/robot/commands/ShotCalibrationCmd.java): 6-mechanism coordinator (`Intake`, `Tilter`, `Indexer`, `Feeder`, dual `Shooter`, `Hood`), 3-step Shoot-on-the-Move solver, and range-table calibration command.
- [`src/main/deploy/configs/`](src/main/deploy/configs/mechanisms.csv): Runtime `FluxFig` CSVs ([`parameters.csv`](src/main/deploy/configs/parameters.csv), [`mechanisms.csv`](src/main/deploy/configs/mechanisms.csv), [`swerve_modules.csv`](src/main/deploy/configs/swerve_modules.csv), [`cameras.csv`](src/main/deploy/configs/cameras.csv), [`batteries.csv`](src/main/deploy/configs/batteries.csv), [`releases.csv`](src/main/deploy/configs/releases.csv), [`range_table.csv`](src/main/deploy/configs/range_table.csv)) loaded by [`libs/fluxfig-1.0.0.jar`](libs/fluxfig-1.0.0.jar).

### Essential Commands (Run from Repository Root)
```bash
./gradlew updateStudentDocs spotlessApply test build   # Format, sync STUDENTS.md, and run all Tier 1-5 JUnit tests
./gradlew checkExercises                               # Grade 4-level StudentExercises.java
./gradlew renderAuto [-Pauto="<name>" | -Pauto=list]   # Simulate any 2026 auto (15s) -> build/auto-sim/student-auto.{html,mp4}
./gradlew simulateJava                                 # Launch interactive WPILib Sim (AdvantageScope nt4://localhost)
./gradlew deploy                                       # Fast ~6s spot-deploy to roboRIO 2.0 over Wi-Fi/USB
```

---

## 2. Target Robots & RoboRIO / CANivore Hardware Rules (`RobotIdentity`)
Robots identify at boot via `RobotController.getSerialNumber()` (with fallback to `RobotController.getComments()`):

| Robot ID (`RobotIdentity`) | Controller | CAN Bus Mapping | Vision & Superstructure Mode |
| :--- | :--- | :--- | :--- |
| **`FUEL`** | roboRIO #1 | • `"Drivetrain"` CANivore (Pigeon 20)<br>• `"Mech"` CANivore<br>• `"rio"` native CAN (`auxCanBus`) | • **3x OV9281** on PhotonVision (`OV9281-2`, `OV9281-1`, `OV9281-3`)<br>• **6 Real Mechanisms:** `Intake` (ID 1), `Tilter` (ID 20, `-17.0 rot`), `Indexer` (ID 4), `Feeder` (ID 3), `Shooter` (ID 11 + follower 10), `Hood` (ID 12) |
| **`CORAL` / `CHASSIS_3`** | roboRIO #2 / #3 | • `"CANdace"` (Pigeon 24) / CTRE CANivore<br>• `"rio"` native CAN (`auxCanBus`) | • **1–3x OV9281** on PhotonVision + **Ghost Superstructure Mode** (`REAL` drive/vision + `SIM` virtual mechanisms feeding `VirtualBatterySim` & `FaultSupervisor`) |
| **`COMP_2027`** *(Fallback)* | roboRIO 2.0 | • `"Drivetrain"` + `"Mech"` CANivores + `"rio"` | • **3–4x OV9281** on PhotonVision (`ROBORIO_CANIVORE_2026` profile) |

### Hardware Constraints (NEVER Violate)
1. **Inline Battery Fuel Gauge (`BFG` `SEN-36005`) on `auxCanBus`:** Wire the 2-pin `BFG` CAN lead strictly to `"rio"` native 1 Mbps CAN 2.0 (`auxCanBus`—never on a CTRE CAN FD CANivore). `BatteryIO.java` decodes 29-bit frames `0x010`, `0x011`, and `0x012`.
2. **Swerve Current Limits & `MotionMagicExpoVoltage` Steer Gains:** Enforce **`60.0 A` Stator / `60.0 A` Slip** and **`40.0 A` Supply** current limits on all swerve drive motors (`withSlipCurrent` overwrites `StatorCurrentLimit`), `MotionMagicExpoVoltage` steer gains (`kP = 15.0, kD = 0.9, kS = 0.1, kV = 1.5, MotionMagicExpo_kV = 1.5, MotionMagicExpo_kA = 0.1`), and dual translation/rotation `PiecewiseSensitivity` curves.
3. **Log Paths & `.hoot` Auto-Logging (`LogFileManager`, `Robot.java`):** Resolve `/U/logs` first, then `new File(Filesystem.getOperatingDirectory(), "logs")`, and keep `SignalLogger.enableAutoLogging(false)` in `Robot.java`.

---

## 3. Package Boundaries (`frc.lib.*` vs. `frc.robot.*`) & Loop Standards
1. **`frc.lib.*` (`41` Core Engine Classes) vs. `frc.robot.*` (`9` 2026 `FUEL` / `CORAL` Classes):**
   - `frc.lib.*` (`drive`, `vision`, `mechanism`, `safety`, `commands`, `util`) must **never** import `frc.robot.*` (enforced by `Tier1ArchitectureHygieneTest.testFrcLibNeverImportsFrcRobotMechanisms`).
   - Students edit `frc.robot.*` (`RobotContainer.java`, `Superstructure.java`, `ShotCalculator.java`, `ShotCalibrationCmd.java`, `StudentAutoRoutines.java`, `StudentExercises.java`) and `src/main/deploy/configs/{mechanisms,range_table}.csv`.
2. **Pure AdvantageKit IO & Zero `SmartDashboard` / `Preferences`:**
   - Subsystems interact with hardware strictly via `*IO` interfaces (`ModuleIO`, `GyroIO`, `VisionIO`, `MechanismIO`, `BatteryIO`) with matching `*IOSim` implementations.
   - `SmartDashboard`, `SendableChooser`, `Preferences`, and `java.awt.*` in `src/main/java/` are banned by Tier 1 CI. Use `Logger.recordOutput("Subsystem/Key", value)` and `LoggedTunableNumber`.
3. **Zero Per-Loop Allocations, Google Java Style & Batched CAN Signals:**
   - Follow the **Google Java Style Guide** (`camelCase` instance/static fields with zero `m_` or `s_` prefixes, `UPPER_SNAKE_CASE` constants, and `google-java-format` via `./gradlew spotlessApply`).
   - Reuse `private final` CTRE `ControlRequest` fields (`VelocityVoltage`, `MotionMagicExpoVoltage`, `PositionVoltage`, `VoltageOut`). Never call blocking `TalonFX.getConfigurator().apply(...)` inside the 50 Hz loop; scale brownout demand via `brownoutDemandScale` in `Drive.runVelocity()`.
   - Batch 50 Hz signals via `PhoenixSignals.register(...)` and 250 Hz swerve odometry via `PhoenixOdometryThread`.
4. **Vision & Field Coordinates (`Vision.java`, `FieldPositions.java`, `PreMatchCalibrator.java`):**
   - Drain all unread frames (`getAllUnreadResults`) from every camera every cycle. Seed Pigeon 2 yaw via `PreMatchCalibrator` (MegaTag1 while Disabled) and switch to MegaTag2 when Enabled.
   - Always use `FieldPositions.isInsideField(x, y, margin)`, `FieldPositions.forAlliance(...)`, and `FieldPositions.operatorForward()` (`WALL_BLUE_2026` $[0, L] \times [0, W]$; `FULL_FIELD`, `HALF_FIELD_BLUE/RED`, `SHOP_5X3`; NWU $+X$ forward, $+Y$ left, $+CCW$).
5. **Motor Safety & `Superstructure` Interlocks:**
   - Every `TalonFXConfiguration` must enable both `StatorCurrentLimit` and `SupplyCurrentLimit`.
   - Route multi-mechanism sequences (`Intake`, `Tilter`, `Indexer`, `Feeder`, `Shooter`, `Hood`) through `Superstructure.java` (`IDLE`, `INTAKING`, `PREPPING_SHOT`, `SHOOTING`, `OUTTAKING`, `CLIMBING`) guarded by `isReadyToFire()`.

---

## 4. Verification Gate, Repo Gotchas & Student Coaching
- **Same-Commit Docs Rule:** Whenever any Java class, CSV config, or `TODO(student)` changes, run `./gradlew updateStudentDocs spotlessApply test` and append a 3-line entry (*🔧 What Changed*, *🎮 How It Affects Sim/Robot*, *🎯 What to Try Next*) to [`STUDENT-CHANGELOG.md`](STUDENT-CHANGELOG.md) in the same commit.
- **Simulation & Test Gotchas:**
  - In `MechanismIOSim` (`ARM_COSINE`), keep `armRadPerPosRot = (spanRot > 2.0) ? (2.0 * Math.PI / armGearing) : (2.0 * Math.PI)` so geared mechanisms (`Hood` `0..12.5 rot`, `Tilter` `-17..0 rot`) travel at true speed.
  - In `Tier3CtreSimHardwareTest`, keep class-scoped `keepAliveTalon` on `"rio"` and call `DriverStationSim.setEnabled(true)` + `Unmanaged.feedEnable(100)` inside 250 Hz sim loops.
  - In JUnit tests and `StudentAutoRoutines`, use `withLoopTimeout(Command, double)` rather than `SimHooks.stepTiming` so `Timer.getFPGATimestamp()` is never frozen across tests.
  - Commit SVG badges locally in [`docs/badges/`](docs/badges/tier1-hygiene.svg) using relative links in `README.md`.
- **Agent Student Coaching Mode:** When scaffolding student tasks, provide clear Javadocs, `// TODO(student)` steps, plain-English physics/coordinate explanations, and a pre-written JUnit 5 test.
