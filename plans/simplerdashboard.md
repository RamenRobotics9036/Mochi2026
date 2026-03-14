# Plan: Push-based `BasicInfoDashboard`

Replace the supplier-callback pattern with a push-based design: vision classes hold a
`BasicInfoDashboard` reference and call individual `updateXxx()` methods during their
`periodic()`. The dashboard stores cached values and publishes them in `update()` — no
null-guarded suppliers needed.

---

## Steps

### Phase 1 — Redesign `BasicInfoDashboard`

1. Remove `BooleanSupplier` and `DoubleSupplier` imports (keep `Supplier` for
   `setVisionKalmanSupplier`).
2. Remove the 9 supplier fields; add plain cached fields with safe defaults:
   - `double m_cachedVisionConfidence = 0.0`
   - `boolean m_cachedHasTargetLock = false`
   - `boolean m_cachedHasMultiTagLock = false`
   - `boolean m_cachedIsLatestMt2 = false`
   - `double m_cachedTx = 0.0`
   - `List<Integer> m_cachedTargetList = List.of()`
   - `OptionalDouble m_cachedVisionError = OptionalDouble.empty()`
   - `boolean m_cachedIsMotionless = false`
   - `double m_cachedSecondsStill = 0.0`
3. Add individual push methods called by `MultiCamOdometryWrapper.periodic()`:
   - `updateVisionConfidence(double score)`
   - `updateTargetLock(boolean hasLock)`
   - `updateMultiTagLock(boolean hasMultiLock)`
   - `updateIsLatestMt2(boolean isMt2)`
   - `updatePrimaryTagTx(double tx)`
   - `updateVisibleTagIds(List<Integer> ids)`
   - `updateVisionErrorAtSnapTime(OptionalDouble error)`
   - `updateMotionlessState(boolean isMotionless, double secondsStill)`
4. Delete `setVisionDependenciesOnDash(...)` — no callers remain after this refactor.
5. Keep `setVisionKalmanSupplier()` unchanged (Kalman wiring stays in `RobotContainer`).
6. Simplify `update()` to read the cached fields directly — drop all supplier null-guards
   for vision data (safe defaults cover the unset/pre-wired case).
7. Constructor is **unchanged** — still takes `configInterface`, `drivetrain`, `cameraNames`.

### Phase 2 — Wire `MultiCamOdometryWrapper`

1. Add `BasicInfoDashboard m_dashboard` and `MotionlessTracker m_motionlessTracker` fields.
2. Keep the existing 2-arg constructor `(real, initiallyEnabled)` — delegates to the new
   4-arg constructor with `null, null`. This preserves all existing unit tests.
3. Add a 4-arg constructor `(real, initiallyEnabled, dashboard, motionlessTracker)` for
   production use.
4. At the end of `periodic()`, if dashboard is non-null, call each update method:
   ```
   m_dashboard.updateVisionConfidence(getConfidenceScore())
   m_dashboard.updateTargetLock(hasTargetLock())
   m_dashboard.updateMultiTagLock(hasMultiTagLock())
   m_dashboard.updateIsLatestMt2(isLatestMt2())
   m_dashboard.updatePrimaryTagTx(getPrimaryTagTx())
   m_dashboard.updateVisibleTagIds(getVisibleTagIds())
   m_dashboard.updateVisionErrorAtSnapTime(getVisionErrorAtSnapTime())
   m_dashboard.updateMotionlessState(m_motionlessTracker.isMotionless(),
                                     m_motionlessTracker.getSecondsStill())
   ```

### Phase 3 — Update `MultiCamOdometryFactory`

1. Update the `wrapper` constructor call to pass `basicInfoDashboard` and
   `motionlessTracker` (4-arg form).
2. Remove the `basicInfoDashboard.setVisionDependenciesOnDash(...)` call block.
3. Change return type from `CamOdometryInterface` to `MultiCamOdometryWrapper`.
4. Update Javadoc to reflect the new return type.

### Phase 4 — `RobotContainer` (no changes needed)

- Already passes `basicInfoDashboard` and `m_motionlessTracker` to `MultiCamOdometryFactory.create()`.
- `basicInfoDashboard.setVisionKalmanSupplier(() -> m_visionKalmanFilter)` stays as-is.

---

## Relevant Files

| File | Phase | Change |
|------|-------|--------|
| [src/main/java/frc/robot/visutils/BasicInfoDashboard.java](../src/main/java/frc/robot/visutils/BasicInfoDashboard.java) | 1 | Remove supplier fields/setters; add cached fields + `updateXxx()` methods; simplify `update()` |
| [src/main/java/frc/robot/visutils/MultiCamOdometryWrapper.java](../src/main/java/frc/robot/visutils/MultiCamOdometryWrapper.java) | 2 | Store dashboard + motionlessTracker; call updates in `periodic()` |
| [src/main/java/frc/robot/visutils/MultiCamOdometryFactory.java](../src/main/java/frc/robot/visutils/MultiCamOdometryFactory.java) | 3 | Update return type; pass params to wrapper constructor; remove old supplier wiring |
| [src/main/java/frc/robot/RobotContainer.java](../src/main/java/frc/robot/RobotContainer.java) | 4 | **No changes** — already correct |

---

## Decisions

- **`VisionKalmanFilter`**: the single `setVisionKalmanSupplier()` call in `RobotContainer` is
  preserved — the supplier pattern is fine for this one stable reference.
- **`MotionlessTracker`**: travels through the factory → wrapper constructor, consistent with
  the existing factory signature.
- **`setVisionDependenciesOnDash()`** is deleted — callers are replaced by the factory change.
- **2-arg constructor preserved** on `MultiCamOdometryWrapper` to avoid touching unit tests.
- Scope excludes `SingleCamOdometry` and `MultiCamOdometry` — the wrapper is the correct
  boundary for dashboard updates.

---

## Verification

1. `./gradlew assemble` — zero compile errors
2. `./gradlew test` — all existing tests pass (2-arg constructor is still valid)
3. Simulate with `./gradlew simulateJava` — confirm `BasicInfo/*` NT keys publish correctly
4. No null-pointer risk: before `wireVision` is wired (tests, early init), safe-default cached
   values (0.0 / false / empty list) are published
