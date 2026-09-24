# Debugging case studies

These short cases preserve reusable fault-isolation lessons from the project's development records. They are not a substitute for the original run logs, and they do not claim that every later candidate passed hardware acceptance.

## 1. Separate a control request from actuator motion

**Symptom:** the controller's requested motor angle changed, but the reported actual angle stayed near 11–12 degrees.

**Diagnosis:** the MCU had produced changing commands, while X42S position feedback showed that the mechanism was not following. That evidence did not support changing the outer-loop P/D gains. The investigation returned to zeroing order, command sequencing, driver state, wiring and supply. UART transmit success alone was not treated as proof of motion.

**Reusable lesson:** compare command and feedback on the same timeline (`cmd10` versus `act10`) before tuning control gains. A changing command with stationary feedback is an actuator-path problem until disproven.

## 2. Include outward velocity in the stopping-distance model

**Symptom:** a ball moving away from the center coasted toward the end of travel before the controller entered its braking profile.

**Diagnosis:** source review found that the stopping-distance velocity term was included only when motion was toward the center. During outward motion the speed contribution became zero, leaving a fixed-distance fallback as the effective trigger. A full outward velocity term caused braking to engage too early and interact poorly with stiction and profile transitions. The tuning record then proposed a scaled outward-speed candidate while keeping the return-speed model separate; it does not establish that this candidate is part of the final Q4 source.

**Evidence boundary:** the original tuning record documents both the code-path defect and the failed full-scale experiment. Any scale factor is an empirical candidate, not a universal physical constant or a claim about the final source.

## 3. Reject sticky pass/fail state transitions

**Symptom:** a run briefly reported success, then the controller left its hold state on the next sample.

**Diagnosis:** the acceptance flag was set when entering HOLD but was not cleared when HOLD conditions stopped being true and the state returned to RECOVER. The displayed pass therefore outlived the condition it represented.

**Fix principle:** derive acceptance from the current continuous hold condition, or explicitly clear the latch on every exit path. Verify the state sequence after the first in-band sample; a one-frame pass indication is not a stable-hold result.

## 4. Distinguish useful Q5 fixes from unverified hypotheses

**Symptom:** the user reported intermittent vehicle stutter, but the corresponding new hardware trace was unavailable.

**Source review:** the prior control path switched the speed request among discrete levels near ball-position thresholds without hysteresis, while acceleration/feedforward work used a fixed interval. This exposed plausible discontinuities and a timing assumption to review. The same record explicitly notes that IRQ overload was not measured and that the proposed preview/feedforward values still needed field calibration.

**Evidence boundary:** treat these as source-level defects or hypotheses according to the evidence named above. Do not present the proposed numerical changes as an experimentally proven stutter fix without a matching hardware run.

## 5. Validate the actual run, not startup recovery

**Symptom:** a log contained recovery activity and a pass-like status, but no deliberate disturbance had begun.

**Diagnosis:** startup residual recovery (`run=0`) was mixed with formal trials. Comparing that interval with a completed disturbance/recovery cycle produced invalid tuning conclusions. A separate acceptance bug also allowed a pass flag to remain set after HOLD exited.

**Fix principle:** segment evidence by the formal run marker, require the expected disturbance and complete state sequence, and independently verify continuous hold. Invalid setup runs remain diagnostically useful but are not performance samples.

## 6. Isolate static-bench effects at compile time

**Symptom:** on a stationary bench, Q6 requested motor angles beyond the configured brake clamp and oscillated instead of settling.

**Diagnosis:** the Q5 launch boost was selected using route distance. On a stationary bench that distance remained zero, so the launch multiplier was applied throughout the Q6 run. The diagnostic decomposition showed `pd100=-252`, equal to a `-1.8 deg` brake request multiplied by `1.40`; the resulting motor command exceeded the intended clamp.

**Fix principle:** gate the bench-only exclusion at compile time with the static-test build option. Do not disable the launch term merely because runtime mode is Q6: the moving-vehicle Q6 path still needs the inherited Q5 launch behavior.

**Reported result:** the subsequent `+50 mm` bench run recorded 134/150 samples in band and a final-10-second maximum error of 0.10 cm. This is a historical user-provided trial result, not a validation run from the current cleanup.

## 7. Treat stiction push and integral as one interacting hold mechanism

**Symptom:** small residual position error either failed to break static friction or produced repeated overshoot around the target.

**Diagnosis:** the first hold path suppressed the bounded stiction pulse, leaving an integral term that could not overcome the measured friction threshold. Allowing a sufficiently bounded pulse in HOLD fixed the deadlock, but triggering it at the scoring-band edge caused repeated pushes and reversals. Separately, allowing the integral to accumulate inside the scoring band slowly increased motor command until it broke friction and displaced the ball.

**Fix principle:** keep the push bounded, gate it outside a wider engagement threshold, shorten its active window, and hold the learned bias constant while parked inside that window. These are Q6-gated adaptations; Q4/Q5 settings remain separate.

**Reported result:** the user-provided trial chronology records a passing `-30 mm` run after HOLD push was enabled, a passing `+30 mm` run after engagement/pulse geometry was narrowed, and a 30-second near-target hold without the earlier integral-induced flick after the integral window was aligned. These were separate iterations, not a single combined A/B trial.

## Source and verification notes

- Q4 cases are condensed from local static-recovery debugging records; per-trial parameter sweeps and raw UART captures are intentionally excluded from the public tree.
- Q5 case 4 comes from a source review and an older log, not from a hardware capture of the reported stutter. IRQ load and the proposed field calibration were not confirmed there.
- Q6 cases 6–7 summarize the historical trial chronology supplied with the project handoff. Raw run files remain outside the publication tree; current acceptance is the user's report and was not repeated by this cleanup.
- The user reports that the Q6 competition behavior was accepted. This repository cleanup has not repeated that physical validation.
