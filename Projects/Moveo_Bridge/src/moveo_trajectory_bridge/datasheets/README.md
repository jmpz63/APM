# Datasheets / Mechanical References

Place motor and transmission datasheets here. Suggested naming:

- joint1_motor.pdf
- joint2_motor_left.pdf
- joint2_motor_right.pdf
- joint2_belt_stage.png (photo of pulley tooth counts for confirmation)
- joint3_motor.pdf
- joint4_motor.pdf
- joint5_motor.pdf
- joint6_motor.pdf
- gripper_motor_or_servo.pdf
- gearbox_jointX.pdf (if any gearbox datasheet)
- belt_profile_gt2.pdf (if needed)

## Optional Supplemental Files
- torque_curves_joint2.png (digitized from PDF if necessary)
- thermal_test_log.md (notes from holding torque / heating test)
- payload_tests.csv (mass vs deflection observations)

## Workflow
1. Drop PDFs/images here.
2. Refer to them from `HARDWARE_NOTES.md` using relative links, e.g. `[Joint2 Motor](datasheets/joint2_motor_left.pdf)`.
3. Once all major datasheets are present, we will parse/record key values into the summary table.

## Future Automation
A helper script could hash the PDFs and store a small JSON index to detect when specs change and re-trigger limit recomputation.
