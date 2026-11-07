# S3: drawing on the real arm

Tool positions are the FK of the commanded and the measured joints (servo encoders), so these numbers are about how the servos follow the commands, not about the absolute accuracy of the model. Offset = mean of measured - commanded (x is out of the plane, z is up). Shape error = what is left after removing the offset. Along = mean of the remaining error along the direction of motion (negative: behind the command). Delay = the time shift that best lines up the two paths.

| Run | Status | Duration | Error median / max | Offset x, y, z | Shape error median / max | Along | Delay | Size cmd -> meas (y x z) |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| square_2cms_20261106_173448 | complete | 10.2 s | 5.9 / 9.7 mm | +1.1, +0.3, -4.9 mm | 3.6 / 5.1 mm | -2.6 mm | 133 ms | 5.09 x 5.09 -> 5.09 x 5.63 cm |

Largest following error per joint (deg) and the sag measured at the start pose (deg):

| Run | shoulder_pan | shoulder_lift | elbow_flex | wrist_flex | wrist_roll |
|---|---:|---:|---:|---:|---:|
| square_2cms_20261106_173448 | 0.98 (sag -0.04) | 1.66 (sag +0.43) | 2.88 (sag -1.87) | 1.40 (sag -0.49) | 0.42 (sag +0.33) |

![square_2cms_20261106_173448](s3_square_2cms_20261106_173448.png)
