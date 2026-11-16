# WARDEN: presentation media evidence and provenance

This register was assembled from the user-provided `SPECTRA, Taiwan(1).pptx` (17 slides, five embedded MP4 clips) and `Capstone poster, копия (10)(1).pdf` (one-page team poster). It documents **historical team materials**, not newly reproduced engineering measurements. No raw experiment logs or labeled target-by-target evaluation dataset were supplied with these media.

## Actual embedded demonstration footage

| Slide | Original embedded video | Duration | Visible subject | Responsible interpretation |
|---|---|---:|---|---|
| 8 | `VAHMmAFtAgM.mp4` | ~7.75 s | Physical touchscreen on the robot; guest-assistant selection and confirmation screens | Evidence that the UI was displayed and interacted with; does not prove actual staff dispatch |
| 9 | `VAHMl8vncjU.mp4` | ~9.0 s | Physical robot inside a staged hotel-room scene, portrait footage | Evidence of the physical room-test setup; by itself, not proof of automated scanning/detection success |
| 9 | `VAHMl5SkrFQ.mp4` | ~3.09 s | A test scene containing a tabletop target/object | Context for the staged inspection environment |
| 10 | `VAHMl3kEJa0.mp4` | ~13.77 s | Physical robot by floor markings, portrait footage | Movement/test footage, but does not establish autonomous line-following control mode |
| 10 | `VAHMl1RZ6fs.mp4` | ~15.65 s | Physical robot moving in staged hotel-room environment, landscape footage | Strong candidate for portfolio demo; visual evidence of mobility in a controlled setup |

The screenshots committed under `assets/images/` were extracted from the team's original embedded videos; they are **not AI-generated images or renders**. No source MP4 was committed to GitHub here. Obtain team consent before public redistribution of joint media.

## Useful visual source sections

- Taiwan slides 4–5: concept illustration and hardware stack. Slide 4's polished product image should be described as a **concept rendering**, not as a photograph of the assembled robot.
- Taiwan slide 6: conceptual LED ON/OFF multi-angle scanning explanation. The final recovered host source configured `USE_LED=False`, so this should not be presented as the validated final running mode.
- Taiwan slide 7: historical sensor-fusion formula and illustrative/dashboard screenshot. The slide gives **initial weights** 0.50 optical / 0.20 thermal / 0.20 RF / 0.10 AI and threshold 0.60; the recovered final host uses different, LED-off weights and threshold 0.55. Do not silently equate the two configurations.
- Taiwan slide 8: a design-style operations dashboard with aggregate analytics and incident examples, alongside **actual touchscreen footage**. Figures and room-incident reports on its design panels have not been traced to WARDEN's recovered Flask code or a real telemetry database, and should not be represented as measured deployment outcomes.
- Taiwan slides 9–10: strongest original demonstration videos and physical robot photographs.
- Capstone poster: architecture diagram and hardware listing useful as a **historical design document**, subject to code consistency checks.

## Historical result inconsistency — not yet resolved

- **Capstone poster:** 4 planted camera-like targets, 8 confounders; fused method detected **4/4**, with **1 false positive**.
- **Taiwan presentation slide 10:** 5 planted camera-like targets, 8 confounders; fused method detected **4/5**, with **1 false positive**.

These may reflect separate testing iterations, revisions, or a reporting error. Without original measurements, it is **not possible** to know which interpretation is correct. Neither result is currently asserted as an independently validated detection benchmark in this repository.

The poster and slide 7 also describe the full optical/thermal/RF/AI pipeline. But the original recovered Raspberry Pi host had `FAKE_THERMAL=True` and `USE_LED=False`. This is an important distinction between a **designed research framework** and the **last saved source configuration**. The current reviewed code eliminates fake telemetry and reports failed/unavailable measurements honestly; that revision has **not yet been hardware-validated**.

## Portfolio editorial decisions

1. Lead with an authentic physical-robot image, not the 3D concept render.
2. Describe WARDEN as a **working university research prototype**; do not claim production hotel deployment or validated hidden-camera detection performance.
3. Distinguish **joystick-controlled motion** from experimental line-following; black floor tape visible in footage is not proof that autonomous line-following worked.
4. Present the **dashboard and physical touchscreen** as evidenced prototypes; distinguish illustrative operations UI from recorded telemetry and actual staff notification.
5. Keep personal/group presentation recordings and original commercial video off public GitHub until everyone involved agrees to sharing them.
6. Publish method details and failure-mode handling, not unsupported precision/accuracy or statistical confidence claims.

## Source assets retained outside the repository

- Original Taiwan PPTX (includes all five video files).
- Original capstone PDF poster.

Neither original source file is modified by this review. Do not publish poster QR codes, team contacts or identifiable photos without checking them and obtaining appropriate sharing permission.
