# Avis design sweep: what limits each candidate

Simulated flight, real parts. Every number below comes from a file under `out/verification/` or `out/sweep/`; this report restates them and changes none.

## Method gates

| Gate | Passed | Measured | Limit |
|---|---|---|---|
| V1 | pass | amp 3.79%, phase 2.44 deg | {"amp_error_pct":5,"phase_error_deg":5,"k_range":[0.1,0.5]} |
| V2 | **FAIL** | 9.38% at AR 4 vs prandtl-lifting-line | {"max_error_pct":3,"AR_range":[4,12]} |
| V3 | pass | 1.29%, trend true | {"ct_error_pct":15,"trend":"thrust rises with k as Garrick predicts"} |
| V4 | **FAIL** | 7.0 W vs 18 W (-61.4%), by frozen-body trim at 2 Hz (no free-flight trim table could be built; free flight not run) | {"published_w":18,"band_w":[12.6,23.4]} |

## larus-gl-2

Screened 56448 parameterised designs from real parts in 13.5 s; refined 5 in the flight model.

Chosen: span 1.60 m, area 0.284 m2, AR 9.0, mass 327 g (43 g of it the 15% margin), wing loading 11.3 N/m2.
Motor motor-axi-2208-26-v3 (no continuous rating: lower of efficient band top 9 A and 50% of burst 13 A, 6.5 A), battery batt-gensace-300-3s-45c, ESC esc-hobbywing-skywalker-15a-v2, twist servos servo-kst-x06-v6, gear ratio 20.0.
Cruise 6.5 m/s; rough-air speed 9.5 m/s.

| Gate | Passed | Key measured values |
|---|---|---|
| V5 | **FAIL** | held-out lift 29.1%, thrust 15.8% |
| M1 | **FAIL** | climb 2-3 s -0.44 m/s, lost 0.14 m; robust false |
| M2 | **FAIL** | 3.2 min, power ratio 1.37, cruise 51.1 W; robust false |
| M3 | **FAIL** | turned 360 deg, altitude 6.59 m; robust false |
| M4 | **FAIL** | 4/100 seeds, worst H 20 m; robust false |
| M5 | pass | sink 0.46 m/s, speed 6.78 m/s; robust true |
| M6 | pass | 327 g, 0 uncited parts |
| M7 | **FAIL** | FoS min 0.14, gears false, twist servo 0.705/0.116 N m |
| M8 | **FAIL** | motor 13.40/6.50 A, battery 9.64/13.50 A, servo bus 1.82/2.0 A |

What limits it:

- M1: {"release_speed_m_s":8,"release_pitch_deg":5,"climb_rate_2_to_3s_m_s":-0.4408617782197713,"first_1s_window_climbing_s":1,"height_lost_m":0.13973009452773424,"height_at_6s_m":-1.9978824065757386,"peak_motor_A":14.926325658149752,"peak_batt_A":6.344013359036491,"nominal_passed":false,"robustness_passe
- M2: {"cruise_power_w":51.05623469493752,"cruise_batt_A":4.55858693473339,"endurance_min":3.1588736172346765,"usable_fraction":0.8,"continuous_limit_A":6.5,"power_available_w":69.71368853821471,"power_ratio":1.3654294907322488,"altitude_dev_m":173.3910379910478,"simulated_s":120,"motor_rating_rule":"no c
- M3: {"heading_change_deg":360.08752447019,"altitude_dev_m":6.590790813422813,"mean_bank_deg":27.42341221024561,"max_bank_deg":45.788085786992745,"max_sideslip_deg":25.067310893166486,"turn_time_s":7.079166666666667,"nominal_passed":false,"robustness_passed":false,"robustness":{"aero0.9 mass0.95 cg-5mm":
- M4 rough air: 4/100 seeds. H 5 m: 2/23, worst altitude 98.7 m, roll 45 deg, pitch 38 deg; H 10 m: 1/19, worst altitude 109.7 m, roll 39 deg, pitch 38 deg; H 20 m: 0/30, worst altitude 110.0 m, roll 43 deg, pitch 39 deg; H 40 m: 1/28, worst altitude 130.4 m, roll 38 deg, pitch 39 deg.
- M7 structure: FoS 0.14; gears within rating false; twist servo 0.705 N m of 0.116 N m rated.
- M8 electrical: motor rms 13.40 A vs 6.50 A; battery 9.64 A vs 13.50 A; servo bus 1.82 A vs BEC 2 A.
- V5: {"protocol":"multipliers fitted on cruise and cruise x1.3 stroke; judged on four other points","earlier_protocol_note":"single-point fit gave 11.4% held-out lift error on columba-rp-2 (2026-11-02), replaced before falco and larus ran","held_out_points":4,"worst_lift_err_pct":29.07924374616733,"worst

Corridor at its real elevation (1444 m at the origin): best sustained climb -1.10 m/s at 6.5 m/s (2.2 A of 6.5 A); -0.43 m/s at 12.0 m/s (5.8 A of 6.5 A). Needed: 1.1-1.3 m/s at 12 m/s. Corridor flight completed false, worst vertical error 296.6 m, worst cross-track 64.0 m.

Refined designs and their quick checks (8 M4 seeds each):

| # | Mass g | Span | AR | Motor | Battery | M4 quick | Quick passes of 5 |
|---|---|---|---|---|---|---|---|
| 0 | 324 | 1.6 | 9 | motor-axi-2208-34-v3 | batt-gensace-300-3s-45c | 2/8 | 3 |
| 1 (chosen) | 327 | 1.6 | 9 | motor-axi-2208-26-v3 | batt-gensace-300-3s-45c | 1/8 | 4 |
| 2 | 328 | 1.7 | 10 | motor-axi-2208-34-v3 | batt-gensace-300-3s-45c | 1/8 | 1 |
| 3 | 329 | 1.6 | 9 | motor-eflite-park370-1360 | batt-gensace-300-3s-45c | 2/8 | 3 |
| 4 | 330 | 1.7 | 10 | motor-eflite-park370-1080 | batt-gensace-300-3s-45c | 1/8 | 3 |

## falco-pf-2

Screened 21168 parameterised designs from real parts in 1.1 s; refined 6 in the flight model.

Chosen: span 1.20 m, area 0.240 m2, AR 6.0, mass 412 g (54 g of it the 15% margin), wing loading 16.8 N/m2.
Motor motor-eflite-park480-910 (published continuous rating, 20.0 A), battery batt-gensace-800-3s-45c, ESC esc-hobbywing-skywalker-20a-v2-mini, twist servos servo-kst-x06-v6, gear ratio 10.0.
Cruise 9.0 m/s; rough-air speed 12.0 m/s.

| Gate | Passed | Key measured values |
|---|---|---|
| V5 | **FAIL** | held-out lift 12.0%, thrust 10.0% |
| M1 | **FAIL** | climb 2-3 s 0.33 m/s, lost 2.11 m; robust false |
| M2 | **FAIL** | 20.2 min, power ratio 9.45, cruise 23.4 W; robust false |
| M3 | **FAIL** | turned 360 deg, altitude 2.53 m; robust false |
| M4 | **FAIL** | 56/100 seeds, worst H 20 m; robust false |
| M5 | **FAIL** | sink 0.50 m/s, speed 8.45 m/s; robust false |
| M6 | pass | 412 g, 0 uncited parts |
| M7 | **FAIL** | FoS min 0.10, gears false, twist servo 0.612/0.116 N m |
| M8 | **FAIL** | motor 26.68/20.00 A, battery 25.21/36.00 A, servo bus 1.98/2.0 A |

What limits it:

- M1: {"release_speed_m_s":8,"release_pitch_deg":5,"climb_rate_2_to_3s_m_s":0.32658217056348926,"first_1s_window_climbing_s":2.529166666666667,"height_lost_m":2.108083210901494,"height_at_6s_m":-0.23639445394302322,"peak_motor_A":20.04400093530566,"peak_batt_A":11.05929497897564,"nominal_passed":false,"ro
- M2: {"cruise_power_w":23.361803384457136,"cruise_batt_A":1.8992213499711994,"endurance_min":20.218812304622794,"usable_fraction":0.8,"continuous_limit_A":20,"power_available_w":220.82881350085114,"power_ratio":9.452558514715134,"altitude_dev_m":0.6314432750399582,"simulated_s":120,"motor_rating_rule":"p
- M3: {"heading_change_deg":360.34440376882014,"altitude_dev_m":2.531037004984779,"mean_bank_deg":28.6866192792012,"max_bank_deg":40.747306670384404,"max_sideslip_deg":4.216883756861051,"turn_time_s":10.3625,"nominal_passed":true,"robustness_passed":false,"robustness":{"aero0.9 mass0.95 cg-5mm":true,"aero
- M4 rough air: 56/100 seeds. H 5 m: 14/23, worst altitude 25.2 m, roll 22 deg, pitch 34 deg; H 10 m: 13/19, worst altitude 31.4 m, roll 25 deg, pitch 32 deg; H 20 m: 15/30, worst altitude 30.9 m, roll 28 deg, pitch 36 deg; H 40 m: 14/28, worst altitude 46.6 m, roll 21 deg, pitch 30 deg.
- M5: {"touchdown_sink_m_s":0.49571633543835336,"touchdown_speed_m_s":8.454593250028857,"pitch_deg":0.030533304740300627,"roll_deg":0.1295317439392457,"time_s":21.341666666665777,"from_height_m":20,"nominal_passed":true,"robustness_passed":false,"robustness":{"aero0.9 mass0.95 cg-5mm":true,"aero0.9 mass0.
- M7 structure: FoS 0.10; gears within rating false; twist servo 0.612 N m of 0.116 N m rated.
- M8 electrical: motor rms 26.68 A vs 20.00 A; battery 25.21 A vs 36.00 A; servo bus 1.98 A vs BEC 2 A.
- V5: {"protocol":"multipliers fitted on cruise and cruise x1.3 stroke; judged on four other points","earlier_protocol_note":"single-point fit gave 11.4% held-out lift error on columba-rp-2 (2026-11-02), replaced before falco and larus ran","held_out_points":4,"worst_lift_err_pct":11.968942815395266,"wors

Corridor at its real elevation (1444 m at the origin): best sustained climb -0.17 m/s at 9.0 m/s (3.3 A of 20.0 A); 0.75 m/s at 12.0 m/s (5.1 A of 20.0 A). Needed: 1.1-1.3 m/s at 12 m/s. Corridor flight completed true, worst vertical error 68.6 m, worst cross-track 5.4 m.

Refined designs and their quick checks (8 M4 seeds each):

| # | Mass g | Span | AR | Motor | Battery | M4 quick | Quick passes of 5 |
|---|---|---|---|---|---|---|---|
| 0 | 488 | 1.2 | 7 | motor-eflite-park480-910 | batt-gensace-2200-3s-35c | 6/8 | 3 |
| 1 | 381 | 1.2 | 6 | motor-eflite-park480-910 | batt-gensace-450-3s-45c | 4/8 | 4 |
| 2 (chosen) | 412 | 1.2 | 6 | motor-eflite-park480-910 | batt-gensace-800-3s-45c | 6/8 | 4 |
| 3 | 518 | 1.2 | 7 | motor-eflite-park480-910 | batt-gensace-2200-3s-25c-gtech | 5/8 | 3 |
| 4 | 472 | 1.2 | 7 | motor-axi-2217-12-v3 | batt-gensace-2200-3s-35c | 4/8 | 3 |
| 5 | 440 | 1.2 | 6 | motor-eflite-park480-910 | batt-gensace-1000-3s-25c | 6/8 | 4 |

## columba-rp-2

Screened 21168 parameterised designs from real parts in 1.2 s; refined 6 in the flight model.

Chosen: span 0.90 m, area 0.147 m2, AR 5.5, mass 281 g (37 g of it the 15% margin), wing loading 18.7 N/m2.
Motor motor-eflite-park370-1360 (published continuous rating, 12.0 A), battery batt-gensace-300-3s-45c, ESC esc-hobbywing-skywalker-15a-v2, twist servos servo-kst-x06-v6, gear ratio 16.0.
Cruise 10.5 m/s; rough-air speed 13.0 m/s.

| Gate | Passed | Key measured values |
|---|---|---|
| V5 | **FAIL** | held-out lift 16.0%, thrust 7.3% |
| M1 | **FAIL** | climb 2-3 s -0.06 m/s, lost 5.73 m; robust false |
| M2 | **FAIL** | 6.3 min, power ratio 4.87, cruise 26.9 W; robust false |
| M3 | pass | turned 360 deg, altitude 1.01 m; robust true |
| M4 | **FAIL** | 37/100 seeds, worst H 40 m; robust false |
| M5 | **FAIL** | sink 0.95 m/s, speed 9.78 m/s; robust false |
| M6 | pass | 281 g, 0 uncited parts |
| M7 | **FAIL** | FoS min 0.16, gears false, twist servo 0.281/0.116 N m |
| M8 | **FAIL** | motor 16.10/12.00 A, battery 16.55/13.50 A, servo bus 1.72/2.0 A |

What limits it:

- M1: {"release_speed_m_s":8,"release_pitch_deg":5,"climb_rate_2_to_3s_m_s":-0.06285190551700381,"first_1s_window_climbing_s":3.0833333333333335,"height_lost_m":5.734617525360871,"height_at_6s_m":-2.2904607492863343,"peak_motor_A":17.961265350879508,"peak_batt_A":12.959112743949891,"nominal_passed":false,
- M2: {"cruise_power_w":26.882836362655933,"cruise_batt_A":2.2822483065259855,"endurance_min":6.309567613140014,"usable_fraction":0.8,"continuous_limit_A":12,"power_available_w":130.94818167089437,"power_ratio":4.871070146928393,"altitude_dev_m":0.776522669811225,"simulated_s":120,"motor_rating_rule":"pub
- M4 rough air: 37/100 seeds. H 5 m: 14/23, worst altitude 56.4 m, roll 22 deg, pitch 31 deg; H 10 m: 7/19, worst altitude 60.6 m, roll 32 deg, pitch 30 deg; H 20 m: 9/30, worst altitude 68.7 m, roll 22 deg, pitch 30 deg; H 40 m: 7/28, worst altitude 58.7 m, roll 21 deg, pitch 24 deg.
- M5: {"touchdown_sink_m_s":0.954271766560685,"touchdown_speed_m_s":9.784726651843988,"pitch_deg":-2.4333641782144535,"roll_deg":0.022466122377879233,"time_s":16.895833333332696,"from_height_m":20,"nominal_passed":false,"robustness_passed":false,"robustness":{"aero0.9 mass0.95 cg-5mm":false,"aero0.9 mass0
- M7 structure: FoS 0.16; gears within rating false; twist servo 0.281 N m of 0.116 N m rated.
- M8 electrical: motor rms 16.10 A vs 12.00 A; battery 16.55 A vs 13.50 A; servo bus 1.72 A vs BEC 2 A.
- V5: {"protocol":"multipliers fitted on cruise and cruise x1.3 stroke; judged on four other points","earlier_protocol_note":"single-point fit gave 11.4% held-out lift error on columba-rp-2 (2026-11-02), replaced before falco and larus ran","held_out_points":4,"worst_lift_err_pct":15.96049204112948,"worst

Corridor at its real elevation (1444 m at the origin): best sustained climb 0.32 m/s at 10.5 m/s (2.9 A of 12.0 A); 0.64 m/s at 12.0 m/s (4.4 A of 12.0 A). Needed: 1.1-1.3 m/s at 12 m/s. Corridor flight completed true, worst vertical error 70.8 m, worst cross-track 6.9 m.

Refined designs and their quick checks (8 M4 seeds each):

| # | Mass g | Span | AR | Motor | Battery | M4 quick | Quick passes of 5 |
|---|---|---|---|---|---|---|---|
| 0 | 362 | 0.9 | 4.5 | motor-eflite-park480-910 | batt-gensace-450-3s-45c | 4/8 | 3 |
| 1 | 393 | 0.9 | 4.5 | motor-eflite-park480-910 | batt-gensace-800-3s-45c | 5/8 | 2 |
| 2 | 375 | 0.9 | 4.5 | motor-axi-2217-12-v3 | batt-gensace-800-3s-45c | 5/8 | 2 |
| 3 | 344 | 0.9 | 4.5 | motor-axi-2217-12-v3 | batt-gensace-450-3s-45c | 6/8 | 2 |
| 4 | 403 | 0.9 | 4.5 | motor-axi-2217-12-v3 | batt-gensace-1000-3s-25c | 3/8 | 3 |
| 5 (chosen) | 281 | 0.9 | 5.5 | motor-eflite-park370-1360 | batt-gensace-300-3s-45c | 6/8 | 2 |

## Reading of the results

- No candidate passes the mission gates. M4 (rough air) is the hardest: the best is falco-pf-2, the heaviest wing loading, at 56/100 seeds; failures are altitude excursions (sinking air beyond the climb the stroke envelope gives) more than attitude.
- The KST X06 twist servo the screen chose is far too small: measured twist torque peaks are 0.6-0.7 N m against 0.116 N m at 5 V, so the servo slows (slew falls with load) and roll control degrades; this, not aerodynamics, is what collapses larus-gl-2 in M1-M4. The only eligible servo with more torque (Savox SH0255MG, 0.31 N m) is still below the peaks.
- M7 worst loads come from M4 seeds that depart; crank torque runs 2-3x the DS0.8-80 gear allowable and the 3 mm pultruded coupler buckles at the peak force. A spring balancer or a higher-module final gear, and a tube coupler, are the obvious structural changes.
- M8: motor currents exceed continuous ratings in rough air; the controller limits battery current, not motor current through the speed ripple of the flap cycle.
- Climb: every finalist is short of the corridor's 1.1-1.3 m/s. The limit is the controller's stroke envelope (frequency up to 2x trim, amplitude up to 1.35x trim), set to keep crank torque near the gear rating; current stays well below the limit.

## Unverified and assumed

- Film-over-rib drag factor 1.2 on the E387 and AG24 fits: no published source; LBM and OpenFOAM are to replace it.
- Tail section: assumed thin plate. Motor rotor inertia, battery internal resistance, ESC efficiency 0.95, avionics 0.35 A at 5 V: estimated, not published.
- Free-flight trim: the controller table is trimmed by flying the closed loop (the body recoils against heavy wings, so a frozen-body trim is optimistic).
- Dryden frozen-field turbulence is weak for a bird flying in a wind close to its own speed (flight-standards.md).
- UVLM wake is prescribed in V5; no free-wake roll-up. V2 against Prandtl lifting line fails below AR 12; see the gate file.
