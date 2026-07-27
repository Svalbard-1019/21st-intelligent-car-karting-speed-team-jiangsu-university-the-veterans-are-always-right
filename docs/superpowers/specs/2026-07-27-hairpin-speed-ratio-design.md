# Portion1 Hairpin Speed Ratio Design

## Goal

Raise only the portion1 hairpin speed ratio from `0.45` to `0.60`.

At the tested menu speeds, the hairpin center-speed targets become:

- 3.5 m/s menu speed: 2.10 m/s
- 3.8 m/s menu speed: 2.28 m/s
- 4.2 m/s menu speed: 2.52 m/s

## Scope

Change `GUANDAO_HAIRPIN_SPEED_RATIO` in `code/guandao.c`.

Do not add a fixed minimum-speed clamp. Do not change the ordinary curve,
sharp-turn, accumulated-turn, final approach, parking, reverse, or active-brake
rules. Final approach logic remains after curve limiting and may still command
less than the calculated hairpin speed.

## Verification

Update the portion1 speed-planner source test to require the `0.60` ratio and
to keep the final-approach ordering intact. Run the focused test first, then
the complete host test suite.
