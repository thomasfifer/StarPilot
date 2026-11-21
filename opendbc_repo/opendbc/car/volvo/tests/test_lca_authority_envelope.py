import pytest

from opendbc.car.volvo.helpers import LcaAuthorityEnvelope
from opendbc.car.volvo.values import CarControllerParams as P

DT = 0.01  # the envelope runs once per 100 Hz controller frame
REBUILD_STEP = P.LCA_AUTH_REBUILD_RATE * DT


def _run(env, torque, frames):
  for _ in range(frames):
    pos, neg = env.update(torque)
  return pos, neg


def _saturated():
  env = LcaAuthorityEnvelope()
  frames_to_saturate = int(P.LCA_AUTH_MAX / REBUILD_STEP) + 2
  assert _run(env, 0.0, frames_to_saturate) == (P.LCA_AUTH_MAX, -P.LCA_AUTH_MAX)
  return env


def test_rebuilds_from_zero_to_saturation():
  env = LcaAuthorityEnvelope()
  pos, neg = _run(env, 0.0, 100)
  assert pos == pytest.approx(100 * REBUILD_STEP)
  assert neg == pytest.approx(-100 * REBUILD_STEP)
  _saturated()


@pytest.mark.parametrize("torque", [20.0, -20.0])
def test_sustained_push_yields_toward_driver_and_holds_counter_arm(torque):
  # Well above the override threshold: the yield arm (the side the driver pushes
  # toward) collapses to YIELD_MIN, the counter arm holds at the plateau.
  env = _saturated()
  pos, neg = _run(env, torque, 500)
  if torque > 0:  # pushing right yields the + arm
    assert pos == pytest.approx(P.LCA_AUTH_YIELD_MIN)
    assert neg == pytest.approx(-P.LCA_AUTH_PLATEAU_COUNTER)
  else:
    assert pos == pytest.approx(P.LCA_AUTH_PLATEAU_COUNTER)
    assert neg == pytest.approx(-P.LCA_AUTH_YIELD_MIN)


def test_release_rebuilds_at_rebuild_rate():
  env = _saturated()
  pos0, neg0 = _run(env, 20.0, 500)
  pos, neg = _run(env, 0.0, 10)
  assert pos == pytest.approx(pos0 + 10 * REBUILD_STEP)
  assert neg == pytest.approx(neg0 - 10 * REBUILD_STEP)


def test_override_latch_hysteresis():
  between = (P.LCA_AUTH_OVERRIDE_ENTER + P.LCA_AUTH_OVERRIDE_EXIT) / 2
  assert P.LCA_AUTH_OVERRIDE_EXIT < between < P.LCA_AUTH_OVERRIDE_ENTER

  env = _saturated()
  _run(env, P.LCA_AUTH_OVERRIDE_ENTER + 1, 50)  # enter override

  # Dropping between EXIT and ENTER stays latched: no rebuild toward saturation.
  for _ in range(100):
    pos, neg = env.update(between)
    assert pos <= P.LCA_AUTH_YIELD_BASE
    assert neg == pytest.approx(-P.LCA_AUTH_PLATEAU_COUNTER)

  # Below EXIT the latch releases and both arms rebuild.
  pos_latched, neg_latched = pos, neg
  below = P.LCA_AUTH_OVERRIDE_EXIT - 1
  assert below < P.LCA_AUTH_LIGHT_THRESH  # so this isn't also a light-contact edge
  pos, neg = _run(env, below, 50)
  assert pos == pytest.approx(pos_latched + 50 * REBUILD_STEP)
  assert neg == pytest.approx(neg_latched - 50 * REBUILD_STEP)


def test_light_contact_gives_brief_dip_without_latching():
  # Resting hands: a step to a torque above LIGHT_THRESH but below the override
  # threshold. Authority dips for LIGHT_HOLD_FRAMES, then rebuilds and doesn't
  # re-arm while the torque stays constant.
  light = P.LCA_AUTH_OVERRIDE_ENTER - 1
  assert P.LCA_AUTH_LIGHT_THRESH < light < P.LCA_AUTH_OVERRIDE_ENTER
  step = P.LCA_AUTH_COLLAPSE_RATE * max(1.0, light / P.LCA_AUTH_OVERRIDE_ENTER) * DT
  dipped = P.LCA_AUTH_MAX - P.LCA_AUTH_LIGHT_HOLD_FRAMES * step
  assert dipped > P.LCA_AUTH_SPLIT  # stays in the symmetric phase for this check

  env = _saturated()
  pos, neg = _run(env, light, P.LCA_AUTH_LIGHT_HOLD_FRAMES)
  assert pos == pytest.approx(dipped)
  assert neg == pytest.approx(-dipped)

  prev = pos
  for _ in range(100):
    pos, neg = env.update(light)
    assert pos > prev  # rebuilding, never dipping again
    assert neg == pytest.approx(-pos)
    prev = pos
  assert pos == pytest.approx(dipped + 100 * REBUILD_STEP)


def test_reset_starts_over_from_zero():
  env = _saturated()
  _run(env, 20.0, 500)
  env.reset()
  assert (env.pos, env.neg) == (0.0, 0.0)
  pos, neg = env.update(0.0)
  assert pos == pytest.approx(REBUILD_STEP)
  assert neg == pytest.approx(-REBUILD_STEP)
