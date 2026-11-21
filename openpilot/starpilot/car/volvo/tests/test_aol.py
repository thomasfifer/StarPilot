"""Volvo/Polestar CMA actual factory/Card/parser/controller/native AOL path.

The cruise double tap (off->on->off inside 0.5 s) is the MAIN press on these cars.
"""

import json
import os
from contextlib import contextmanager
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from opendbc.can import CANPacker
from opendbc.car import car_helpers, gen_empty_fingerprint, structs
from opendbc.car.volvo.tests.test_cma import CMA, packet, sources
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car import card as card_module
from openpilot.starpilot.aol.vehicle import policy_for
from openpilot.starpilot.car.volvo.aol import qualified, temporary_eps_restriction
from openpilot.starpilot.vehicle_selection import encode

VIN = 'YV1XZEFV0M2000001'
FRAME_NS = 10_000_000  # 100 Hz


@contextmanager
def factory(identity, *, enabled=True):
  with (
    OpenpilotPrefix(),
    patch.dict(os.environ, {'SIMULATION': '1', 'AOL_REPLAY_RUNTIME': '0', 'FINGERPRINT': '', 'SKIP_FW_QUERY': '', 'DISABLE_FW_CACHE': '1'}),
  ):
    os.environ.pop('REPLAY', None)
    params = Params()
    for key, value in [('AlwaysOnLateral', enabled), ('OpenpilotEnabledToggle', True), ('IsReleaseBranch', False),
                       ('AlphaLongitudinalEnabled', False), ('IsOffroad', False)]:
      params.put_bool(key, value, block=True)
    params.put('VehicleSelection', json.loads(encode(str(identity))), block=True)
    with (
      patch.object(card_module.messaging, 'recv_one_retry', return_value=SimpleNamespace(can=[1])),
      patch.object(card_module, 'can_comm_callbacks', return_value=(lambda *a, **kw: [], lambda *a, **kw: None)),
      patch.object(card_module, 'obd_callback', return_value=lambda _: None),
      patch.object(car_helpers, 'get_vin', return_value=(-1, -1, VIN)),
      patch.object(car_helpers, 'get_present_ecus', return_value=set()),
      patch.object(car_helpers, 'get_fw_versions_ordered', return_value=[]),
      patch.object(car_helpers, 'can_fingerprint', return_value=(None, gen_empty_fingerprint())),
    ):
      card = card_module.Car()
    try:
      assert card.CP.carFingerprint == identity
      yield card, params
    finally:
      card.vehicle_startup.close()


def frames(identity, frame, *, cruise, gear=3):
  result = [msg for msg in sources(identity, cruise=cruise, frame=frame) if msg[0] != 0x80]
  result.append(CANPacker('volvo_mid_1').make_can_msg('GEAR_POSITION', 0, {'GEAR_POSITION': gear}))
  return result


class Drive:
  """Feeds the real CarInterface (and optionally native safety) one 100 Hz frame at a time."""

  def __init__(self, card, *, native=None):
    self.card = card
    self.native = native
    self.frame = 0
    self.state = None

  def step(self, *, cruise, gear=3, request=1):
    self.frame += 1
    data = frames(str(self.card.CP.carFingerprint), self.frame, cruise=cruise, gear=gear)
    if self.native is not None:
      self.native.set_timer(self.frame * 10_000)
      for msg in data:
        assert self.native.safety_rx_hook(packet(msg)), msg
      self.native.safety_tick()
      self.native.aol_set_host_request(request)
    self.state = self.card.CI.update([(self.frame * FRAME_NS, data)])
    return self.state

  def hold(self, frames, **kwargs):
    for _ in range(frames):
      self.step(**kwargs)
    return self.state

  def tap(self):
    """off -> on (0.2 s) -> off: the arming double tap."""
    self.step(cruise=False)
    self.hold(20, cruise=True)
    return self.step(cruise=False)

  def intent(self):
    owner = self.card.aol_card_intent
    owner.update(self.state.as_reader(), now_ns=self.frame * FRAME_NS, fault_active=False)
    return owner


@pytest.mark.parametrize('identity', CMA)
def test_actual_factory_enrolls_volvo_aol(identity):
  with factory(identity) as (card, params):
    assert card.aol_qualified and card.aol_card_intent is not None
    assert policy_for(card.CP).normal_runtime_supported
    assert card.CP.alternativeExperience == 32
    assert card.CP.safetyConfigs[0].safetyParam == 0
    assert not card.CP.openpilotLongitudinalControl and card.CP.pcmCruise
    with structs.CarParams.from_bytes(params.get('CarParams')) as published:
      assert published.to_dict() == card.CP.to_dict()


def test_c1_has_no_aol_port():
  with factory('VOLVO_V40') as (card, _):
    assert not card.aol_qualified and card.aol_card_intent is None
    assert card.CP.alternativeExperience == 0
    assert not policy_for(card.CP).intent_supported


@pytest.mark.parametrize('identity', CMA)
def test_explicit_off_retains_upstream(identity):
  with factory(identity, enabled=False) as (card, params):
    assert card.aol_card_intent is None and not card.aol_qualified
    assert card.CP.alternativeExperience == 0
    assert params.get('AlwaysOnLateral') is False
    # Without the AOL marking, availability is unconditional as before.
    drive = Drive(card)
    assert drive.hold(5, cruise=False).cruiseState.available


@pytest.mark.parametrize('identity', CMA)
def test_intent_follows_the_double_tap_through_the_real_parser(identity):
  with factory(identity) as (card, _):
    drive = Drive(card)
    state = drive.hold(20, cruise=False)
    assert state.canValid and not state.cruiseState.available
    assert not drive.intent().allowed_latch

    # A slow press is a normal engagement attempt, not an arming tap.
    drive.hold(80, cruise=True)
    drive.intent()
    state = drive.step(cruise=False)
    assert not state.cruiseState.available and not drive.intent().allowed_latch

    state = drive.tap()
    assert state.cruiseState.available and not state.cruiseState.enabled
    assert drive.intent().allowed_latch

    # Reverse keeps the intent; Drive output resumes without a new tap.
    drive.step(cruise=False, gear=1)
    owner = drive.intent()
    assert owner.allowed_latch and not owner.output(drive.state)[0]
    drive.step(cruise=False)
    assert drive.intent().allowed_latch

    # A normal engagement keeps it latched; the cancel afterwards ends it.
    drive.hold(20, cruise=True)
    assert drive.intent().allowed_latch
    drive.step(cruise=False)
    assert not drive.intent().allowed_latch

    # A fresh tap re-arms.
    drive.tap()
    assert drive.intent().allowed_latch


@pytest.mark.parametrize('identity', CMA)
def test_current_native_and_real_controller_boundaries(identity):
  with factory(identity) as (card, _):
    cp = card.CP
    safety = libsafety_py.libsafety
    safety.init_tests()
    safety.set_alternative_experience(32)
    assert safety.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, cp.safetyConfigs[0].safetyParam) == 0
    safety.set_aol_test_heartbeat(True)
    try:
      _native_boundaries(card, safety)
    finally:
      # libsafety is shared by every test in the process: don't leak the AOL marking.
      safety.set_alternative_experience(0)
      safety.set_aol_test_heartbeat(False)


def _native_boundaries(card, safety):
  drive = Drive(card, native=safety)
  drive.hold(20, cruise=False)
  assert safety.aol_get_permission_mask() == 0

  drive.tap()
  assert drive.state.cruiseState.available
  assert safety.aol_get_permission_mask() == 1
  assert not safety.get_controls_allowed()

  # The real controller's lateral output passes native TX on the armed grant alone.
  sent = set()
  for _ in range(10):
    drive.step(cruise=False)
    command = structs.CarControl(enabled=False, latActive=bool(safety.aol_get_permission_mask() & 1))
    command.actuators.steeringAngleDeg = 0
    _, sends = card.CI.apply(command.as_reader(), drive.frame * FRAME_NS)
    for msg in sends:
      assert safety.safety_tx_hook(packet(msg)), msg
      sent.add(msg[0])
  assert 0x67 in sent  # LCA_5 angle command

  # Reverse withdraws permission; Drive restores it while still armed.
  drive.step(cruise=False, gear=1)
  assert safety.aol_get_permission_mask() == 0
  drive.step(cruise=False)
  assert safety.aol_get_permission_mask() == 1

  # A normal engagement grants both axes, its cancel withdraws everything.
  drive.hold(5, cruise=True, request=3)
  assert safety.aol_get_permission_mask() == 3
  drive.step(cruise=False, request=3)
  assert safety.aol_get_permission_mask() == 0

  # Host request lease expiry withdraws permission.
  drive.tap()
  assert safety.aol_get_permission_mask() == 1
  safety.set_timer(drive.frame * 10_000 + 300_001)
  assert safety.aol_get_permission_mask() == 0


@pytest.mark.parametrize('identity', CMA)
def test_exact_owner_denies_other_configs(identity):
  with factory(identity) as (card, _):
    cp = card.CP
    assert qualified(cp, marked_only=True)
    for field in ('passive', 'dashcamOnly', 'notCar', 'openpilotLongitudinalControl'):
      changed = structs.CarParams(**cp.to_dict())
      setattr(changed, field, True)
      assert not qualified(changed), field
    changed = structs.CarParams(**cp.to_dict())
    changed.flags = 1
    assert not qualified(changed)
    changed = structs.CarParams(**cp.to_dict())
    changed.safetyConfigs[0].safetyParam = 2
    assert not qualified(changed)
    changed = structs.CarParams(**cp.to_dict())
    changed.alternativeExperience = 0
    assert qualified(changed) and not qualified(changed, marked_only=True)


@pytest.mark.parametrize('identity', CMA)
def test_temporary_eps_retention_rejects_other_vehicle_conditions(identity):
  with factory(identity) as (card, _):
    drive = Drive(card)
    drive.hold(20, cruise=False)
    state = structs.CarState(**drive.tap().to_dict())
    assert state.cruiseState.available and not state.steerFaultTemporary
    state.steerFaultTemporary = True  # PSCM_RELATED reported an unsupported status
    assert temporary_eps_restriction(card.CP, state)
    for field, value in [('canValid', False), ('canTimeout', True), ('steerFaultTemporary', False),
                         ('steerFaultPermanent', True), ('accFaulted', True), ('brakePressed', True),
                         ('gasPressed', True), ('regenBraking', True), ('doorOpen', True),
                         ('seatbeltUnlatched', True), ('gearShifter', structs.CarState.GearShifter.reverse)]:
      bad = structs.CarState(**state.to_dict())
      setattr(bad, field, value)
      assert not temporary_eps_restriction(card.CP, bad), field
    for field in ('available', 'enabled'):
      bad = structs.CarState(**state.to_dict())
      setattr(bad.cruiseState, field, field == 'enabled')
      assert not temporary_eps_restriction(card.CP, bad), field
