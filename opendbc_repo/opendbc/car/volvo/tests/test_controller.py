from types import SimpleNamespace

import pytest

from opendbc.can.parser import CANParser
from opendbc.car import Bus
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.helpers import checksum_lca_5_message
from opendbc.car.volvo.interface import CarInterface
from opendbc.car.volvo.values import CAR, DBC
from opendbc.car.volvo.volvocan import create_c1_checksum
from opendbc.safety.tests.libsafety import libsafety_py


def _stock_messages(dbc, names):
  """Message dicts as carstate passes them: a CANParser fills in every DBC signal (zero until a frame arrives)."""
  parser = CANParser(dbc, [(name, 0) for name in names], 0)
  return {name: parser.vl[name] for name in names}


def _state():
  msgs = _stock_messages(DBC[CAR.POLESTAR_2][Bus.party], ["LCA", "PSCM", "PSCM_RELATED", "LCA_2", "LCA_3", "LCA_4", "LCA_5"])
  return SimpleNamespace(
    out=SimpleNamespace(steeringAngleDeg=0.0, vEgoRaw=12.0, steeringTorque=0.0),
    msg_lca=msgs["LCA"],
    msg_pscm=msgs["PSCM"],
    msg_pscm_related=msgs["PSCM_RELATED"],
    msg_lca_3=msgs["LCA_3"],
    msg_lca_2=msgs["LCA_2"],
    msg_lca_5=msgs["LCA_5"],
    msg_lca_4=msgs["LCA_4"],
    pilot_assist_engaged=False,
  )


class _Actuators:
  steeringAngleDeg = 30.0

  def as_builder(self):
    return SimpleNamespace(steeringAngleDeg=self.steeringAngleDeg)


def test_controller_emits_valid_eight_byte_messages_and_lca5_checksum():
  cp = CarInterface.get_non_essential_params("POLESTAR_2")
  controller = CarController(DBC[cp.carFingerprint], cp)
  cs = _state()
  cc = SimpleNamespace(latActive=True, actuators=_Actuators())

  actuators, can_sends = controller.update(cc, cs, 0, None)

  assert can_sends
  assert {msg[2] for msg in can_sends} == {0, 2}
  assert all(len(msg[1]) == 8 for msg in can_sends)
  assert 0.0 < actuators.steeringAngleDeg < 540.0

  lca5 = next(msg for msg in can_sends if msg[0] == 0x67)
  data = lca5[1]
  assert data[2] == checksum_lca_5_message(data[0], data[1], data[3], data[4], data[5])


@pytest.mark.parametrize("fingerprint", [CAR.POLESTAR_2, CAR.VOLVO_XC40_RECHARGE])
def test_controller_leaves_lca5_to_panda_forwarding_when_inactive(fingerprint):
  # While inactive, the controller must not TX an LCA_5 substitute at all: panda only
  # blocks the stock frame while openpilot is recently sending one (see
  # volvo_fwd_hook), so the real stock frame flows through untouched.
  cp = CarInterface.get_non_essential_params(fingerprint)
  controller = CarController(DBC[cp.carFingerprint], cp)
  cs = _state()
  cc = SimpleNamespace(latActive=False, actuators=_Actuators())

  for _ in range(4):
    _, can_sends = controller.update(cc, cs, 0, None)
    assert not any(msg[0] == 0x67 for msg in can_sends)

  # Once active, the stock counter is passed straight through (LCA_5 is 50 Hz, so
  # step until it's sent)
  cs.msg_lca_5["COUNTER"] = 7
  cc.latActive = True
  sent = []
  for _ in range(2):
    _, can_sends = controller.update(cc, cs, 0, None)
    sent += [msg for msg in can_sends if msg[0] == 0x67]
  assert len(sent) == 1
  data = sent[0][1]
  assert data[3] >> 4 == 7
  assert data[2] == checksum_lca_5_message(data[0], data[1], data[3], data[4], data[5])


@pytest.mark.parametrize("fingerprint", [CAR.POLESTAR_2, CAR.VOLVO_XC40_RECHARGE])
@pytest.mark.parametrize("driver_torque", [20.0, -20.0, 128.0, -127.0])
def test_override_lca_stream_passes_safety_and_recovers(fingerprint, driver_torque):
  cp = CarInterface.get_non_essential_params(fingerprint)
  controller = CarController(DBC[cp.carFingerprint], cp)
  cs = _state()
  cc = SimpleNamespace(latActive=True, actuators=_Actuators())
  safety = libsafety_py.libsafety
  config = cp.safetyConfigs[0]
  assert safety.set_safety_hooks(config.safetyModel.raw, config.safetyParam) == 0
  safety.init_tests()
  safety.set_controls_allowed(True)

  for active, torque in [(True, 0), (True, driver_torque), (True, -driver_torque),
                        (True, 0), (False, 0), (True, 0)]:
    cc.latActive = active
    cs.out.steeringTorque = torque
    for _ in range(350):
      _, messages = controller.update(cc, cs, 0, None)
      lca_msgs = [msg for msg in messages if msg[0] == 0x58]
      if active:
        address, data, bus = lca_msgs[0]
        assert safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data)), (
          active, torque, controller.frame, controller.lca_auth.pos, controller.lca_auth.neg)
      else:
        # Inactive: the controller leaves LCA to panda's forwarding (volvo_fwd_hook),
        # it must not also TX a substitute.
        assert not lca_msgs
    if active and torque == 0:
      assert controller.lca_auth.pos == 614
      assert controller.lca_auth.neg == -614
    elif active:
      assert min(abs(controller.lca_auth.pos), abs(controller.lca_auth.neg)) == 0


def _c1_state():
  return SimpleNamespace(
    out=SimpleNamespace(steeringAngleDeg=10.0, vEgo=12.0, vEgoRaw=12.0),
    c1_lka_torque=5,
    c1_msg_pscm=_stock_messages(DBC[CAR.VOLVO_V40][Bus.pt], ["PSCM1"])["PSCM1"],
  )


def test_c1_controller_emits_checked_steering_and_pscm_relay():
  cp = CarInterface.get_non_essential_params(CAR.VOLVO_V40)
  controller = CarController(DBC[cp.carFingerprint], cp)
  cc = SimpleNamespace(
    latActive=True,
    actuators=_Actuators(),
    cruiseControl=SimpleNamespace(cancel=False),
  )

  actuators, can_sends = controller.update(cc, _c1_state(), 0, None)
  assert [(msg[0], msg[2]) for msg in can_sends] == [(0x125, 2), (0xD0, 0)]

  fsm = can_sends[1][1]
  assert fsm[7] & 0x3 == 3
  assert fsm[6] == create_c1_checksum(fsm)
  assert 0.0 < actuators.steeringAngleDeg <= 2.0


def test_c1_controller_sends_only_cancel_button():
  cp = CarInterface.get_non_essential_params(CAR.VOLVO_V40)
  controller = CarController(DBC[cp.carFingerprint], cp)
  cc = SimpleNamespace(
    latActive=False,
    actuators=_Actuators(),
    cruiseControl=SimpleNamespace(cancel=True),
  )

  _, can_sends = controller.update(cc, _c1_state(), 0, None)
  buttons = next(msg for msg in can_sends if msg[0] == 0x10)
  assert buttons[2] == 0
  assert buttons[1][7] == 0x10
  assert buttons[1][6] == 0


def test_c1_controller_temporarily_drops_steering_on_zero_torque_fault():
  cp = CarInterface.get_non_essential_params(CAR.VOLVO_V40)
  controller = CarController(DBC[cp.carFingerprint], cp)
  cs = _c1_state()
  cs.c1_lka_torque = 0
  cc = SimpleNamespace(
    latActive=True,
    actuators=_Actuators(),
    cruiseControl=SimpleNamespace(cancel=False),
  )

  directions = []
  for _ in range(23):
    _, can_sends = controller.update(cc, cs, 0, None)
    directions.extend(msg[1][7] & 0x3 for msg in can_sends if msg[0] == 0xD0)

  assert directions[:-1] == [3] * 11
  assert directions[-1] == 0

  while controller.frame <= 122:
    _, can_sends = controller.update(cc, cs, 0, None)
  fsm = next(msg for msg in can_sends if msg[0] == 0xD0)
  assert fsm[1][7] & 0x3 == 3
