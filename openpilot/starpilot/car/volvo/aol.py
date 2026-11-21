"""Volvo AOL, one native profile for two receiver layouts.

SPA (XC40 Recharge, S60 Recharge, Polestar 2, safetyParam 0): there is no MAIN switch.
An off->on->off tap of stock cruise inside 0.5 s arms lateral-only control (carstate.py
and volvo_cma.h detect it independently), and any later cruise edge disarms it.
CarState reports that armed state, or a normal cruise engagement, as
cruiseState.available, which the intent follows like MAIN.

V60 (EUCD, safetyParam 8): lateral follows the car's LKA on/off when the LKAS button is
mapped, else the ACC main.

C1 (V40) has its own MAIN and no AOL profile.
"""

from opendbc.car import structs
from opendbc.car.volvo.values import CAR, VolvoSafetyFlags
from openpilot.starpilot.aol.intent import AolCardIntent, AOL_TOGGLE, ButtonType
from openpilot.starpilot.aol.policy import AolVehiclePolicy

# Cars on the SPA receiver layout.
SPA_CARS = frozenset((CAR.VOLVO_XC40_RECHARGE, CAR.VOLVO_S60_RECHARGE, CAR.POLESTAR_2))
VOLVO_SPA_SAFETY_PARAM = 0
VOLVO_EUCD = VolvoSafetyFlags.EUCD.value
_DRIVE = (structs.CarState.GearShifter.drive,)


def _qualified(cp, *, marked_only, cars, safety_param, stock_longitudinal_only):
  try:
    if (
      cp.brand != 'volvo'
      or cp.carFingerprint not in cars
      or cp.flags != 0
      or cp.passive
      or cp.notCar
      or cp.dashcamOnly
      or not cp.pcmCruise
      or (stock_longitudinal_only and cp.openpilotLongitudinalControl)
      or cp.steerControlType != structs.CarParams.SteerControlType.angle
      or len(cp.safetyConfigs) != 1
      or cp.alternativeExperience not in ((32,) if marked_only else (0, 32))
    ):
      return False
    config = cp.safetyConfigs[0]
    return config.safetyModel == structs.CarParams.SafetyModel.volvo and config.safetyParam == safety_param
  except (AttributeError, TypeError, ValueError, IndexError):
    return False


def spa_qualified(cp, *, marked_only=False):
  return _qualified(cp, marked_only=marked_only, cars=SPA_CARS, safety_param=VOLVO_SPA_SAFETY_PARAM, stock_longitudinal_only=True)


def eucd_qualified(cp, *, marked_only=False):
  return _qualified(cp, marked_only=marked_only, cars=(CAR.VOLVO_V60,), safety_param=VOLVO_EUCD, stock_longitudinal_only=False)


def qualified(cp, *, marked_only=False):
  return spa_qualified(cp, marked_only=marked_only) or eucd_qualified(cp, marked_only=marked_only)


def policy_for(cp):
  admitted = qualified(cp)
  return AolVehiclePolicy(
    intent_supported=admitted,
    settings_supported=admitted,
    runtime_supported=admitted,
    normal_runtime_supported=admitted,
    ordinary_axis_ack_required=qualified(cp, marked_only=True),
    explicit_latch=admitted,
    # SPA has no LKAS button; the V60's LKAS button toggles the car's LKA.
    lkas_button_supported=not spa_qualified(cp),
    alternative_experience_addition=32 if admitted else 0,
  )


def native_profile_supported(model, param):
  return model == int(structs.CarParams.SafetyModel.volvo) and param in (VOLVO_SPA_SAFETY_PARAM, VOLVO_EUCD)


def native_accepts_cp(cp, model, param):
  return qualified(cp, marked_only=True) and native_profile_supported(model, param) and param == cp.safetyConfigs[0].safetyParam


def ordinary_axis_request_allowed(cp, state):
  return qualified(cp, marked_only=True) and state.canValid and not state.canTimeout and not state.steerFaultPermanent


def temporary_restriction(cp, state):
  if state is None or not state.canValid or state.canTimeout or state.steerFaultPermanent:
    return False
  if spa_qualified(cp, marked_only=True):
    return bool(state.gearShifter not in _DRIVE or state.steerFaultTemporary)
  if eucd_qualified(cp, marked_only=True):
    return bool(state.gearShifter not in _DRIVE or state.steerFaultTemporary or state.vehicleSensorsInvalid or state.invalidLkasSetting)
  return False


def native_latch_rejected(cp, native):
  return qualified(cp, marked_only=True) and native.requestedLateral and not native.lateralAllowed


def retain_on_native_denial(cp, native, state):
  return temporary_restriction(cp, state)


def native_observation(native, *, latched, panda_ready, restricted, now_ns, pending_since_ns):
  lost = not panda_ready or native is None
  if not latched or lost or restricted:
    return 0, lost, False
  if native.requestedLateral and native.lateralAllowed:
    return 0, False, False
  pending_since_ns = pending_since_ns or now_ns
  return pending_since_ns, False, now_ns - pending_since_ns >= 200_000_000


def temporary_eps_restriction(cp, state):
  """SPA: armed LAT-only PSCM status withdrawal (PSCM_RELATED); physical disarms stay authoritative."""
  return bool(spa_qualified(cp, marked_only=True) and state is not None and state.canValid and not state.canTimeout and
    state.steerFaultTemporary and not state.steerFaultPermanent and not state.accFaulted and
    state.cruiseState.available and not state.cruiseState.enabled and
    state.gearShifter == structs.CarState.GearShifter.drive and
    not state.brakePressed and not state.gasPressed and not state.regenBraking and
    not state.doorOpen and not state.seatbeltUnlatched and
    not state.vehicleSensorsInvalid and not state.steeringDisengage and not state.buttonEvents)


class VolvoCardIntent(AolCardIntent):
  """SPA: a fresh cruise double tap is this car's MAIN press."""
  observe_stock_engagement = True

  def __init__(self, cp, settings):
    self.main_seen = False
    self.main_available = False
    self.main_pulse = False
    self.needs_rearm = False
    super().__init__(settings, explicit_latch=True, lkas_button_supported=False)

  def _perform(self, action):
    if action == AOL_TOGGLE and self.settings.enabled:
      self.allowed_latch = not self.allowed_latch
      self.needs_rearm = not self.allowed_latch
      if self.allowed_latch:
        self.pause_lateral = False
      else:
        self._record_latch_clear('configured_toggle')
        self._withdraw_full_engagement()
    else:
      super()._perform(action)

  def _arm_from_full_engagement(self, CS, *, source_ns):
    super()._arm_from_full_engagement(CS, source_ns=source_ns)
    self.needs_rearm = False

  def update(self, CS, **kwargs):
    saved = self.settings
    automatic_main = saved.main_action == 0
    state = structs.CarState(**CS.to_dict())
    healthy = bool(
      CS.canValid
      and not CS.canTimeout
      and not CS.steerFaultPermanent
      and not CS.accFaulted
      and kwargs.get('fault_active') is not True
      and not kwargs.get('native_rejection_ns', 0)
    )
    was_latched = self.allowed_latch
    if not healthy and (was_latched or CS.steerFaultPermanent or CS.accFaulted):
      self.needs_rearm = True
    available = bool(CS.cruiseState.available)
    if healthy and not available:
      self.needs_rearm = False
    events = [structs.CarState.ButtonEvent(**event.to_dict()) for event in CS.buttonEvents]
    if self.main_pulse:
      events.append(structs.CarState.ButtonEvent(type=ButtonType.mainCruise, pressed=False))
      self.main_pulse = False
    if healthy:
      # A fresh double tap is this car's MAIN press.
      if self.main_seen and available and not self.main_available:
        if saved.main_action == AOL_TOGGLE:
          events.append(structs.CarState.ButtonEvent(type=ButtonType.mainCruise, pressed=True))
          self.main_pulse = True
        elif saved.main_action:
          self._perform(saved.main_action)
      self.main_seen = True
      self.main_available = available
    else:
      self.main_seen = False
    state.buttonEvents = events
    super().update(state.as_reader(), **kwargs)
    if was_latched and not self.allowed_latch:
      self.needs_rearm = True
    if not available:
      self.allowed_latch = False
    elif (
      automatic_main
      and healthy
      and saved.enabled
      and not self.needs_rearm
      and not self.pause_lateral
      and not self._fault_inhibit
      and not self._calibration_inhibit
      and not self._calibration_rearm_required
    ):
      # Follow the armed state, including boot with it already armed. This is
      # intent only: current native/vehicle permission still gates CAN.
      self.allowed_latch = True


class VolvoEucdCardIntent(AolCardIntent):
  observe_stock_engagement = True

  def __init__(self, settings):
    self.lka_on = False
    self.cruise_enabled = False
    super().__init__(settings, explicit_latch=True)

  def observe_vehicle_state(self, car_state):
    self.lka_on = bool(getattr(car_state, 'eucd_lka_on', False))

  def update(self, CS, **kwargs):
    saved = self.settings
    lka_mode = saved.lkas_action == AOL_TOGGLE
    healthy = bool(
      CS.canValid
      and not CS.canTimeout
      and not CS.steerFaultPermanent
      and not CS.accFaulted
      and kwargs.get('fault_active') is not True
      and not kwargs.get('native_rejection_ns', 0)
    )
    state = structs.CarState(**CS.to_dict())
    events = []
    for event in CS.buttonEvents:
      if event.type != ButtonType.lkas:
        events.append(structs.CarState.ButtonEvent(**event.to_dict()))
      elif lka_mode and event.pressed:
        # Each press flips the LKA, already read: turning it off while engaged pauses steering, turning it on resumes
        if CS.cruiseState.enabled or self.pause_lateral:
          self.pause_lateral = not self.lka_on
          if self.pause_lateral:
            self._withdraw_full_engagement()
      elif not lka_mode and event.pressed and healthy:
        self._perform(saved.lkas_action)
    state.buttonEvents = events
    # A pause by the LKA ends with the next ACC engage
    if lka_mode and CS.cruiseState.enabled and not self.cruise_enabled:
      self.pause_lateral = False
    self.cruise_enabled = bool(CS.cruiseState.enabled)
    super().update(state.as_reader(), **kwargs)

    # Latch follows the car's LKA, or an ACC engage until the LKA pauses it (route 00000007, 296 s)
    engaged_steering = lka_mode and CS.cruiseState.enabled and not self.pause_lateral
    desired = (self.lka_on or engaged_steering) if lka_mode else bool(CS.cruiseState.available)
    if not desired:
      if self.allowed_latch:
        self._record_latch_clear('vehicle_state_off')
      self.allowed_latch = False
    elif (healthy and saved.enabled and not self.pause_lateral and not self._fault_inhibit and
          not self._calibration_inhibit and not self._calibration_rearm_required):
      self.allowed_latch = True


def create_intent(cp, settings):
  return VolvoCardIntent(cp, settings) if spa_qualified(cp) else VolvoEucdCardIntent(settings)
