"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time
from dataclasses import dataclass

from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.common.params import Params
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import PCM_LONG_REQUIRED_MAX_SET_SPEED, CONFIRM_SPEED_THRESHOLD
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import compare_cluster_target, set_speed_limit_assist_availability

ButtonType = car.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName
SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source
MessageKind = custom.LongitudinalPlanSP.SpeedLimit.MessageKind

ACTIVE_STATES = (SpeedLimitAssistState.active, SpeedLimitAssistState.adapting)
ENABLED_STATES = (SpeedLimitAssistState.preActive, SpeedLimitAssistState.pending, *ACTIVE_STATES)

DISABLED_GUARD_PERIOD = 0.5  # secs.
# secs. Time to wait after activation before considering temp deactivation signal.
PRE_ACTIVE_GUARD_PERIOD = {
  True: 15,
  False: 8,  # Fork: 8 s to tap +/- (was 5)
}
SPEED_LIMIT_CHANGED_HOLD_PERIOD = 1  # secs. Time to wait after speed limit change before switching to preActive.

LIMIT_MIN_ACC = -1.5  # m/s^2 Maximum deceleration allowed for limit controllers to provide.
LIMIT_MAX_ACC = 1.0   # m/s^2 Maximum acceleration allowed for limit controllers to provide while active.
LIMIT_MIN_SPEED = 8.33  # m/s, Minimum speed limit to provide as solution on limit controllers.
LIMIT_SPEED_OFFSET_TH = -1.  # m/s Maximum offset between speed limit and current speed for adapting state.
V_CRUISE_UNSET = 255.

CRUISE_BUTTONS_PLUS = (ButtonType.accelCruise, ButtonType.resumeCruise)
CRUISE_BUTTONS_MINUS = (ButtonType.decelCruise, ButtonType.setCruise)
CRUISE_BUTTON_CONFIRM_HOLD = 0.5  # secs.

# Fork: when a new limit is applied by itself and when the driver is asked to confirm it with a +/- tap.
# The camera reads almost every sign but sometimes the wrong one; the map is often right but has gaps.
#   - the map backs up the sign (same limit here, or at a change point just ahead/behind): apply
#   - the map gives a different limit here: ask
#   - the map has nothing: apply between highway limits (both >= 55 mph / 90 km/h) and for increases from
#     an arterial limit (>= 45 mph / 70 km/h); ask for everything else (drops onto slower roads, low limits)
HIGHWAY_LIMIT = {True: 90, False: 55}   # km/h, mph
REPEAT_ANNOUNCE_QUIET = 120.  # s, the same limit isn't announced again within this
ARTERIAL_LIMIT = {True: 70, False: 45}  # km/h, mph

# Fork (v34): what the pop-up says is decided here, from what Speed Limit Assist is doing, not re-guessed in the UI
# from the map data (which said 'Slowing for 55' on a 50 -> 55 early step, and 'set to 50' on its way to 60).
ANNOUNCE_REPLACE_TIME = 3.  # s: a pop-up this soon after the previous one replaces it without a second chime
SLOWING_MARGIN = 0.5        # m/s (~1 mph): 'slowing' only when the target is below the car's speed by this much
ANNOUNCE_DEFER_MAX = 30.    # s: the easing's pop-up waits at most this long for the car to start slowing
V_CRUISE_VALID_MAX = 70.    # m/s: above this the cluster set speed is unset (255 km/h)


@dataclass
class SpeedLimitMessage:
  kind: int = MessageKind.none
  limit: float = 0.      # m/s
  target: float = 0.     # m/s
  direction: int = 0     # -1 slowing, +1 speeding up, 0 the set speed changes but the car's speed doesn't
  silent: bool = False


class SpeedLimitAssist:
  _speed_limit_final_last: float
  _distance: float
  v_ego: float
  a_ego: float
  v_offset: float

  def __init__(self, CP: car.CarParams, CP_SP: custom.CarParamsSP):
    self.params = Params()
    self.CP = CP
    self.CP_SP = CP_SP
    self.frame = -1
    self.long_engaged_timer = 0
    self.pre_active_timer = 0
    self.is_metric = self.params.get_bool("IsMetric")
    set_speed_limit_assist_availability(self.CP, self.CP_SP, self.params)
    self.enabled = self.params.get("SpeedLimitMode", return_default=True) == Mode.assist
    self.long_enabled = False
    self.long_enabled_prev = False
    self.is_enabled = False
    self.is_active = False
    self.output_v_target = V_CRUISE_UNSET
    self.output_a_target = 0.
    self.v_ego = 0.
    self.a_ego = 0.
    self.v_offset = 0.
    self.target_set_speed_conv = 0
    self.prev_target_set_speed_conv = 0
    self.v_cruise_cluster = 0.
    self.v_cruise_cluster_prev = 0.
    self.v_cruise_cluster_conv = 0
    self.prev_v_cruise_cluster_conv = 0
    self._has_speed_limit = False
    self._speed_limit = 0.
    self._speed_limit_final_last = 0.
    self.speed_limit_prev = 0.
    self.speed_limit_final_last_conv = 0
    self.prev_speed_limit_final_last_conv = 0
    self._distance = 0.
    self.state = SpeedLimitAssistState.disabled
    self._state_prev = SpeedLimitAssistState.disabled
    self.pcm_op_long = CP.openpilotLongitudinalControl and CP.pcmCruise
    self._quiet_change = False
    self._raise_step = False  # Fork: the target is the early step toward a higher limit ahead
    self._sign_expected = False  # Fork: the sign confirms what the easing / early step was heading for: no announcement
    self._map_agrees = False
    self._map_conflict = False
    self._sign_prev = 0.
    self._unconfirmed = False  # Fork: a limit that was asked about and not answered
    self._last_announced: tuple[int, float] | None = None
    self._ahead_limit = 0.
    self._limit_last = 0.
    self.message = SpeedLimitMessage()
    self._message_time = -1e9
    self._deferred: float | None = None  # time an easing pop-up became due, while waiting for the car to slow

    self._plus_hold = 0.
    self._minus_hold = 0.
    self._release_toggle_prev = 0

    # TODO-SP: SLA's own output_a_target for planner
    # Solution functions mapped to respective states
    self.acceleration_solutions = {
      SpeedLimitAssistState.disabled: self.get_current_acceleration_as_target,
      SpeedLimitAssistState.inactive: self.get_current_acceleration_as_target,
      SpeedLimitAssistState.preActive: self.get_current_acceleration_as_target,
      SpeedLimitAssistState.pending: self.get_current_acceleration_as_target,
      SpeedLimitAssistState.adapting: self.get_adapting_state_target_acceleration,
      SpeedLimitAssistState.active: self.get_active_state_target_acceleration,
    }

  @property
  def speed_limit_changed(self) -> bool:
    return self._has_speed_limit and bool(self._speed_limit != self.speed_limit_prev)

  @property
  def v_cruise_cluster_changed(self) -> bool:
    return bool(self.v_cruise_cluster_conv != self.prev_v_cruise_cluster_conv)

  @property
  def target_set_speed_confirmed(self) -> bool:
    return bool(self.v_cruise_cluster_conv == self.target_set_speed_conv)

  @property
  def v_cruise_cluster_below_confirm_speed_threshold(self) -> bool:
    return bool(self.v_cruise_cluster_conv < CONFIRM_SPEED_THRESHOLD[self.is_metric])

  def update_active_event(self, events_sp: EventsSP, reactivation: bool = False) -> None:
    # Fork: on re-engaging (e.g. every cruise resume in town), don't repeat what was just announced; a change always is
    now = time.monotonic()
    if reactivation and self._last_announced is not None and self._last_announced[0] == self.speed_limit_final_last_conv and \
       now - self._last_announced[1] < REPEAT_ANNOUNCE_QUIET:
      return
    # Fork (v34): nor when the set speed is already where it goes: nothing changes, so nothing to say
    if reactivation and self.v_cruise_cluster_conv == self.speed_limit_final_last_conv:
      self._last_announced = (self.speed_limit_final_last_conv, now)
      return
    message = self._compose_message()
    # Fork (v34): a limit ahead that changes nothing yet (the set speed is already there) isn't worth a pop-up
    if message.kind != MessageKind.sign and message.direction == 0 and \
       round(message.target * (CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH)) == self.v_cruise_cluster_conv:
      self._last_announced = (self.speed_limit_final_last_conv, now)
      return
    # Fork (v34): the easing toward a lower limit ahead starts from the set speed, which can be well above the car's
    # speed (Nov 3: 'Slowing for 25' and 8 s more at 47 mph). Say it when the car actually starts to slow.
    if message.kind == MessageKind.lowerAhead and self._speed_limit_final_last > self.v_ego + SLOWING_MARGIN:
      if self._deferred is None:
        self._deferred = now
      return
    self._announce(events_sp, message, now)

  def _compose_message(self, silent: bool = False) -> SpeedLimitMessage:
    """Fork (v34): what the pop-up says: the limit (here or ahead), the set speed it goes to, and whether that
    slows the car down, speeds it up, or only changes the set speed."""
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    limit_here = self._speed_limit if self._speed_limit > 0. else self._limit_last
    offset = self._speed_limit_final_last - limit_here if limit_here > 0. else 0.
    if self._ahead_limit > 0. and self._raise_step:
      kind, limit, target = MessageKind.higherAhead, self._ahead_limit, self._speed_limit_final_last
    elif self._ahead_limit > 0.:
      # the easing steps the set speed down to the limit ahead (plus the offset) by the sign
      kind, limit, target = MessageKind.lowerAhead, self._ahead_limit, self._ahead_limit + offset
    else:
      kind, limit, target = MessageKind.sign, limit_here, self._speed_limit_final_last

    target_conv = round(target * speed_conv)
    set_valid = 0. < self.v_cruise_cluster < V_CRUISE_VALID_MAX
    direction = 0
    if target < self.v_ego - SLOWING_MARGIN and (not set_valid or target_conv < self.v_cruise_cluster_conv):
      direction = -1
    elif target > self.v_ego + SLOWING_MARGIN and set_valid and target_conv > self.v_cruise_cluster_conv:
      direction = 1
    return SpeedLimitMessage(kind, limit, target, direction, silent)

  def _announce(self, events_sp: EventsSP, message: SpeedLimitMessage, now: float) -> None:
    self._deferred = None
    if now - self._message_time < ANNOUNCE_REPLACE_TIME:
      message.silent = True
    self.message = message
    self._message_time = now
    self._last_announced = (self.speed_limit_final_last_conv, now)
    if self.v_cruise_cluster_below_confirm_speed_threshold:
      events_sp.add(EventNameSP.speedLimitChanged)
    else:
      events_sp.add(EventNameSP.speedLimitActive)

  def get_v_target_from_control(self) -> float:
    if self._has_speed_limit:
      if self.pcm_op_long and self.is_enabled:
        return self._speed_limit_final_last
      if not self.pcm_op_long and self.is_active:
        return self._speed_limit_final_last

    # Fallback
    return V_CRUISE_UNSET

  # TODO-SP: SLA's own output_a_target for planner
  def get_a_target_from_control(self) -> float:
    return self.a_ego

  def update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.is_metric = self.params.get_bool("IsMetric")
      set_speed_limit_assist_availability(self.CP, self.CP_SP, self.params)
      self.enabled = self.params.get("SpeedLimitMode", return_default=True) == Mode.assist

  def update_buttons(self, release_toggle: int) -> None:
    released = self._release_toggle_prev ^ release_toggle
    self._release_toggle_prev = release_toggle
    if not released:
      return
    now = time.monotonic()
    if any((released >> b) & 1 for b in CRUISE_BUTTONS_PLUS):
      self._plus_hold = max(self._plus_hold, now + CRUISE_BUTTON_CONFIRM_HOLD)
    if any((released >> b) & 1 for b in CRUISE_BUTTONS_MINUS):
      self._minus_hold = max(self._minus_hold, now + CRUISE_BUTTON_CONFIRM_HOLD)

  def _get_button_release(self, req_plus: bool, req_minus: bool) -> bool:
    now = time.monotonic()
    if req_plus and now <= self._plus_hold:
      self._plus_hold = 0.
      return True
    elif req_minus and now <= self._minus_hold:
      self._minus_hold = 0.
      return True

    # expired
    if now > self._plus_hold:
      self._plus_hold = 0.
    if now > self._minus_hold:
      self._minus_hold = 0.
    return False

  def update_calculations(self, v_cruise_cluster: float) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    self.v_cruise_cluster = v_cruise_cluster

    # Update current velocity offset (error)
    self.v_offset = self._speed_limit_final_last - self.v_ego

    self.speed_limit_final_last_conv = round(self._speed_limit_final_last * speed_conv)
    self.v_cruise_cluster_conv = round(self.v_cruise_cluster * speed_conv)

    cst_low, cst_high = PCM_LONG_REQUIRED_MAX_SET_SPEED[self.is_metric]
    pcm_long_required_max = cst_low if self._has_speed_limit and self.speed_limit_final_last_conv < CONFIRM_SPEED_THRESHOLD[self.is_metric] else \
                            cst_high
    pcm_long_required_max_set_speed_conv = round(pcm_long_required_max * speed_conv)

    self.target_set_speed_conv = pcm_long_required_max_set_speed_conv if self.pcm_op_long else self.speed_limit_final_last_conv

  @property
  def apply_confirm_speed_threshold(self) -> bool:
    """True when the driver should confirm the new limit with a +/- tap, False to apply it by itself.
    See HIGHWAY_LIMIT above for the rules."""
    if self._map_agrees:
      return False
    if self._map_conflict:
      return True
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    posted = round(self._speed_limit * speed_conv)
    prev = round(self._sign_prev * speed_conv)
    if posted >= HIGHWAY_LIMIT[self.is_metric] and prev >= HIGHWAY_LIMIT[self.is_metric]:
      return False
    if posted > prev >= ARTERIAL_LIMIT[self.is_metric]:
      return False
    return True

  def get_current_acceleration_as_target(self) -> float:
    return self.a_ego

  def get_adapting_state_target_acceleration(self) -> float:
    if self._distance > 0:
      return (self._speed_limit_final_last ** 2 - self.v_ego ** 2) / (2. * self._distance)

    return self.v_offset / float(ModelConstants.T_IDXS[CONTROL_N])

  def get_active_state_target_acceleration(self) -> float:
    return self.v_offset / float(ModelConstants.T_IDXS[CONTROL_N])

  def _update_confirmed_state(self):
    if self._has_speed_limit:
      if self.v_offset < LIMIT_SPEED_OFFSET_TH:
        self.state = SpeedLimitAssistState.adapting
      else:
        self.state = SpeedLimitAssistState.active
    else:
      self.state = SpeedLimitAssistState.pending

  def _update_non_pcm_long_confirmed_state(self) -> bool:
    if self.target_set_speed_confirmed:
      return True

    if self.state != SpeedLimitAssistState.preActive:
      return False

    req_plus, req_minus = compare_cluster_target(self.v_cruise_cluster, self._speed_limit_final_last, self.is_metric)

    return self._get_button_release(req_plus, req_minus)

  def update_state_machine_pcm_op_long(self):
    self.long_engaged_timer = max(0, self.long_engaged_timer - 1)
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    # ACTIVE, ADAPTING, PENDING, PRE_ACTIVE, INACTIVE
    if self.state != SpeedLimitAssistState.disabled:
      if not self.long_enabled or not self.enabled:
        self.state = SpeedLimitAssistState.disabled

      else:
        # ACTIVE
        if self.state == SpeedLimitAssistState.active:
          if self.v_cruise_cluster_changed:
            self.state = SpeedLimitAssistState.inactive
          elif self.speed_limit_changed and self.apply_confirm_speed_threshold:
            self.state = SpeedLimitAssistState.preActive
            self.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD[self.pcm_op_long] / DT_MDL)
          elif self._has_speed_limit and self.v_offset < LIMIT_SPEED_OFFSET_TH:
            self.state = SpeedLimitAssistState.adapting

        # ADAPTING
        elif self.state == SpeedLimitAssistState.adapting:
          if self.v_cruise_cluster_changed:
            self.state = SpeedLimitAssistState.inactive
          elif self.speed_limit_changed and self.apply_confirm_speed_threshold:
            self.state = SpeedLimitAssistState.preActive
            self.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD[self.pcm_op_long] / DT_MDL)
          elif self.v_offset >= LIMIT_SPEED_OFFSET_TH:
            self.state = SpeedLimitAssistState.active

        # PENDING
        elif self.state == SpeedLimitAssistState.pending:
          if self.target_set_speed_confirmed:
            self._update_confirmed_state()
          elif self.speed_limit_changed:
            self.state = SpeedLimitAssistState.preActive
            self.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD[self.pcm_op_long] / DT_MDL)

        # PRE_ACTIVE
        elif self.state == SpeedLimitAssistState.preActive:
          if self.target_set_speed_confirmed:
            self._update_confirmed_state()
          elif self.pre_active_timer <= 0:
            # Timeout - session ended
            self.state = SpeedLimitAssistState.inactive

        # INACTIVE
        elif self.state == SpeedLimitAssistState.inactive:
          pass

    # DISABLED
    elif self.state == SpeedLimitAssistState.disabled:
      if self.long_enabled and self.enabled:
        # start or reset preActive timer if initially enabled or manual set speed change detected
        if not self.long_enabled_prev or self.v_cruise_cluster_changed:
          self.long_engaged_timer = int(DISABLED_GUARD_PERIOD / DT_MDL)

        elif self.long_engaged_timer <= 0:
          if self.target_set_speed_confirmed:
            self._update_confirmed_state()
          elif self._has_speed_limit:
            self.state = SpeedLimitAssistState.preActive
            self.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD[self.pcm_op_long] / DT_MDL)
          else:
            self.state = SpeedLimitAssistState.pending

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def update_state_machine_non_pcm_long(self):
    self.long_engaged_timer = max(0, self.long_engaged_timer - 1)
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    # ACTIVE, ADAPTING, PENDING, PRE_ACTIVE, INACTIVE
    if self.state != SpeedLimitAssistState.disabled:
      if not self.long_enabled or not self.enabled:
        self.state = SpeedLimitAssistState.disabled

      else:
        # ACTIVE
        if self.state == SpeedLimitAssistState.active:
          self._unconfirmed = False
          # Fork: a set-speed change that lands exactly on our own target is ours (button management following
          # the limit), not the driver's. Stock dropped to inactive and re-confirmed a frame later, which is
          # harmless for one step but broke on the map easing, whose target moves every half second.
          if self.v_cruise_cluster_changed and not self.target_set_speed_confirmed:
            self.state = SpeedLimitAssistState.inactive

          elif self.speed_limit_changed and self.apply_confirm_speed_threshold:
            self.state = SpeedLimitAssistState.preActive
            self.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD[self.pcm_op_long] / DT_MDL)

        # PRE_ACTIVE
        elif self.state == SpeedLimitAssistState.preActive:
          # Fork: also accept by itself if the map catches up with the sign while asking
          if self._update_non_pcm_long_confirmed_state() or not self.apply_confirm_speed_threshold:
            self.state = SpeedLimitAssistState.active
          elif self.pre_active_timer <= 0:
            # Timeout - session ended
            self.state = SpeedLimitAssistState.inactive
            self._unconfirmed = True

        # INACTIVE
        elif self.state == SpeedLimitAssistState.inactive:
          if self.v_cruise_cluster_changed:
            self._unconfirmed = False  # the driver set a speed: that's the answer
          # Fork: after the driver overrode the set speed, the map easing's 1 mph steps don't ask to be
          # confirmed; the next sign the camera reads is applied or asked about like any other. The early step
          # toward a higher limit is left alone too: the driver's own set speed stands until the sign.
          if self.speed_limit_changed and not self._quiet_change and not self._raise_step:
            if self.apply_confirm_speed_threshold:
              self.state = SpeedLimitAssistState.preActive
              self.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD[self.pcm_op_long] / DT_MDL)
            else:
              self.state = SpeedLimitAssistState.active
          # Fork: an unanswered limit is applied once the map backs it up (e.g. a freeway limit read on the on-ramp,
          # before the map has the car on the freeway)
          elif self._unconfirmed and not self.apply_confirm_speed_threshold and not self._raise_step:
            self.state = SpeedLimitAssistState.active
          elif self._update_non_pcm_long_confirmed_state():
            self.state = SpeedLimitAssistState.active

    # DISABLED
    elif self.state == SpeedLimitAssistState.disabled:
      if self.long_enabled and self.enabled:
        # start or reset preActive timer if initially enabled or manual set speed change detected
        if not self.long_enabled_prev or self.v_cruise_cluster_changed:
          self.long_engaged_timer = int(DISABLED_GUARD_PERIOD / DT_MDL)

        elif self.long_engaged_timer <= 0:
          if self._update_non_pcm_long_confirmed_state():
            self.state = SpeedLimitAssistState.active
          elif self._has_speed_limit:
            # Fork: on engaging, a limit that would be applied by itself is applied (announced), else asked
            if self.apply_confirm_speed_threshold:
              self.state = SpeedLimitAssistState.preActive
              self.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD[self.pcm_op_long] / DT_MDL)
            else:
              self.state = SpeedLimitAssistState.active
          else:
            self.state = SpeedLimitAssistState.inactive

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def update_events(self, events_sp: EventsSP) -> None:
    quiet = self._quiet_change
    if self.state == SpeedLimitAssistState.preActive:
      events_sp.add(EventNameSP.speedLimitPreActive)

    if self.state == SpeedLimitAssistState.pending and self._state_prev != SpeedLimitAssistState.pending:
      events_sp.add(EventNameSP.speedLimitPending)

    if not self.is_active:
      self._deferred = None
    elif self._deferred is not None:
      # Fork (v34): the easing's pop-up waiting for the car to start slowing
      now = time.monotonic()
      if self._ahead_limit <= 0. or self._raise_step:
        self._deferred = None  # the easing ended (the camera read the sign, or let go) before it slowed the car
      elif self._speed_limit_final_last <= self.v_ego + SLOWING_MARGIN or now - self._deferred > ANNOUNCE_DEFER_MAX:
        self._announce(events_sp, self._compose_message(), now)

    if self.is_active:
      if self._state_prev not in ACTIVE_STATES:
        self.update_active_event(events_sp, reactivation=not self.speed_limit_changed)

      # only notify if we acquire a valid speed limit
      # do not check has_speed_limit here
      # Fork: not on each 1 mph step of the map easing (it announced itself once when it started), and not for the
      # sign that confirms the limit the easing or the early step was already heading for
      elif self._speed_limit != self.speed_limit_prev and self._sign_expected:
        # counts as announced (the easing / early step said it), so a cruise resume soon after doesn't repeat it
        self._last_announced = (self.speed_limit_final_last_conv, time.monotonic())
        self._deferred = None
        # Fork (v34): the early step went part of the way (55 -> 70: to 65); the sign takes the set speed the rest.
        # Shown, but without a chime: the earlier pop-up already said this limit was coming.
        if self.speed_limit_final_last_conv > self.v_cruise_cluster_conv and self._speed_limit_final_last > self.v_ego + SLOWING_MARGIN:
          self._announce(events_sp, self._compose_message(silent=True), time.monotonic())
      elif self._speed_limit != self.speed_limit_prev and not quiet:
        if self.speed_limit_prev <= 0:
          self.update_active_event(events_sp)
        elif self.speed_limit_prev > 0 and self._speed_limit > 0:
          self.update_active_event(events_sp)

  def update(self, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, v_cruise_cluster: float, speed_limit: float,
             speed_limit_final_last: float, has_speed_limit: bool, distance: float, events_sp: EventsSP,
             quiet_change: bool = False, map_agrees: bool = False, map_conflict: bool = False, sign_prev: float = 0.,
             raise_step: bool = False, sign_expected: bool = False, ahead_limit: float = 0.) -> None:
    self.long_enabled = long_enabled
    self._quiet_change = quiet_change
    self._raise_step = raise_step
    self._sign_expected = sign_expected
    self._ahead_limit = ahead_limit
    self._map_agrees = map_agrees
    self._map_conflict = map_conflict
    self._sign_prev = sign_prev
    self.v_ego = v_ego
    self.a_ego = a_ego

    self._has_speed_limit = has_speed_limit
    self._speed_limit = speed_limit
    if speed_limit > 0.:
      self._limit_last = speed_limit
    self._speed_limit_final_last = speed_limit_final_last
    self._distance = distance

    self.update_params()
    self.update_calculations(v_cruise_cluster)

    self._state_prev = self.state
    if self.pcm_op_long:
      self.is_enabled, self.is_active = self.update_state_machine_pcm_op_long()
    else:
      self.is_enabled, self.is_active = self.update_state_machine_non_pcm_long()

    self.update_events(events_sp)

    # Update change tracking variables
    self.speed_limit_prev = self._speed_limit
    self.v_cruise_cluster_prev = self.v_cruise_cluster
    self.long_enabled_prev = self.long_enabled
    self.prev_target_set_speed_conv = self.target_set_speed_conv
    self.prev_v_cruise_cluster_conv = self.v_cruise_cluster_conv
    self.prev_speed_limit_final_last_conv = self.speed_limit_final_last_conv

    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    self.frame += 1
