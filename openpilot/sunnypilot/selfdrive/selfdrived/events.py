"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import EventsBase, Priority, ET, Alert, \
  NoEntryAlert, ImmediateDisableAlert, EngagementAlert, NormalPermanentAlert, AlertCallbackType, wrong_car_mode_alert
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import PCM_LONG_REQUIRED_MAX_SET_SPEED, CONFIRM_SPEED_THRESHOLD
from openpilot.common.hardware import HARDWARE

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
AudibleAlertSP = custom.SelfdriveStateSP.AudibleAlert
EventNameSP = custom.OnroadEventSP.EventName


# get event name from enum
EVENT_NAME_SP = {v: k for k, v in EventNameSP.schema.enumerants.items()}

IS_MICI = HARDWARE.get_device_type() == 'mici'


def speed_limit_adjust_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  speedLimit = sm['longitudinalPlanSP'].speedLimit.resolver.speedLimit
  speed = round(speedLimit * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH))
  message = f'Limit {speed}'  # Fork (v34): short enough for the big font
  return Alert(
    message,
    "",
    AlertStatus.normal, AlertSize.small,
    Priority.LOW, VisualAlert.none, AudibleAlert.none, 4.)


SpeedLimitSourceSP = custom.LongitudinalPlanSP.SpeedLimit.Source
MessageKindSP = custom.LongitudinalPlanSP.SpeedLimit.MessageKind


def _speed_limit_facts(sm: messaging.SubMaster, metric: bool) -> dict:
  """Fork: the numbers behind a speed-limit message: the sign, what the map says, the set speed it goes to."""
  conv = CV.MS_TO_KPH if metric else CV.MS_TO_MPH
  res = sm['longitudinalPlanSP'].speedLimit.resolver
  facts = {'limit': round(res.speedLimitLast * conv), 'final': round(res.speedLimitFinalLast * conv),
           'offset': round(res.speedLimitOffset * conv), 'easing': res.source == SpeedLimitSourceSP.map,
           'map': 0, 'ahead': 0}
  try:
    lmd = sm['liveMapDataSP']
    if lmd.speedLimitValid:
      facts['map'] = round(lmd.speedLimit * conv)
    if lmd.speedLimitAheadValid:
      facts['ahead'] = round(lmd.speedLimitAhead * conv)
  except (KeyError, AttributeError):
    pass
  return facts


def speed_limit_applied_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int,
                              personality) -> Alert:
  """Fork (v34): short, and it says which way the car goes: 'Limit 45 / slowing to 50', 'Limit 55 / speeding up to 60',
  '25 ahead / slowing to 30' (the map easing), '55 ahead / speeding up to 60' (the early step), 'Limit 55 / set to 60'
  when the set speed changes but the car's speed won't (already below it), 'Limit 55' alone when nothing changes.
  Speed Limit Assist decides all of it (speedLimit.assist.message); the UI only words it."""
  conv = CV.MS_TO_KPH if metric else CV.MS_TO_MPH
  m = sm['longitudinalPlanSP'].speedLimit.assist.message
  if m.kind == MessageKindSP.none:  # not from this fork's planner: say what is known
    f = _speed_limit_facts(sm, metric)
    kind, limit, target, direction, silent = MessageKindSP.sign, f['limit'], f['final'], 0, False
  else:
    kind, limit, target, direction, silent = m.kind, round(m.limit * conv), round(m.target * conv), m.direction, m.silent

  set_now = round(CS.vCruiseCluster * (1. if metric else CV.KPH_TO_MPH)) if 0. < CS.vCruiseCluster < 250. else 0
  text1 = f"{limit} ahead" if kind in (MessageKindSP.lowerAhead, MessageKindSP.higherAhead) else f"Limit {limit}"
  if direction < 0:
    text2 = f"slowing to {target}"
  elif direction > 0:
    # the early step toward a higher limit ahead goes only part of the way (55 -> 70: to 65, then 75 at the sign):
    # no number that looks like a mistake next to '70 ahead'
    offset = round(sm['longitudinalPlanSP'].speedLimit.resolver.speedLimitOffset * conv)
    partial = kind == MessageKindSP.higherAhead and target < limit + offset
    text2 = "speeding up" if partial else f"speeding up to {target}"
  elif target != set_now:
    text2 = f"set to {target}"
  else:
    text2 = ""
  sound = AudibleAlert.none if silent else AudibleAlertSP.promptSingleHigh
  return Alert(text1, text2, AlertStatus.normal, AlertSize.mid if text2 else AlertSize.small,
               Priority.LOW, VisualAlert.none, sound, 5.)


def speed_limit_pre_active_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  speed_conv = CV.MS_TO_KPH if metric else CV.MS_TO_MPH
  v_cruise_cluster = CS.vCruiseCluster
  set_speed = sm['controlsState'].deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
  # Fork: the set speed is in km/h; stock multiplied it by the m/s conversion (x2.24 in mph), so the prompt could say
  # "tap -" when the dash was below the new target
  set_speed_conv = round(set_speed * (1. if metric else CV.KPH_TO_MPH))

  speed_limit_final_last = sm['longitudinalPlanSP'].speedLimit.resolver.speedLimitFinalLast
  speed_limit_final_last_conv = round(speed_limit_final_last * speed_conv)
  alert_1_str = ""
  alert_size = AlertSize.small

  if CP.openpilotLongitudinalControl and CP.pcmCruise:
    # PCM long
    cst_low, cst_high = PCM_LONG_REQUIRED_MAX_SET_SPEED[metric]
    pcm_long_required_max = cst_low if speed_limit_final_last_conv < CONFIRM_SPEED_THRESHOLD[metric] else cst_high
    pcm_long_required_max_set_speed_conv = round(pcm_long_required_max * speed_conv)
    speed_unit = "km/h" if metric else "mph"

    alert_1_str = f"Speed Limit Assist: set to {pcm_long_required_max_set_speed_conv} {speed_unit} to engage"
  else:
    # Fork: 'Limit 35? / tap - for 40' (the question mark is the hint that it isn't sure)
    f = _speed_limit_facts(sm, metric)
    alert_2_str = ""
    if set_speed_conv != speed_limit_final_last_conv:
      tap = "+" if set_speed_conv < speed_limit_final_last_conv else "-"
      alert_1_str = f"Limit {f['limit']}?"
      alert_2_str = f"tap {tap} for {speed_limit_final_last_conv}"
    alert_size = AlertSize.mid if alert_1_str else AlertSize.none
    return Alert(
      alert_1_str,
      alert_2_str,
      AlertStatus.normal, alert_size,
      Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleLow, .1)

  return Alert(
    alert_1_str,
    "",
    AlertStatus.normal, alert_size,
    Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleLow, .1)


class EventsSP(EventsBase):
  def __init__(self):
    super().__init__()
    self.event_counters = dict.fromkeys(EVENTS_SP.keys(), 0)

  def get_events_mapping(self) -> dict[int, dict[str, Alert | AlertCallbackType]]:
    return EVENTS_SP

  def get_event_name(self, event: int):
    return EVENT_NAME_SP[event]

  def get_event_msg_type(self):
    return custom.OnroadEventSP.Event


EVENTS_SP: dict[int, dict[str, Alert | AlertCallbackType]] = {
  # sunnypilot
  EventNameSP.lkasEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },

  EventNameSP.lkasDisable: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
  },

  EventNameSP.manualSteeringRequired: {
    ET.USER_DISABLE: Alert(
      "Steering off",
      "you steer",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.disengage, 1.),
  },

  EventNameSP.manualLongitudinalRequired: {
    ET.WARNING: Alert(
      "Cruise off",
      "you control speed",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1.),
  },

  EventNameSP.silentLkasEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.none),
  },

  EventNameSP.silentLkasDisable: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.none),
  },

  EventNameSP.silentBrakeHold: {
    ET.WARNING: EngagementAlert(AudibleAlert.none),
    ET.NO_ENTRY: NoEntryAlert("Brake Hold Active"),
  },

  EventNameSP.silentWrongGear: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: Alert(
      "Shift to D",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 0.),
  },

  EventNameSP.silentReverseGear: {
    ET.PERMANENT: Alert(
      "Reverse\nGear",
      "",
      AlertStatus.normal, AlertSize.full,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2, creation_delay=0.5),
    ET.NO_ENTRY: NoEntryAlert("Reverse Gear"),
  },

  EventNameSP.silentDoorOpen: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: NoEntryAlert("Door Open"),
  },

  EventNameSP.silentSeatbeltNotLatched: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: NoEntryAlert("Seatbelt Unlatched"),
  },

  EventNameSP.silentParkBrake: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: NoEntryAlert("Parking Brake Engaged"),
  },

  EventNameSP.controlsMismatchLateral: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Controls Mismatch: Lateral"),
    ET.NO_ENTRY: NoEntryAlert("Controls Mismatch: Lateral"),
  },

  EventNameSP.experimentalModeSwitched: {
    ET.WARNING: NormalPermanentAlert("Experimental Mode Switched", duration=1.5)
  },

  EventNameSP.wrongCarModeAlertOnly: {
    ET.WARNING: wrong_car_mode_alert,
  },

  EventNameSP.pedalPressedAlertOnly: {
    ET.WARNING: NoEntryAlert("Pedal Pressed")
  },

  EventNameSP.laneTurnLeft: {
    ET.WARNING: Alert(
      "Turning Left",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1.),
  },

  EventNameSP.laneTurnRight: {
    ET.WARNING: Alert(
      "Turning Right",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1.),
  },

  EventNameSP.speedLimitActive: {
    ET.WARNING: speed_limit_applied_alert,
  },

  EventNameSP.speedLimitChanged: {
    ET.WARNING: speed_limit_applied_alert,
  },

  EventNameSP.speedLimitPreActive: {
    ET.WARNING: speed_limit_pre_active_alert,
  },

  EventNameSP.speedLimitPending: {
    ET.WARNING: Alert(
      "Using last limit",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleHigh, 5.),
  },

  EventNameSP.e2eChime: {
    ET.PERMANENT: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.MID, VisualAlert.none, AudibleAlert.prompt, 3.),
  },

  EventNameSP.laneChangeRoadEdge: {
    ET.WARNING: Alert(
      "No lane change",
      "road edge",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, 0.1),
  },

  EventNameSP.laneChangeOff: {
    ET.PERMANENT: Alert(
      "Lane changes off",
      "hold LKA to undo",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, 2.5),
  },

  EventNameSP.laneChangeOn: {
    ET.PERMANENT: Alert(
      "Lane changes on",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, 2.5),
  },

  EventNameSP.bigModelReady: {
    ET.PERMANENT: Alert(
      "Big Model Ready",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, 2.),
  },
}
