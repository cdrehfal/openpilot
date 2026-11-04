"""Fork: the speed-limit pop-ups say what is happening, briefly, with the numbers (v34 wording)."""
from unittest import mock
from openpilot.cereal import custom
from openpilot.common.constants import CV
from opendbc.car.structs import car
import openpilot.sunnypilot.selfdrive.selfdrived.events as ev

Source = custom.LongitudinalPlanSP.SpeedLimit.Source
Kind = custom.LongitudinalPlanSP.SpeedLimit.MessageKind
MPH = CV.MPH_TO_MS


def sm(limit=55, final=60, set_mph=None, message=None):
  lp = custom.LongitudinalPlanSP.new_message()
  r = lp.speedLimit.resolver
  r.speedLimitLast = limit * MPH
  r.speedLimitFinalLast = final * MPH
  r.speedLimitOffset = 5 * MPH
  r.source = Source.car
  if message is not None:
    kind, lim, target, direction, silent = message
    m = lp.speedLimit.assist.message
    m.kind, m.limit, m.target, m.direction, m.silent = kind, lim * MPH, target * MPH, direction, silent
  lmd = custom.LiveMapDataSP.new_message()
  msgs = {'longitudinalPlanSP': lp.as_reader(), 'liveMapDataSP': lmd.as_reader(), 'controlsState': mock.MagicMock()}
  s = mock.MagicMock()
  s.__getitem__.side_effect = lambda k: msgs[k]
  cs = car.CarState.new_message(vCruiseCluster=(set_mph or 0) * CV.MPH_TO_KPH).as_reader()
  return s, cs


def applied(message, set_mph=60):
  s, cs = sm(set_mph=set_mph, message=message)
  return ev.speed_limit_applied_alert(CP, cs, s, False, 0, None)


def texts(alert):
  return alert.alert_text_1, alert.alert_text_2


CP = car.CarParams.new_message(openpilotLongitudinalControl=False, pcmCruise=True).as_reader()


class TestSpeedLimitMessages:
  def test_sign_slowing_and_speeding_up(self):
    assert texts(applied((Kind.sign, 45, 50, -1, False))) == ("Limit 45", "slowing to 50")
    assert texts(applied((Kind.sign, 65, 70, 1, False))) == ("Limit 65", "speeding up to 70")

  def test_limits_ahead(self):
    assert texts(applied((Kind.lowerAhead, 25, 30, -1, False))) == ("25 ahead", "slowing to 30")
    assert texts(applied((Kind.higherAhead, 55, 60, 1, False))) == ("55 ahead", "speeding up to 60")

  def test_early_step_part_of_the_way_has_no_number(self):
    # 55 here, 70 ahead: the early step goes to 65 (it said '70 ahead / set to 65' before v34)
    assert texts(applied((Kind.higherAhead, 70, 65, 1, False))) == ("70 ahead", "speeding up")

  def test_never_slowing_when_going_up(self):
    # the Nov bug: a 50 -> 55 early step said 'Slowing for 55'
    for kind in (Kind.sign, Kind.lowerAhead, Kind.higherAhead):
      assert "slowing" not in texts(applied((kind, 55, 60, 1, False)))[1]
      assert "speeding" not in texts(applied((kind, 45, 50, -1, False)))[1]

  def test_set_speed_change_the_car_wont_feel(self):
    a = applied((Kind.sign, 50, 55, 0, False), set_mph=60)
    assert texts(a) == ("Limit 50", "set to 55")

  def test_nothing_changes(self):
    a = applied((Kind.sign, 55, 60, 0, False), set_mph=60)
    assert texts(a) == ("Limit 55", "") and a.alert_size == ev.AlertSize.small

  def test_chime_unless_silent(self):
    assert applied((Kind.sign, 45, 50, -1, False)).audible_alert == ev.AudibleAlertSP.promptSingleHigh
    assert applied((Kind.sign, 45, 50, -1, True)).audible_alert == ev.AudibleAlert.none

  def test_short_enough_for_the_big_font(self):
    for m in ((Kind.sign, 45, 50, -1, False), (Kind.lowerAhead, 25, 30, -1, False), (Kind.higherAhead, 70, 75, 1, False)):
      t1, t2 = texts(applied(m))
      assert len(t1) <= 12 and len(t2) <= 18, (t1, t2)

  def test_without_a_message_falls_back_to_the_numbers(self):
    s, cs = sm(55, 60, set_mph=50)
    assert texts(ev.speed_limit_applied_alert(CP, cs, s, False, 0, None)) == ("Limit 55", "set to 60")

  def test_asking(self):
    s, cs = sm(35, 40, set_mph=60)
    assert texts(ev.speed_limit_pre_active_alert(CP, cs, s, False, 0, None)) == ("Limit 35?", "tap - for 40")

  def test_asking_direction_uses_the_right_units(self):
    # dash 37, new target 60: tap + (stock compared km/h against mph and said -)
    s, cs = sm(55, 60, set_mph=37)
    assert texts(ev.speed_limit_pre_active_alert(CP, cs, s, False, 0, None)) == ("Limit 55?", "tap + for 60")

  def test_no_prompt_when_already_set(self):
    s, cs = sm(55, 60, set_mph=60)
    assert texts(ev.speed_limit_pre_active_alert(CP, cs, s, False, 0, None))[0] == ""


def test_fork_alerts_are_short():
  """Every fork / sunnypilot pop-up fits the comma four's big first line (<= 16 chars) and a one-line second."""
  for name in ('manualSteeringRequired', 'manualLongitudinalRequired', 'laneChangeRoadEdge', 'laneChangeOff', 'laneChangeOn',
               'speedLimitPending', 'laneTurnLeft', 'laneTurnRight'):
    for alert in ev.EVENTS_SP[getattr(ev.EventNameSP, name)].values():
      assert len(alert.alert_text_1) <= 16 and len(alert.alert_text_2) <= 20, (name, alert.alert_text_1, alert.alert_text_2)
