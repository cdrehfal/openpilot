"""Fork: the speed-limit messages say what is happening, with the numbers."""
from unittest import mock
from openpilot.cereal import custom
from openpilot.common.constants import CV
from opendbc.car.structs import car
import openpilot.sunnypilot.selfdrive.selfdrived.events as ev

Source = custom.LongitudinalPlanSP.SpeedLimit.Source
MPH = CV.MPH_TO_MS


def sm(limit, final, source=Source.car, map_here=0, ahead=0, set_mph=None):
  lp = custom.LongitudinalPlanSP.new_message()
  r = lp.speedLimit.resolver
  r.speedLimitLast = limit * MPH
  r.speedLimitFinalLast = final * MPH
  r.speedLimitOffset = 5 * MPH
  r.source = source
  lmd = custom.LiveMapDataSP.new_message(speedLimitValid=map_here > 0, speedLimit=map_here * MPH,
                                         speedLimitAheadValid=ahead > 0, speedLimitAhead=ahead * MPH)
  msgs = {'longitudinalPlanSP': lp.as_reader(), 'liveMapDataSP': lmd.as_reader(), 'controlsState': mock.MagicMock()}
  s = mock.MagicMock()
  s.__getitem__.side_effect = lambda k: msgs[k]
  cs = car.CarState.new_message(vCruiseCluster=(set_mph or 0) * CV.MPH_TO_KPH).as_reader()
  return s, cs


def texts(alert):
  return alert.alert_text_1, alert.alert_text_2


CP = car.CarParams.new_message(openpilotLongitudinalControl=False, pcmCruise=True).as_reader()


class TestSpeedLimitMessages:
  def test_applied_from_sign(self):
    s, cs = sm(55, 60, map_here=55)
    assert texts(ev.speed_limit_applied_alert(CP, cs, s, False, 0, None)) == ("Speed limit 55", "set to 60")
    s, cs = sm(70, 75)
    assert texts(ev.speed_limit_applied_alert(CP, cs, s, False, 0, None)) == ("Speed limit 70", "set to 75")

  def test_applied_easing(self):
    s, cs = sm(64, 69, source=Source.map, map_here=70, ahead=55)
    assert texts(ev.speed_limit_applied_alert(CP, cs, s, False, 0, None)) == ("Slowing for 55", "limit ahead")

  def test_asking(self):
    s, cs = sm(35, 40, set_mph=60)
    assert texts(ev.speed_limit_pre_active_alert(CP, cs, s, False, 0, None)) == ("Speed limit 35?", "tap - to set 40")

  def test_asking_direction_uses_the_right_units(self):
    # dash 37, new target 60: tap + (stock compared km/h against mph and said -)
    s, cs = sm(55, 60, set_mph=37)
    assert texts(ev.speed_limit_pre_active_alert(CP, cs, s, False, 0, None)) == ("Speed limit 55?", "tap + to set 60")

  def test_no_prompt_when_already_set(self):
    s, cs = sm(55, 60, set_mph=60)
    assert texts(ev.speed_limit_pre_active_alert(CP, cs, s, False, 0, None))[0] == ""
