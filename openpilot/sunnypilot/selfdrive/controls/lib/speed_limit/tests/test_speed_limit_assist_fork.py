"""Fork: when Speed Limit Assist applies a new limit by itself, and when it asks for a +/- tap.

The camera reads nearly every sign but sometimes the wrong one; the map is usually right but has gaps. So: apply
when the map backs up the sign, ask when the map disagrees, and with no map data apply only between highway limits
and for increases from an arterial limit."""
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist


def asks(new_mph, prev_mph, map_agrees=False, map_conflict=False, metric=False):
  sla = SpeedLimitAssist.__new__(SpeedLimitAssist)  # only the decision is under test
  conv = CV.KPH_TO_MS if metric else CV.MPH_TO_MS
  sla.is_metric = metric
  sla._speed_limit = new_mph * conv
  sla._sign_prev = prev_mph * conv
  sla._map_agrees, sla._map_conflict = map_agrees, map_conflict
  return sla.apply_confirm_speed_threshold


class TestApplyOrAsk:
  def test_highway_changes_apply_without_map(self):
    for new, prev in ((65, 70), (70, 65), (55, 70), (70, 55), (75, 55), (55, 65), (80, 75)):
      assert not asks(new, prev), (prev, new)

  def test_map_backed_changes_apply_anywhere(self):
    # entering towns, school-free 35 zones, leaving towns
    for new, prev in ((25, 55), (35, 55), (35, 25), (55, 25), (20, 30), (45, 70)):
      assert not asks(new, prev, map_agrees=True), (prev, new)

  def test_map_disagreeing_asks(self):
    # a work-zone 55 on a 70 freeway the map calls 70; an ATV/side-road 35 on a 55 road
    assert asks(55, 70, map_conflict=True)
    assert asks(35, 55, map_conflict=True)
    assert asks(65, 55, map_conflict=True)

  def test_no_map_slower_roads_ask(self):
    for new, prev in ((35, 55), (45, 55), (25, 35), (40, 45), (30, 25)):
      assert asks(new, prev), (prev, new)

  def test_no_map_increases(self):
    assert not asks(55, 45)       # leaving onto a highway from an arterial
    assert not asks(50, 45)
    assert asks(45, 35)           # from a lower-speed road: ask
    assert asks(55, 25)

  def test_first_reading_without_map_asks(self):
    assert asks(70, 0)
    assert not asks(70, 0, map_agrees=True)

  def test_metric(self):
    assert not asks(100, 110, metric=True)
    assert asks(50, 90, metric=True)
    assert not asks(50, 90, map_agrees=True, metric=True)


class TestAnnouncements:
  def _sla(self):
    sla = SpeedLimitAssist.__new__(SpeedLimitAssist)
    sla.is_metric = False
    sla._last_announced = None
    sla.v_cruise_cluster_conv = 60
    sla.speed_limit_final_last_conv = 60
    return sla

  def test_resume_doesnt_repeat_but_changes_do(self):
    from unittest import mock
    import openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist as A
    events = mock.MagicMock()
    sla = self._sla()
    with mock.patch.object(A.time, 'monotonic', side_effect=[0., 30., 40., 400.]):
      sla.update_active_event(events)                     # announced
      sla.update_active_event(events, reactivation=True)  # resume 30 s later, same limit: quiet
      sla.update_active_event(events)                     # a change to the same set speed (easing starts): announced
      sla.update_active_event(events, reactivation=True)  # resume much later: announced
    assert events.add.call_count == 3


class TestRaiseStepAfterOverride:
  """The early step toward a higher limit ahead never pulls a set speed the driver chose back down."""

  def _sla(self):
    from openpilot.cereal import custom
    from opendbc.car.structs import car
    import openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist as A
    CP = car.CarParams.new_message(brand="hyundai", pcmCruise=True, openpilotLongitudinalControl=False).as_reader()
    CP_SP = custom.CarParamsSP.new_message(pcmCruiseSpeed=False).as_reader()
    from unittest import mock
    sla = A.SpeedLimitAssist(CP, CP_SP)
    sla.params = mock.MagicMock()
    sla.params.get.side_effect = lambda k, **kw: A.Mode.assist if k == "SpeedLimitMode" else 0
    sla.params.get_bool.return_value = False
    sla.is_metric = False
    sla.enabled = True
    return sla, A

  def _step(self, sla, limit_mph, set_mph, n=1, **kw):
    from unittest import mock
    ev = mock.MagicMock()
    for _ in range(n):
      sla.update(True, False, 20., 0., set_mph * CV.MPH_TO_MS, limit_mph * CV.MPH_TO_MS, (limit_mph + 5) * CV.MPH_TO_MS, True, 0., ev,
                 map_agrees=True, **kw)
    return ['disabled', 'inactive', 'preActive', 'pending', 'adapting', 'active'][int(sla.state)]

  def test_driver_override_stands_through_the_raise_then_sign_applies(self):
    sla, A = self._sla()
    self._step(sla, 45, 50, n=20)                      # engaged on a 45, set 50
    assert self._step(sla, 45, 50) == 'active'
    assert self._step(sla, 45, 60) == 'inactive'       # driver went to 60
    self._step(sla, 45, 60, n=3)
    st = self._step(sla, 50, 60, raise_step=True)      # map: 55 ahead -> early step to 50 (+5 = 55): below the driver's 60
    assert st == 'inactive'
    assert self._step(sla, 50, 60, n=40, raise_step=True) == 'inactive'
    assert self._step(sla, 55, 60) == 'active'          # the 55 sign: applied (map agrees), set 60 = 60

  def test_raise_applies_when_assist_is_in_control(self):
    sla, A = self._sla()
    self._step(sla, 45, 50, n=20)
    assert self._step(sla, 45, 50) == 'active'
    assert self._step(sla, 50, 50, raise_step=True) == 'active'
    assert round(sla.output_v_target * CV.MS_TO_MPH) == 55
