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
