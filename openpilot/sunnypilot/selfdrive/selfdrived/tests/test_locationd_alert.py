from unittest import mock
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.selfdrived.events import Events, ET, EventName

def test_standing_message_while_condition_holds():
  ev = Events()
  seen = []
  for frame in range(400):  # 4 s with the event present every frame
    ev.clear()
    ev.add(EventName.locationdTemporaryError)
    alerts = ev.create_alerts([ET.PERMANENT], [None, None, mock.MagicMock(), False, 0, None])
    seen.append([(a.alert_text_1, a.alert_text_2) for a in alerts])
  assert seen[50] == []                                   # not yet: 2 s delay against the sub-second start-up blip
  assert seen[250] == [("Motion sensors calibrating", "please wait to use sunnypilot")]
  # the moment the condition clears, nothing is created any more (the on-screen alert then expires within 0.2 s)
  ev.clear()
  assert ev.create_alerts([ET.PERMANENT], [None, None, mock.MagicMock(), False, 0, None]) == []

def test_button_press_and_handback_use_the_same_words():
  ev = Events(); ev.add(EventName.locationdTemporaryError)
  args = [None, None, mock.MagicMock(), False, 100, None]
  ne = ev.create_alerts([ET.NO_ENTRY], args)[0]
  assert "Motion sensors calibrating" in (ne.alert_text_1, ne.alert_text_2)
  sd = ev.create_alerts([ET.SOFT_DISABLE], args)[0]
  assert sd.alert_text_2 == "Motion sensors calibrating"
