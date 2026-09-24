"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD, get_sanitize_int_param
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import LIMIT_MAX_MAP_DATA_AGE, LIMIT_ADAPT_ACC
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy, OffsetType

SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source

ALL_SOURCES = tuple(SpeedLimitSource.schema.enumerants.values())

# Fork: ease off ahead of a lower limit the map knows about. The car's camera reports a new limit only at the
# sign; map data can see it coming. Map data is used for this and nothing else: only to lower the target, only
# when the map agrees with the sign the camera has already read (so a wrong map entry can't act), and along a
# gentle ramp that ends at the sign. The camera's reading always wins once it arrives.
ANTICIPATE_DECEL = 0.5      # m/s^2, ramp gentleness (55 -> 35 mph starts about 360 m before the sign)
ANTICIPATE_AGREE_TOL = 1.0  # m/s, map current limit must match the car's reading within this (~2 mph)
ANTICIPATE_MAX_DIST = 600.  # m, ignore map limits further ahead than this


class SpeedLimitResolver:
  limit_solutions: dict[custom.LongitudinalPlanSP.SpeedLimit.Source, float]
  distance_solutions: dict[custom.LongitudinalPlanSP.SpeedLimit.Source, float]
  v_ego: float
  speed_limit: float
  speed_limit_last: float
  speed_limit_final: float
  speed_limit_final_last: float
  distance: float
  source: custom.LongitudinalPlanSP.SpeedLimit.Source
  speed_limit_offset: float

  def __init__(self):
    self.params = Params()
    self.frame = -1

    self._gps_location_service = get_gps_location_service(self.params)
    self.limit_solutions = {}  # Store for speed limit solutions from different sources
    self.distance_solutions = {}  # Store for distance to current speed limit start for different sources

    self.policy = self.params.get("SpeedLimitPolicy", return_default=True)
    self.policy = get_sanitize_int_param(
      "SpeedLimitPolicy",
      Policy.min().value,
      Policy.max().value,
      self.params
    )
    self._policy_to_sources_map = {
      Policy.car_state_only: [SpeedLimitSource.car],
      Policy.map_data_only: [SpeedLimitSource.map],
      Policy.car_state_priority: [SpeedLimitSource.car, SpeedLimitSource.map],
      Policy.map_data_priority: [SpeedLimitSource.map, SpeedLimitSource.car],
      Policy.combined: [SpeedLimitSource.car, SpeedLimitSource.map],
    }
    self.source = SpeedLimitSource.none
    for source in ALL_SOURCES:
      self._reset_limit_sources(source)

    self.is_metric = self.params.get_bool("IsMetric")
    self.offset_type = get_sanitize_int_param(
      "SpeedLimitOffsetType",
      OffsetType.min().value,
      OffsetType.max().value,
      self.params
    )
    self.offset_value = self.params.get("SpeedLimitValueOffset", return_default=True)

    self.speed_limit = 0.
    self.speed_limit_last = 0.
    self.speed_limit_final = 0.
    self.speed_limit_final_last = 0.
    self.speed_limit_offset = 0.

  def update_speed_limit_states(self) -> None:
    self.speed_limit_final = self.speed_limit + self.speed_limit_offset

    if self.speed_limit > 0.:
      self.speed_limit_last = self.speed_limit
      self.speed_limit_final_last = self.speed_limit_final

  @property
  def speed_limit_valid(self) -> bool:
    return bool(self.speed_limit > 0.)

  @property
  def speed_limit_last_valid(self) -> bool:
    return bool(self.speed_limit_last > 0.)

  def update_params(self):
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.policy = self.params.get("SpeedLimitPolicy", return_default=True)
      self.is_metric = self.params.get_bool("IsMetric")
      self.offset_type = self.params.get("SpeedLimitOffsetType", return_default=True)
      self.offset_value = self.params.get("SpeedLimitValueOffset", return_default=True)

  def _get_speed_limit_offset(self) -> float:
    if self.offset_type == OffsetType.off:
      return 0
    elif self.offset_type == OffsetType.fixed:
      return float(self.offset_value * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS))
    elif self.offset_type == OffsetType.percentage:
      return float(self.offset_value * 0.01 * self.speed_limit)
    else:
      raise NotImplementedError("Offset not supported")

  def _reset_limit_sources(self, source: custom.LongitudinalPlanSP.SpeedLimit.Source) -> None:
    self.limit_solutions[source] = 0.
    self.distance_solutions[source] = 0.

  def _get_from_car_state(self, sm: messaging.SubMaster) -> None:
    self._reset_limit_sources(SpeedLimitSource.car)
    self.limit_solutions[SpeedLimitSource.car] = sm['carStateSP'].speedLimit
    self.distance_solutions[SpeedLimitSource.car] = 0.

  def _get_from_map_data(self, sm: messaging.SubMaster) -> None:
    self._reset_limit_sources(SpeedLimitSource.map)
    self._process_map_data(sm)

  def _process_map_data(self, sm: messaging.SubMaster) -> None:
    gps_data = sm[self._gps_location_service]
    map_data = sm['liveMapDataSP']

    gps_fix_age = time.monotonic() - gps_data.unixTimestampMillis * 1e-3
    if gps_fix_age > LIMIT_MAX_MAP_DATA_AGE:
      return

    speed_limit = map_data.speedLimit if map_data.speedLimitValid else 0.
    next_speed_limit = map_data.speedLimitAhead if map_data.speedLimitAheadValid else 0.

    self._calculate_map_data_limits(sm, speed_limit, next_speed_limit)

  def _calculate_map_data_limits(self, sm: messaging.SubMaster, speed_limit: float, next_speed_limit: float) -> None:
    gps_data = sm[self._gps_location_service]
    map_data = sm['liveMapDataSP']

    distance_since_fix = self.v_ego * (time.monotonic() - gps_data.unixTimestampMillis * 1e-3)
    distance_to_speed_limit_ahead = max(0., map_data.speedLimitAheadDistance - distance_since_fix)

    self.limit_solutions[SpeedLimitSource.map] = speed_limit
    self.distance_solutions[SpeedLimitSource.map] = 0.

    # FIXME-SP: this is not working as expected
    if 0. < next_speed_limit < self.v_ego:
      adapt_time = (next_speed_limit - self.v_ego) / LIMIT_ADAPT_ACC
      adapt_distance = self.v_ego * adapt_time + 0.5 * LIMIT_ADAPT_ACC * adapt_time ** 2

      if distance_to_speed_limit_ahead <= adapt_distance:
        self.limit_solutions[SpeedLimitSource.map] = next_speed_limit
        self.distance_solutions[SpeedLimitSource.map] = distance_to_speed_limit_ahead

  def _get_source_solution_according_to_policy(self) -> custom.LongitudinalPlanSP.SpeedLimit.Source:
    sources_for_policy = self._policy_to_sources_map[Policy(self.policy)]

    if Policy(self.policy) != Policy.combined:
      # They are ordered in the order of preference, so we pick the first that's non-zero
      for source in sources_for_policy:
        if self.limit_solutions[source] > 0.:
          return source
      return SpeedLimitSource.none

    sources_with_limits = [(s, limit) for s, limit in [(s, self.limit_solutions[s]) for s in sources_for_policy] if limit > 0.]
    if sources_with_limits:
      return min(sources_with_limits, key=lambda x: x[1])[0]

    return SpeedLimitSource.none

  def _resolve_limit_sources(self, sm: messaging.SubMaster) -> tuple[float, float, custom.LongitudinalPlanSP.SpeedLimit.Source]:
    """Get limit solutions from each data source"""
    self._get_from_car_state(sm)
    self._get_from_map_data(sm)

    source = self._get_source_solution_according_to_policy()
    speed_limit = self.limit_solutions[source] if source else 0.
    distance = self.distance_solutions[source] if source else 0.

    if source == SpeedLimitSource.car:
      anticipated, dist_ahead = self._anticipate_lower_limit_ahead(sm, speed_limit)
      if 0. < anticipated < speed_limit:
        speed_limit, distance, source = anticipated, dist_ahead, SpeedLimitSource.map

    return speed_limit, distance, source

  def _anticipate_lower_limit_ahead(self, sm: messaging.SubMaster, car_limit: float) -> tuple[float, float]:
    """A gently ramped target toward a lower limit the map says is ahead, or (0, 0) when not applicable."""
    map_data = sm['liveMapDataSP']
    if car_limit <= 0. or not map_data.speedLimitValid or not map_data.speedLimitAheadValid:
      return 0., 0.
    # Age of the GPS fix, from the message's receive time. (The receiver's unixTimestampMillis is wall-clock
    # time; measured against time.monotonic(), seconds since boot, the fix looked ~50 years old and the sign
    # ~10^10 m away, so this never fired on the device.)
    gps_fix_age = time.monotonic() - sm.logMonoTime[self._gps_location_service] * 1e-9
    if not 0. <= gps_fix_age <= LIMIT_MAX_MAP_DATA_AGE:
      return 0., 0.
    # the map must agree with the sign the camera already read, and the limit ahead must be lower
    if abs(map_data.speedLimit - car_limit) > ANTICIPATE_AGREE_TOL:
      return 0., 0.
    next_limit = map_data.speedLimitAhead
    if not 0. < next_limit < car_limit:
      return 0., 0.

    dist_ahead = max(0., map_data.speedLimitAheadDistance - self.v_ego * gps_fix_age)
    if dist_ahead > ANTICIPATE_MAX_DIST:
      return 0., 0.

    # speed that reaches next_limit exactly at the sign with a constant, gentle deceleration
    ramp = (next_limit ** 2 + 2. * ANTICIPATE_DECEL * dist_ahead) ** 0.5
    # plain floats: v_ego comes from the planner's filter as a numpy float, and a numpy result here made
    # speed_limit_valid a numpy.bool, which capnp refuses when plannerd publishes (crashed plannerd, Sep 24)
    return float(min(ramp, car_limit)), float(dist_ahead)

  def update(self, v_ego: float, sm: messaging.SubMaster) -> None:
    self.v_ego = float(v_ego)
    self.update_params()

    self.speed_limit, self.distance, self.source = self._resolve_limit_sources(sm)
    self.speed_limit_offset = self._get_speed_limit_offset()

    self.update_speed_limit_states()

    self.frame += 1
