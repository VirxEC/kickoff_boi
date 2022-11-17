from collections import deque
from math import atan2, degrees, floor, inf, sqrt
from time import perf_counter

from rlbot.agents.base_agent import SimpleControllerState
from rlbot.agents.standalone.standalone_bot import StandaloneBot, run_bot
from rlbot.utils.game_state_util import Vector3
from rlbot.utils.structures.game_data_struct import GameTickPacket

from impossibum_states import *


class Kamael(StandaloneBot):
    def initialize_agent(self):
        self.controller_state = None
        self.me = physicsObject()
        self.ball = physicsObject()
        self.me.team = self.team
        self.me.index = self.index
        self.allies = []
        self.enemies = []
        self.start = 5
        self.flipStart = 0
        self.flipping = False
        self.controller = None
        self.gameInfo = GameInfo()
        self.onSurface = False
        self.boosts = []
        self.bigBoosts = []
        self.fieldInfo = []
        self.positions = []
        self.time = 0
        self.deltaTime = 0
        self.maxSpd = 2300
        self.l_cap = 350
        self.ballPred = []
        self.oldPreds = []
        self.selectedBallPred = None
        self.ballDelay = 0.00001
        self.ballPredObj = None
        self.forward = True
        self.velAngle = 0
        self.onWall = False
        self.wallLimit = 90
        self.stateTimer = 0
        self.contested = True
        self.flipTimer = 0
        self.goalPred = None #prediction of ball going into player's net
        self.scorePred = None #prediction of ball going into opponant's net
        self.currentSpd = 1
        # octane hitbox
        self.hitbox_set = False
        self.carLength = 118.007
        self.carWidth = 84.2
        self.carHeight = 36.159
        self.functional_car_height = 120
        self.defaultElevation = 17.01
        self.defaultOffset = Vector([13.88, 0.0, 20.75])
        self.groundCutOff = 120  # 93+(self.carHeight*.8)
        self.wallCutOff = 120
        self.ballGrounded = False
        self.closestEnemyToMe = None
        self.closestEnemyToMeDistance = inf
        self.closestEnemyToBall = None
        self.closestEnemyDistances = [0, 0, 0, 0, 0]
        self.enemyBallInterceptDelay = 0
        self.enemyPredTime = 0
        self.closestEnemyToBallDistance = inf
        self.enemyTargetVec = Vector([0, 0, 0])
        self.enemyTargetVel = Vector([0, 0, 0])
        self.contestedThreshold = 300
        self.superSonic = False
        self.wallShot = False
        self.openGoal = False
        self.boostConsumptionRate = 33.334
        self.allowableJumpDifference = 110
        self.wallShotsEnabled = True
        self.touch = None
        self.targetDistance = 1500
        self.fpsLimit = 1 / 120
        self.gravity = -650
        self.sorted_hits = []
        self.first_hit = None
        self.update_time = 0
        self.dribbler = False
        self.goalie = False
        self.last_tick = 0
        self.tick_timers = deque(maxlen=20)
        self.fakeDeltaTime = 1.0 / 120.0
        self.physics_tick = 1.0 / 120
        self.multiplier = 1

        self.accelerationTick = self.boostAccelerationRate * self.fakeDeltaTime
        self.aerialAccelerationTick = (self.boostAccelerationRate+66.666667) * self.fakeDeltaTime
        self.resetLimit = 2
        self.resetCount = 0
        self.resetTimer = 0
        self.timid = False
        self.goalward = False
        self.stubbornessTimer = 0
        self.stubbornessMax = 600
        self.stubbornessMin = 25
        self.stubborness = self.stubbornessMin
        self.activeState = PreemptiveStrike(self)
        self.contestedTimeLimit = 0.5
        self.demoSpawns = [
            [Vector([-2304, -4608, 0]), Vector([2304, -4608, 0])],
            [Vector([2304, 4608, 0]), Vector([-2304, 4608, 0])],
        ]
        self.rotationNumber = 1
        self.hyperJumpTimer = 0
        self.reachLength = 120
        self.groundReachLength = 120
        self.debugging = False
        self.angleLimit = 60
        self.aerialsEnabled = True
        self.aerialsLimited = False
        self.aerial_timer_limit = 0
        self.kickoff_timer = 0
        self.blueGoal = Vector([0, -5120, 0])
        self.orangeGoal = Vector([0, 5120, 0])
        self.boostThreshold = 65
        self.test_done = True
        self.available_delta_v = 0
        self._forward, self.left, self.up = (
            Vector([0, 0, 0]),
            Vector([0, 0, 0]),
            Vector([0, 0, 0]),
        )
        self.defaultRotation = None
        self.recovery_height = 60
        self.log = []
        self.game_active = True
        self.default_demo_location = Vector([20000, 0, 0])
        self.double_point_limit = 500
        self.p_tournament_mode = False
        self.cached_jump_sim = []
        self.boost_testing = False
        self.lastSpd = 0
        self.grounded_timer = 0
        self.enemyGoalLocations = []
        self.goal_locations = []
        self.boost_gobbling = False
        self.ally_hit_count = 0
        self.ally_hit_info = []
        self.aerial_min = 200
        self.match_ended = False
        self.roof_height = None
        self.ball_size = 93
        self.demo_monster = False
        self.aerial_hog = False
        self.ignore_list_names = []
        self.ignore_list_indexes = []
        self.ignore_checked = False
        self.team_size_limit = 3
        self.takeoff_speed = 0.35
        self.demo_rotations = True
        self.boost_req = (1/120) * 6.5 * self.boostAccelerationRate
        self.seekerbot = False
        self.aerial_reach = 145
        self.min_time = self.fakeDeltaTime * 2
        self.cannister_greed = 1500
        self.fake_single_sim = [90, 0.2, 120, 0.2, [0, 0, 0], 0, False]
        self.aerial_slope = 4950 / 5120
        self.aim_range = 3000
        self.coms_timer = perf_counter()
        self.send_rate = 1 / 10
        self.coms_key = "tmcp_version"
        self.allowed_keys = [self.coms_key, "team", "index", "action"]
        self.ally_actions = {}
        self.my_corners = [0, 1]
        self.linep_info = None
        if self.team == 1:
            self.my_corners = [2, 3]
        if self.dribbler:
            self.aerialsEnabled = False

        self.winning = False
        ########################
        "floating sim stuff"
        self.collision_location = Vector([0, 0, 0])
        self.simulated_velocity = Vector([0, 0, 0])
        self.sim_frames = []
        self.collision_timer = 5
        self.aim_direction = Vector([0, 0, -1])
        self.roll_type = 1
        self.squash_index = 2
        self.last_float_sim_time = 0
        self.on_ground_estimate = 0
        self.wall_landing = False
        self.aerial_accel_limit = self.boostAccelerationRate
        self.scared = False
        self.min_aerial_buffer = 100
        self.jumped = False
        self.ally_back_count = 0
        self.coast_decel = -525 * self.fakeDeltaTime
        self.active_decel = -3500 * self.fakeDeltaTime
        self.on_correct_side = False
        self.offensive = False
        self.speed_maximums = [
            [0,0.0069],
            [500,0.00398],
            [1000,0.00235],
            [1500,0.001375],
            [1750,0.0011],
            [2300,0.00088]
        ]
        self.scores = [0,0]
        self.last_controller = SimpleControllerState()

    def find_sim_frame(self, game_time):
        for frame in self.sim_frames:
            if frame[2] >= game_time:
                return frame
        return None

    def is_hot_reload_enabled(self):
        return False

    def retire(self):
        self.game_active = False

    def init_match_config(self, match_config):
        self.matchSettings = match_config
        base_boost_accel = 991 + (2 / 3)
        boost_multi = float(self.matchSettings.mutators.boost_strength[:-1])
        self.boostAccelerationRate = base_boost_accel * boost_multi

    def demoRelocation(self, car):
        return self.default_demo_location

    def setHalfFlip(self):
        _time = self.time
        if _time - self.flipTimer >= 1.9:
            controls = []
            timers = []

            control_1 = SimpleControllerState()
            control_1.throttle = -1
            control_1.jump = True

            controls.append(control_1)
            timers.append(0.125)

            controls.append(SimpleControllerState())
            timers.append(self.fakeDeltaTime * 2)

            control_3 = SimpleControllerState()
            control_3.throttle = -1
            control_3.pitch = 1
            control_3.jump = True
            controls.append(control_3)
            timers.append(self.fakeDeltaTime * 2)

            control_4 = SimpleControllerState()
            control_4.throttle = -1
            control_4.pitch = -1
            control_4.roll = -0.1
            # control_4.jump = True

            controls.append(control_4)
            timers.append(0.5)

            controls.append(SimpleControllerState(throttle=1))
            timers.append(0.6)

            self.activeState = Divine_Mandate(self, controls, timers)

            self.flipTimer = self.time
            self.stubbornessTimer = 2
            self.stubborness = self.stubbornessMax

    def determineFacing(self):
        offset = self.me.location + self.me.velocity.normalize().scale(500)
        loc = toLocal(offset, self.me)
        angle = correctAngle(degrees(atan2(loc[1], loc[0])))

        if abs(angle) > 90:
            if self.currentSpd <= self.stubborness:
                self.forward = True
            else:
                self.forward = False
        else:
            self.forward = True

        self.velAngle = angle

    def setJumping(self, targetType, target=None):
        _time = self.time
        if _time - self.flipTimer >= 1.85:
            if self.onSurface and not self.jumped:
                if targetType == 0:
                    self.createJumpChain(0.15, jumpSim=self.fake_single_sim)
                    self.flipTimer = _time

                elif targetType != -1:
                    self.activeState = LeapOfFaith(self, targetType, target=target)
                    self.flipTimer = _time

                else:
                    self.activeState = Divine_Mandate(
                        self, [SimpleControllerState(jump=True)], [0.21]
                    )
                    self.flipTimer = _time - 1

    def getCurrentSpd(self):
        return self.me.velocity.magnitude()

    def preprocess(self, game):
        self.ball_size = game.game_ball.collision_shape.sphere.diameter / 2
        self.gameInfo.update(game)
        self.oldPreds = self.ballPred
        self.ballPred = self.get_ball_prediction_struct()
        blue_score = game.teams[0].score
        orange_score = game.teams[1].score
        self.winning = (
            True
            if (self.team == 0 and blue_score > orange_score)
            or (self.team == 1 and blue_score < orange_score)
            else False
        )

        self.linep_info = None

        self.scores[0], self.scores[1] = blue_score, orange_score
        self.players = [self.index]
        car = game.game_cars[self.index]
        ally_count = len(self.allies)
        enemy_count = len(self.enemies)
        self.deltaTime = clamp(
            1, self.fpsLimit, game.game_info.seconds_elapsed - self.time
        )
        self.time = game.game_info.seconds_elapsed

        self.me.demolished = car.is_demolished

        self.gravity = game.game_info.world_gravity_z
        if not self.defaultRotation:
            self.defaultRotation = Vector3(
                car.physics.rotation.pitch,
                car.physics.rotation.yaw,
                car.physics.rotation.roll,
            )

        if not car.is_demolished:
            self.me.location = Vector(
                [car.physics.location.x, car.physics.location.y, car.physics.location.z]
            )
            self.me.velocity = Vector(
                [car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z]
            )
            self.me.rotation = Vector(
                [
                    car.physics.rotation.pitch,
                    car.physics.rotation.yaw,
                    car.physics.rotation.roll,
                ]
            )
            self.me.avelocity = Vector(
                [
                    car.physics.angular_velocity.x,
                    car.physics.angular_velocity.y,
                    car.physics.angular_velocity.z,
                ]
            )
            self.me.boostLevel = car.boost
            self.onSurface = car.has_wheel_contact
            self.superSonic = car.is_super_sonic
            self.lastSpd = self.currentSpd * 1
            self.currentSpd = clamp(2300, 1, self.getCurrentSpd())
            self.me.matrix = rotator_to_matrix(self.me)
            self._forward, self.left, self.up = self.me.matrix
            self.me.rotational_velocity = matrixDot(self.me.matrix, self.me.avelocity)
        else:
            self.me.location = self.demoRelocation(car)
            self.me.velocity = Vector([0, 0, 0])
            self.me.rotation = Vector([0, 0, 0])
            self.me.avelocity = Vector([0, 0, 0])
            self.me.boostLevel = 34
            self.onSurface = True
            self.superSonic = False
            self.currentSpd = 0.0001
            self.me.matrix = rotator_to_matrix(self.me)
            self._forward, self.left, self.up = self.me.matrix
            self.me.rotational_velocity = matrixDot(self.me.matrix, self.me.avelocity)

        if not self.hitbox_set:
            self.fieldInfo = self.get_field_info()
            self.carLength = car.hitbox.length
            self.carWidth = car.hitbox.width
            self.carHeight = car.hitbox.height

            self.functional_car_height = self.carHeight
            self.groundCutOff = (self.ball_size + self.carHeight + 17) * 0.9
            self.wallCutOff = (self.ball_size + self.carHeight + 17) * 0.9
            self.hitbox_set = True

            self.defaultOffset = Vector(
                [
                    car.hitbox_offset.x * 1,
                    car.hitbox_offset.y * 1,
                    car.hitbox_offset.z * 1,
                ]
            )
            if int(self.carLength) == 118 or int(self.carLength == 127):
                self.defaultElevation = 17 + self.defaultOffset[2]
            else:
                self.defaultElevation = 18 + self.defaultOffset[2]

            self.recovery_height = self.defaultElevation + 15

            self.enemyGoalLocations.append(
                Vector([893 * -sign(self.team), 5120 * -sign(self.team), 0])
            )
            self.enemyGoalLocations.append(Vector([0, 5120 * -sign(self.team), 0]))
            self.enemyGoalLocations.append(
                Vector([893 * sign(self.team), 5120 * -sign(self.team), 0])
            )

            self.goal_locations.append(
                Vector([893 * -sign(self.team), 5120 * sign(self.team), 0])
            )
            self.goal_locations.append(Vector([0, 5120 * sign(self.team), 0]))
            self.goal_locations.append(
                Vector([893 * sign(self.team), 5120 * sign(self.team), 0])
            )

            add_car_offset(self)
            adjusted_roof_height = self.roof_height
            self.groundReachLength = (
                floor(
                    sqrt(
                        adjusted_roof_height
                        * (self.ball_size * 2 - adjusted_roof_height)
                    )
                )
                + self.carLength * 0.5
            )
            self.reachLength = self.carLength * 0.5 + self.ball_size
            self.aerial_reach = self.carLength * 0.5 + self.ball_size

            if (
                not self.ignore_checked
                and not self.goalie
                and not self.demo_monster
                and not self.aerial_hog
            ):
                active_teammates = []
                self.ignore_list_indexes = []
                self.ignore_checked = True
                for i in range(game.num_cars):
                    if i != self.index:
                        car = game.game_cars[i]
                        if car.team == self.team:
                            for name in self.ignore_list_names:
                                if str(car.name).lower().find(name.lower()) != -1:
                                    if i not in self.ignore_list_indexes:
                                        self.ignore_list_indexes.append(i)
                                        print(
                                            f"{self.name} will ignore {car.name} in rotation logic"
                                        )

                            if i not in self.ignore_list_indexes:
                                active_teammates.append(i)
                    else:
                        active_teammates.append(self.index)

                kam_list = []
                for i in range(game.num_cars):
                    car = game.game_cars[i]
                    if car.team == self.team:
                        if str(car.name).lower().find("kamael") != -1:
                            kam_list.append(i)

                if len(kam_list) > self.team_size_limit:
                    if self.index == kam_list[self.team_size_limit]:
                        self.goalie = True
                        self.demo_rotations = False
                    elif self.index in kam_list[self.team_size_limit:]:
                        self.demo_monster = True
                    else:
                        for ally in kam_list[self.team_size_limit+1:]:
                            self.ignore_list_indexes.append(ally)

        add_car_offset(self)
        self.jumped = car.jumped

        if self.stubbornessTimer > 0:
            self.stubbornessTimer -= self.deltaTime
            if self.stubbornessTimer <= 0:
                self.stubborness = self.stubbornessMin

        ball = game.game_ball.physics
        self.ball.location = Vector([ball.location.x, ball.location.y, ball.location.z])
        self.ball.velocity = Vector([ball.velocity.x, ball.velocity.y, ball.velocity.z])
        self.ball.rotation = Vector(
            [ball.rotation.pitch, ball.rotation.yaw, ball.rotation.roll]
        )
        self.ball.avelocity = Vector(
            [ball.angular_velocity.x, ball.angular_velocity.y, ball.angular_velocity.z]
        )
        self.ball.local_location = localizeVector(self.ball.location, self.me)
        ball.lastTouch = game.game_ball.latest_touch.time_seconds
        ball.lastToucher = game.game_ball.latest_touch.player_name
        touch = ballTouch(game.game_ball.latest_touch)
        if not self.touch:
            self.touch = touch

        if self.touch != touch:
            self.touch = touch
            updateHits = True

        self.on_correct_side = self.me.location[1] * sign(
            self.team
        ) >= self.ball.location[1] * sign(self.team)
        self.offensive = self.ball.location[1] * sign(self.team) < 0

        self.allies.clear()
        self.enemies.clear()

        for i in range(game.num_cars):
            if i != self.index and i not in self.ignore_list_indexes:
                car = game.game_cars[i]
                _obj = physicsObject()
                _obj.index = i
                _obj.team = car.team
                _obj.demolished = car.is_demolished
                if not car.is_demolished:
                    _obj.location = Vector(
                        [
                            car.physics.location.x,
                            car.physics.location.y,
                            car.physics.location.z,
                        ]
                    )
                    _obj.velocity = Vector(
                        [
                            car.physics.velocity.x,
                            car.physics.velocity.y,
                            car.physics.velocity.z,
                        ]
                    )
                    _obj.rotation = Vector(
                        [
                            car.physics.rotation.pitch,
                            car.physics.rotation.yaw,
                            car.physics.rotation.roll,
                        ]
                    )
                    _obj.avelocity = Vector(
                        [
                            car.physics.angular_velocity.x,
                            car.physics.angular_velocity.y,
                            car.physics.angular_velocity.z,
                        ]
                    )
                    _obj.boostLevel = car.boost
                    _obj.local_location = localizeVector(_obj, self.me)
                    _obj.onSurface = car.has_wheel_contact
                else:
                    _obj.location = self.demoRelocation(_obj)
                    _obj.velocity = Vector([0, 0, 0])
                    _obj.rotation = Vector([0, 0, 0])
                    _obj.avelocity = Vector([0, 0, 0])
                    _obj.boostLevel = 33
                    _obj.onSurface = True
                    _obj.man = 3

                if car.team == self.team:
                    self.allies.append(_obj)
                else:
                    self.enemies.append(_obj)
        self.boosts = []
        self.bigBoosts = []

        for index in range(self.fieldInfo.num_boosts):
            packetBoost = game.game_boosts[index]
            fieldInfoBoost = self.fieldInfo.boost_pads[index]
            boostStatus = False
            if packetBoost.timer <= 0:
                if packetBoost.is_active:
                    boostStatus = True
            boost_obj = Boost_obj(
                [
                    fieldInfoBoost.location.x,
                    fieldInfoBoost.location.y,
                    fieldInfoBoost.location.z,
                ],
                fieldInfoBoost.is_full_boost,
                boostStatus,
                index,
            )
            self.boosts.append(boost_obj)
            if boost_obj.bigBoost:
                self.bigBoosts.append(boost_obj)

        self.onWall = False
        self.wallShot = False
        self.aerialsEnabled = True #len(self.allies) > 0
        self.aerialsLimited = False
        self.aerial_timer_limit = clamp(6, 2, len(self.allies) * 3)
        # if len(self.allies) < 1 or self.dribbler:
        if self.dribbler or self.goalie:
            self.aerialsEnabled = False
            self.aerialsLimited = True

        if (
            self.onSurface
            and (abs(self.me.location[0]) > 3900 or abs(self.me.location[1]) > 4900)
            and self.me.location[2] > 50
            and abs(self.me.location[1]) < 5120
        ):
            self.onWall = self.up[2] < 0.98

        self.determineFacing()
        self.boost_gobbling = False
        self.findClosestToEnemies()
        self.resetCount += 1

        if (
            not self.onSurface
            and (self.time - self.last_float_sim_time) > self.fakeDeltaTime * 5 or self.last_controller.boost
        ):
            run_float_simulation(self)

        if self.resetCount >= self.resetLimit:
            findEnemyHits(self)
            self.resetCount = 0
        else:
            self.enemyBallInterceptDelay = self.enemyPredTime - self.time

        if self.gameInfo.is_kickoff_pause:
            self.kickoff_timer = self.time

    def findClosestToEnemies(self):
        if len(self.enemies) > 0:
            (
                self.closestEnemyToBall,
                self.closestEnemyToBallDistance,
            ) = findEnemyClosestToLocation(self, self.ball.location, demo_bias=True)
            (
                self.closestEnemyToMe,
                self.closestEnemyToMeDistance,
            ) = findEnemyClosestToLocation(self, self.me.location, demo_bias=True)
            self.contested = False

            if self.closestEnemyToBallDistance <= self.contestedThreshold:
                self.contested = True

            if self.closestEnemyToBall is not None:
                etv = self.ball.location

                closestEnemyToBallTargetDistance = findDistance(
                    etv, self.closestEnemyToBall.location
                )
                self.closestEnemyDistances.append(closestEnemyToBallTargetDistance)
                del self.closestEnemyDistances[0]

        else:
            self.closestEnemyToBall = self.me
            self.closestEnemyToMe = self.me
            self.closestEnemyToBallDistance = 0
            self.closestEnemyToMeDistance = 0
            self.contested = False

    def createJumpChain(
        self, timeAlloted, jumpSim, set_state=True
    ):
        intercept = None
        target = None

        controls = []
        timers = []
        pitch = 0
        firstJumpDuration = 0.2

        targetTime = timeAlloted
        timeRemaining = targetTime * 1

        if jumpSim[-1] == False:
            pitch = 0
            if jumpSim[2] < 95:
                pitch = clamp(0.75, 0.3, 1 - (clamp(0.75, 0.001, timeRemaining)))
            controls.append(SimpleControllerState(jump=True, pitch=pitch))
            timers.append(
                clamp(
                    5,
                    self.fakeDeltaTime * 7,
                    timeRemaining - self.fakeDeltaTime * 6,
                )
            )

            controls.append(SimpleControllerState(jump=False))
            timers.append(self.fakeDeltaTime * 2)

            controls.append(0)
            timers.append(0.5)

            self.flipTimer = self.time + clamp(1.45, 0.5, timeAlloted)
        else:
            controls.append(SimpleControllerState(jump=True))
            firstJumpDuration += self.fakeDeltaTime*2
            timers.append(firstJumpDuration)
            timeRemaining -= firstJumpDuration

            controls.append(SimpleControllerState(jump=False, throttle=0))
            timers.append(self.fakeDeltaTime * 2)
            timeRemaining -= self.fakeDeltaTime * 2

            controls.append(SimpleControllerState(jump=True, throttle=0))
            timers.append(self.fakeDeltaTime * 3)
            timeRemaining -= self.fakeDeltaTime * 3

        if set_state:
            self.activeState = Divine_Mandate(self, controls, timers, target=target, intercept_time=intercept)
        else:
            return Divine_Mandate(self, controls, timers, target=target, intercept_time=intercept)

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        self.log.clear()
        self.preprocess(packet)

        soloStateManager_testing(self)

        if self.activeState is None:
            self.activeState = PreemptiveStrike(self)

        action = self.activeState.update()
        self.controller_state = action
        self.last_controller = action

        return action
    
if __name__ == "__main__":
    run_bot(Kamael)
