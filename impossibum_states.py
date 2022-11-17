import math

from rlbot.agents.base_agent import SimpleControllerState

from impossibum_utilities import *

"""
Right corner	loc: (-2048, -2560), yaw: 0.25 pi	loc: (2048, 2560), yaw: -0.75 pi
Left corner	loc: (2048, -2560), yaw: 0.75 pi	loc: (-2048, 2560), yaw: -0.25 pi
Back right	loc: (-256.0, -3840), yaw: 0.5 pi	loc: (256.0, 3840), yaw: -0.5 pi
Back left	loc: (256.0, -3840), yaw: 0.5 pi	loc: (-256.0, 3840), yaw: -0.5 pi
Far back center	loc: (0.0, -4608), yaw: 0.5 pi	loc: (0.0, 4608), yaw: -0.5 pi
"""


def getKickoffPosition(vec):
    # 0 == wide diagonal, 1 == short disgonal, 2 == middle
    if abs(vec[0]) >= 350:
        return 0
    elif abs(vec[0]) > 5:
        return 1
    else:
        return 2


class baseState:
    def __init__(self, agent):
        self.agent = agent
        self.active = True

    def __repr__(self):
        return f"{type(self).__name__}"


class LeapOfFaith(baseState):
    def __init__(self, agent, targetCode, target=None):
        super().__init__(agent)
        self.targetCode = targetCode  # 0 flip at ball , 1 flip forward, 2 double jump, 3 flip backwards, 4 flip left, 5 flip right, 6 flip at target ,7 left forward diagnal flip, 8 right forward diagnal flip , -1 hold single jump
        self.flip_obj = FlipStatus(agent.time)
        self.target = target
        self.cancelTimerCap = 0.3
        self.cancelStartTime = None
        self.jumped = False
        self.followThrough = 0
        self.start_time = self.agent.time
        self.last_controller = SimpleControllerState()

    def update(self):
        if self.agent.onSurface:
            if self.agent.time - self.start_time > 0.2:
                self.active = False
        controller_state = SimpleControllerState()
        jump = flipHandler(self.agent, self.flip_obj)

        if jump:
            if self.targetCode == 1:
                controller_state.pitch = -1
                controller_state.roll = 0
                controller_state.throttle = 1

            elif self.targetCode == 0:
                ball_local = self.agent.ball.local_location
                ball_angle = math.atan2(ball_local.data[1], ball_local.data[0])
                controller_state.jump = True
                yaw = math.sin(ball_angle)
                pitch = -math.cos(ball_angle)
                yp = Vector([abs(yaw), abs(pitch)])
                yp = yp.normalize()
                if yaw > 0:
                    yaw = yp[0]
                else:
                    yaw = -yp[0]
                if pitch > 0:
                    pitch = yp[1]
                else:
                    pitch = -yp[1]

                controller_state.yaw = yaw
                controller_state.pitch = pitch

                if pitch > 0:
                    controller_state.throttle = -1
                else:
                    controller_state.throttle = 1

            elif self.targetCode == 2:
                controller_state.pitch = 0
                controller_state.steer = 0
                controller_state.yaw = 0

            elif self.targetCode == 3:
                controller_state.pitch = 1
                controller_state.steer = 0
                controller_state.throttle = -1

            elif self.targetCode == -1:
                controller_state.pitch = 0
                controller_state.steer = 0
                controller_state.throttle = 0

            elif self.targetCode == 4:
                controller_state.pitch = 0
                controller_state.yaw = -1
                controller_state.steer = -1
                controller_state.throttle = 1

            elif self.targetCode == 5:
                controller_state.pitch = 0
                controller_state.yaw = 1
                controller_state.steer = 1
                controller_state.throttle = 1

            elif self.targetCode == 6:
                target_local = toLocal(self.target, self.agent.me).normalize()
                target_angle = math.atan2(target_local.data[1], target_local.data[0])
                controller_state.jump = True
                controller_state.yaw = math.sin(target_angle)
                pitch = -math.cos(target_angle)
                controller_state.pitch = pitch
                if pitch > 0:
                    controller_state.throttle = -1
                else:
                    controller_state.throttle = 1

            elif self.targetCode == 7:
                controller_state.pitch = -1
                controller_state.yaw = -1
                controller_state.steer = -1
                controller_state.throttle = 1

            elif self.targetCode == 8:
                controller_state.pitch = -1
                controller_state.yaw = 1
                controller_state.steer = 1
                controller_state.throttle = 1

            elif self.targetCode == 9:
                # diagnal flip cancel
                controller_state.pitch = -0.2
                controller_state.yaw = -0.8
                # controller_state.steer = -1
                controller_state.throttle = 1

            elif self.targetCode == 10:
                # diagnal flip cancel
                controller_state.pitch = -0.2
                controller_state.yaw = 0.8
                # controller_state.steer = -1
                controller_state.throttle = 1

        controller_state.jump = jump
        controller_state.boost = False
        if self.targetCode == 7 or self.targetCode == 8:
            controller_state.boost = True
        if self.flip_obj.flipDone:
            if self.targetCode != 9 or self.targetCode != 10:
                controller_state = self.last_controller
                if (
                    self.followThrough < 0.33
                    and self.targetCode != 2
                    and self.targetCode != -1
                ):
                    self.followThrough += self.agent.deltaTime
                else:
                    self.active = False
            else:
                if not self.cancelStartTime:
                    self.cancelStartTime = self.agent.time
                    return controller_state
                if self.targetCode == 9:
                    controller_state.pitch = 1
                    controller_state.roll = 1
                    controller_state.throttle = 1
                else:
                    controller_state.pitch = 1
                    controller_state.roll = -1
                    controller_state.throttle = 1
                if self.agent.time - self.cancelStartTime >= self.cancelTimerCap:
                    self.active = False

        self.last_controller = controller_state
        return controller_state


class Divine_Mandate:
    # class for performing consecutive inputs over set periods of time. Example: Flipping forward
    def __init__(self, agent, controls_list: list, durations_list: list, target=None, intercept_time=None):
        self.controls = controls_list
        self.durations = durations_list
        self._controls = controls_list
        self._durations = durations_list
        self.complete = False
        self.index = 0
        self.current_duration = 0
        self.agent = agent
        self.touch = self.agent.ball.lastTouch
        self.target = target
        self.intercept_time = intercept_time
        self.remote = None
        self.remote_timer = 0
        # there should be a duration in the durations for every controller given in the list. This inserts 0 for any lacking
        if len(durations_list) < len(controls_list):
            self.durations += [0 * len(controls_list) - len(durations_list)]
        self.active = True

    def create_custom_controls(self, actionCode):
        # perform specialized actions if creating controlers at creation time wasn't feasible
        controller_state = SimpleControllerState()
        if actionCode == 0:
            target = self.agent.ball.location

            if self.agent.ball.lastTouch == self.touch and self.target is not None:
                target = self.target

            target_local = localizeVector(target, self.agent.me)
            ball_angle = math.atan2(target_local.data[1], target_local.data[0])
            controller_state.jump = True

            yaw = math.sin(ball_angle)
            pitch = -math.cos(ball_angle)
            yp = Vector([abs(yaw), abs(pitch)])
            yp = yp.normalize()
            if yaw > 0:
                yaw = yp[0]
            else:
                yaw = -yp[0]
            if pitch > 0:
                pitch = yp[1]
            else:
                pitch = -yp[1]

            controller_state.pitch = pitch
            controller_state.yaw = yaw
            if pitch > 0:
                controller_state.throttle = -1
            else:
                controller_state.throttle = 1

            self.controls[self.index] = controller_state

        if actionCode == 1:
            remote_loc = None
            target = self.agent.ball.location

            if self.agent.ball.lastTouch != self.touch or self.target is None:
                self.intercept_time = None
            else:
                if self.intercept_time and self.agent.time < self.intercept_time:
                    if self.agent.time - self.remote_timer > self.agent.fakeDeltaTime * 5:
                        remote_loc = self.agent.find_sim_frame(self.intercept_time)
                        if remote_loc is not None:
                            remote_loc = remote_loc[0]
                            self.remote = remote_loc
                        self.remote_timer = self.agent.time
                    else:
                        remote_loc = self.remote

                    target = self.target

            if target[1] * sign(self.agent.team) < 0 and butterZone(target, x=1200, y=4000):
                target += Vector([0, 0, 40])

            if remote_loc != None:
                # just a hack so I don't have to update the point function to work remotely
                target += (self.agent.me.location - remote_loc)

            (
                controller_state.steer,
                controller_state.yaw,
                controller_state.pitch,
                controller_state.roll,
                _,
            ) = point_at_position(self.agent, target)

            controller_state.jump = False
            controller_state.throttle = 1

        return controller_state

    def update(
        self,
    ):  # call this once per frame with delta time to receive updated controls
        self.current_duration += self.agent.deltaTime
        if self.index >= len(self.controls):
            self.active = False
            return SimpleControllerState()
        if self.current_duration > self.durations[self.index]:
            self.index += 1
            self.current_duration = self.current_duration - self.agent.deltaTime
            # self.current_duration = 0
            if self.index >= len(self.controls):
                self.active = False
                return SimpleControllerState()

        if type(self.controls[self.index]) == SimpleControllerState:
            return self.controls[self.index]

        else:
            return self.create_custom_controls(self.controls[self.index])


class PreemptiveStrike(baseState):
    def __init__(self, agent):
        super().__init__(agent)
        self.started = False
        self.firstFlip = None
        self.secondFlip = None
        self.startTime = agent.time
        self.kickoff_type = getKickoffPosition(agent.me.location)
        # 0 == wide diagonal, 1 == short disgonal, 2 == middle
        agent.stubbornessTimer = 5
        self.onRight = True
        self.short_offset = 75
        self.setup()
        self.enemyGoal = Vector([0, 5200 * -sign(self.agent.team), 0])
        self.phase = 1
        self.KO_option = None
        self.maintaining_speed = False
        self.first_recovery = None
        self.greedy = False
        self.greedy_req = 450
        self.final_flip_dist = 500

    def create_speed_flip_cancel(self, left=False):
        controls = []
        timers = []

        second_controller = SimpleControllerState()
        if not left:
            second_controller.steer = 1
        else:
            second_controller.steer = -1

        second_controller.throttle = 1
        second_controller.boost = True
        second_controller.jump = True
        second_controller.pitch = 1
        second_controller.handbrake = True
        controls.append(second_controller)
        timers.append(0.10)

        third_controller = SimpleControllerState()
        third_controller.jump = False
        third_controller.boost = True
        third_controller.throttle = 1
        third_controller.pitch = 1
        controls.append(third_controller)
        timers.append(self.agent.fakeDeltaTime * 2)

        fourth_controller = SimpleControllerState()

        yaw = -1
        if left:
            yaw = 1

        fourth_controller.yaw = yaw
        # fourth_controller.roll = yaw
        fourth_controller.pitch = -1
        fourth_controller.jump = True
        fourth_controller.boost = True
        fourth_controller.throttle = 1
        controls.append(fourth_controller)
        timers.append(0.05)

        fifth_controller = SimpleControllerState()
        fifth_controller.yaw = -yaw
        # fifth_controller.roll = -yaw
        fifth_controller.pitch = 1
        fifth_controller.throttle = 1
        fifth_controller.boost = True
        fifth_controller.handbrake = False
        fifth_controller.jump = True
        controls.append(fifth_controller)
        timers.append(0.75)

        action = Divine_Mandate(self.agent, controls, timers)
        return action

    def create_diagonal_speed_flip(self, left=False):
        controls = []
        timers = []
        # jump start
        first_controller = SimpleControllerState()
        if self.kickoff_type == 0:
            if self.onRight:
                first_controller.yaw = -1
            else:
                first_controller.yaw = 1

        first_controller.jump = True
        first_controller.boost = True
        first_controller.throttle = 1
        first_controller.pitch = -1
        first_controller.jump = True
        controls.append(first_controller)
        timers.append(0.1)

        # jump delay
        second_controller = SimpleControllerState()
        second_controller.jump = False
        second_controller.boost = True
        second_controller.throttle = 1
        if left:
            yaw = -0.75
        else:
            yaw = 0.75

        pitch = -0.25

        second_controller.yaw = yaw
        second_controller.pitch = pitch
        second_controller.jump = False

        controls.append(second_controller)
        timers.append(self.agent.fakeDeltaTime * 4)

        # jump flip
        third_controller = SimpleControllerState()
        third_controller.jump = True
        third_controller.boost = True
        third_controller.throttle = 1

        if left:
            yaw = -0.75
        else:
            yaw = 0.75

        pitch = -0.25

        third_controller.yaw = yaw
        third_controller.pitch = pitch
        controls.append(third_controller)
        timers.append(0.5)

        action = Divine_Mandate(self.agent, controls, timers)
        return action

    def setup(self):
        # setup randomness like offsets to either side of the ball. Make sure it's slightly offset from middle so we can flip center
        # setup flips
        ball_local = self.agent.ball.local_location
        if self.kickoff_type == 0:
            if ball_local[1] > 0:
                self.firstFlip = self.create_speed_flip_cancel(left=False)
                self.onRight = False
            else:
                self.firstFlip = self.create_speed_flip_cancel(left=True)
                self.onRight = True

        elif self.kickoff_type == 1:
            if ball_local[1] < 0:
                self.firstFlip = self.create_diagonal_speed_flip(left=False)
                self.onRight = True
                self.short_offset = -75 * sign(self.agent.team)
            else:
                self.firstFlip = self.create_diagonal_speed_flip(left=True)
                self.onRight = False
                self.short_offset = 75 * sign(self.agent.team)

        else:
            # middle kickoff defaulting to right
            self.firstFlip = self.create_diagonal_speed_flip(left=True)
            # self.onRight shouldn't matter

    def wide_handler(self):
        # stage 1 - drive to boost pad
        if self.phase == 1:
            if distance2D(self.agent.me.location, self.agent.ball.location) > 3110:
                x_offset = 250 if self.agent.me.location[0] > self.agent.ball.location[0] else -250
                return driveController(
                    self.agent,
                    self.agent.ball.location + Vector([x_offset,0,0]),
                    0,
                    expedite=True,
                    flippant=False,
                    maintainSpeed=self.maintaining_speed,
                )
            else:
                self.phase = 3

        # stage 3 - first flip
        if self.phase == 3:
            if self.firstFlip.active:
                return self.firstFlip.update()

            if not self.agent.onSurface:
                _controller = SimpleControllerState()
                (
                    _controller.steer,
                    _controller.yaw,
                    _controller.pitch,
                    _controller.roll,
                    _err,
                ) = point_at_position(
                    self.agent,
                    self.agent.me.location
                    + self.agent.me.velocity.normalize().flatten().scale(1000),
                )
                _controller.boost = True
                _controller.handbrake = True
                return _controller

            else:
                self.phase = 4

        # stage 4 - aim towards just offcenter of ball
        if self.phase == 4:
            if distance2D(self.agent.me.location, self.agent.ball.location) > self.final_flip_dist:
                if (self.agent.me.location - self.agent.ball.location)[0] > 0:
                    dummy_location = [35, 0, 0]
                else:
                    dummy_location = [-35, 0, 0]

                drive_target = self.agent.ball.location + Vector(dummy_location)
                return driveController(
                    self.agent,
                    drive_target,
                    0,
                    expedite=not self.agent.superSonic,
                    flippant=False,
                    maintainSpeed=self.maintaining_speed,
                )
            else:
                self.phase = 5

        # stage 5 - flip through center and end kickoff
        # 4 flip left, 5 flip right
        if self.phase == 5:
            if self.secondFlip is None and not self.greedy:
                self.greedy = distance2D(self.agent.me.location, self.agent.ball.location) + self.greedy_req <= distance2D(self.agent.closestEnemyToBall.location, self.agent.ball.location)
                self.secondFlip = LeapOfFaith(self.agent, 0, target=None)
            if not self.greedy:
                controls = self.secondFlip.update()
                if not self.secondFlip.active:
                    self.retire()
            else:
                controls = driveController(
                    self.agent,
                    self.agent.ball.location,
                    0,
                    expedite=not self.agent.superSonic,
                    flippant=False,
                )
                controls.throttle = 0
            if self.agent.ball.location[0] != 0 or self.agent.ball.location[1] != 0:
                self.retire()
            return controls

    def short_handler(self):
        # stage 1 - drive to boost pad
        if self.phase == 1:
            drive_target = Vector(
                [self.short_offset, sign(self.agent.team) * 2800.0, 0]
            )
            if distance2D(self.agent.me.location, drive_target) > 555:
                return driveController(
                    self.agent,
                    drive_target,
                    0,
                    expedite=True,
                    flippant=False,
                    maintainSpeed=self.maintaining_speed,
                )
            else:
                self.phase = 3

        # stage 2 - angle towards outside of ball
        if self.phase == 2:
            controls = SimpleControllerState()
            if not self.agent.onSurface:
                (
                    controls.steer,
                    controls.yaw,
                    controls.pitch,
                    controls.roll,
                    alignment_error,
                ) = point_at_position(
                    self.agent,
                    self.agent.me.location
                    + self.agent.me.velocity.normalize().flatten().scale(1000),
                )
                return controls
            else:
                self.phase = 3
        # stage 3 - first flip
        if self.phase == 3:
            if self.firstFlip.active:
                return self.firstFlip.update()
            if not self.agent.onSurface:
                _controller = SimpleControllerState()
                (
                    _controller.steer,
                    _controller.yaw,
                    _controller.pitch,
                    _controller.roll,
                    _err,
                ) = point_at_position(
                    self.agent,
                    self.agent.me.location
                    + self.agent.me.velocity.normalize().flatten().scale(1000),
                )
                _controller.boost = True
                _controller.handbrake = True
                return _controller
            else:
                self.phase = 4

        # stage 4 - aim towards just offcenter of ball
        if self.phase == 4:
            if (
                distance2D(self.agent.me.location, self.agent.ball.location) > self.final_flip_dist
                or not self.agent.onSurface
            ):
                drive_target = self.agent.ball.location
                if (self.agent.me.location - self.agent.ball.location)[0] > 0:
                    dummy_location = [45, 0, 0]
                else:
                    dummy_location = [-45, 0, 0]

                return driveController(
                    self.agent,
                    drive_target+Vector(dummy_location),
                    0,
                    expedite=not self.agent.superSonic,
                    flippant=False,
                    maintainSpeed=self.maintaining_speed,
                )
            else:
                self.phase = 5

        # stage 5 - flip through center and end kickoff
        # 4 flip left, 5 flip right
        if self.phase == 5:
            if self.secondFlip is None and not self.greedy:
                self.greedy = distance2D(self.agent.me.location, self.agent.ball.location) + self.greedy_req <= distance2D(
                    self.agent.closestEnemyToBall.location, self.agent.ball.location)
                self.secondFlip = LeapOfFaith(self.agent, 0, target=None)

            if not self.greedy:
                controls = self.secondFlip.update()
                if not self.secondFlip.active:
                    self.retire()
            else:
                controls = driveController(
                    self.agent,
                    self.agent.ball.location,
                    0,
                    expedite=not self.agent.superSonic,
                    flippant=False,
                )
                controls.throttle = 0
            if self.agent.ball.location[0] != 0 or self.agent.ball.location[1] != 0:
                self.retire()
            return controls

    def middle_handler(self):
        # stage 1 - drive to boost pad
        if self.phase == 1:
            drive_target = Vector([0, sign(self.agent.team) * 4015, 0])
            if distance2D(self.agent.me.location, drive_target) > 75:
                return driveController(
                    self.agent,
                    drive_target,
                    0,
                    expedite=True,
                    flippant=False,
                    maintainSpeed=self.maintaining_speed,
                )
            else:
                self.phase = 2

        # stage 2 - angle towards outside of ball
        if self.phase == 2:
            drive_target = Vector([4500 * sign(self.agent.team), 0, 0])
            if (
                distance2D(self.agent.me.location, self.agent.ball.location) > 3875
            ):  # 3875:
                return driveController(
                    self.agent,
                    drive_target,
                    0,
                    expedite=True,
                    flippant=False,
                    maintainSpeed=self.maintaining_speed,
                )
            else:
                self.phase = 3
        # stage 3 - first flip
        if self.phase == 3:
            if self.firstFlip.active:
                return self.firstFlip.update()
            else:
                if self.agent.onSurface:
                    self.phase = 4
                else:
                    controls = SimpleControllerState()
                    (
                        controls.steer,
                        controls.yaw,
                        controls.pitch,
                        controls.roll,
                        alignment_error,
                    ) = point_at_position(
                        self.agent,
                        self.agent.me.location
                        + self.agent.me.velocity.normalize().flatten().scale(1000),
                    )
                    return controls

        # stage 4 - aim towards just offcenter of ball
        if self.phase == 4:

            if (
                distance2D(self.agent.me.location, self.agent.ball.location) > self.final_flip_dist
                or not self.agent.onSurface
            ):
                drive_target = self.agent.ball.location
                if (self.agent.me.location - self.agent.ball.location)[0] > 0:
                    dummy_location = [25, 0, 0]
                else:
                    dummy_location = [-25, 0, 0]

                return driveController(
                    self.agent,
                    drive_target + Vector(dummy_location),
                    0,
                    expedite=not self.agent.superSonic,
                    flippant=False,
                    maintainSpeed=self.maintaining_speed,
                )
            else:
                self.phase = 5

        # stage 5 - flip through center and end kickoff
        # 4 flip left, 5 flip right
        if self.phase == 5:
            if self.secondFlip is None and not self.greedy:
                self.greedy = distance2D(self.agent.me.location, self.agent.ball.location) + self.greedy_req <= distance2D(
                    self.agent.closestEnemyToBall.location, self.agent.ball.location)
                self.secondFlip = LeapOfFaith(self.agent, 0, target=None)
            if not self.greedy:
                controls = self.secondFlip.update()
                if not self.secondFlip.active:
                    self.retire()
            else:
                controls = driveController(
                    self.agent,
                    self.agent.ball.location,
                    0,
                    expedite=not self.agent.superSonic,
                    flippant=False,
                )
                controls.throttle = 0
            if self.agent.ball.location[0] != 0 or self.agent.ball.location[1] != 0:
                self.retire()
            return controls

    def retire(self):
        self.active = False
        if self.secondFlip and self.secondFlip.active:
            self.agent.activeState = self.secondFlip

    def update(self):
        if not self.agent.gameInfo.is_round_active:
            self.setup()

        if not self.agent.gameInfo.is_kickoff_pause:
            self.retire()

        if self.KO_option is not None:
            if not self.KO_option.active:
                self.retire()
            return self.KO_option.update()

        # 0 == wide diagonal, 1 == short disgonal, 2 == middle
        if self.kickoff_type == 0:
            return self.wide_handler()
        elif self.kickoff_type == 1:
            return self.short_handler()
        else:
            return self.middle_handler()


def soloStateManager_testing(agent):
    agentType = type(agent.activeState)

    if (
        agentType != PreemptiveStrike
        or (agentType == PreemptiveStrike and not agent.activeState.active)
    ):
        agent.activeState = PreemptiveStrike(agent)
