import math

import numpy as np
from rlbot.agents.base_agent import SimpleControllerState

maxPossibleSpeed = 2300


class FlipStatus:
    def __init__(self, _time):
        self.started = False
        self.flipStartedTimer = _time
        self.flipDone = False


class Boost_obj:
    def __init__(self, location, bigBoost, spawned, index):
        self.location = Vector(location)  # list of 3 coordinates
        self.bigBoost = bigBoost  # bool indicating if it's a cannister or just a pad
        self.spawned = spawned  # bool indicating whether it's currently
        self.index = index


class GameInfo:
    def __init__(self):
        self.seconds_elapsed = 0
        self.game_time_remaining = 300
        self.is_overtime = False
        self.is_unlimited_time = False
        self.is_round_active = False
        self.is_kickoff_pause = False
        self.world_gravity_z = -1000
        self.game_speed = 1

    def update(self, gamePacket):
        self.seconds_elapsed = gamePacket.game_info.seconds_elapsed
        self.game_time_remaining = gamePacket.game_info.game_time_remaining
        self.is_overtime = gamePacket.game_info.is_overtime
        self.is_unlimited_time = gamePacket.game_info.is_unlimited_time
        self.is_round_active = gamePacket.game_info.is_round_active
        self.is_kickoff_pause = gamePacket.game_info.is_kickoff_pause
        self.world_gravity_z = gamePacket.game_info.world_gravity_z
        self.game_speed = gamePacket.game_info.game_speed


class physicsObject:
    def __init__(self):
        self.location = Vector([0, 0, 0])
        self.velocity = Vector([0, 0, 0])
        self.rotation = Vector([0, 0, 0])
        self.avelocity = Vector([0, 0, 0])
        self.rotational_velocity = Vector([0, 0, 0])
        self.local_location = Vector([0, 0, 0])
        self.boostLevel = 0
        self.team = -1
        self.matrix = []
        self.lastTouch = 0
        self.lastToucher = 0
        self.rot_vector = None
        self.onSurface = False
        self.demolished = False
        self.index = 0
        self.next_hit = None


class Vector:
    def __init__(self, content):  # accepts list of float/int values
        if type(content) == np.array:
            self.data = content.tolist()
        else:
            self.data = content

    def __str__(self):
        return str(self.data)

    def __repr__(self):
        return str(self)

    def __len__(self):
        return len(self.data)

    def __getitem__(self, item):
        return self.data[item]

    def raiseLengthError(self, other, operation):
        raise ValueError(
            f"Tried to perform {operation} on 2 vectors of differing lengths"
        )

    def raiseCrossError(self):
        raise ValueError("Both vectors need 3 terms for cross product")

    def __mul__(self, other):
        if len(self.data) == len(other.data):
            return Vector([self.data[i] * other[i] for i in range(len(other))])
        else:
            self.raiseLengthError(other, "multiplication")

    __rmul__ = __mul__

    def __eq__(self, other):
        try:
            return self.data == other.data
        except:
            return False

    def __add__(self, other):
        if len(self.data) == len(other.data):
            return Vector([self.data[i] + other[i] for i in range(len(other))])
        else:
            self.raiseLengthError(other, "addition")

    def __sub__(self, other):
        if len(self.data) == len(other.data):
            return Vector([self.data[i] - other[i] for i in range(len(other))])
        else:
            self.raiseLengthError(other, "subtraction")

    def align_to(self, rot):
        v = Vector([self[0], self[1], self[2]])
        v.data = [
            v[0],
            math.cos(rot[2]) * v[1] + math.sin(rot[2]) * v[2],
            math.cos(rot[2]) * v[2] - math.sin(rot[2]) * v[1],
        ]
        v.data = [
            math.cos(-rot[1]) * v[0] + math.sin(-rot[1]) * v[2],
            v[1],
            math.cos(-rot[1]) * v[2] - math.sin(-rot[1]) * v[0],
        ]
        v.data = [
            math.cos(-rot[0]) * v[0] + math.sin(-rot[0]) * v[1],
            math.cos(-rot[0]) * v[1] - math.sin(-rot[0]) * v[0],
            v[2],
        ]

        return v

    def align_from(self, rot):
        v = Vector([self[0], self[1], self[2]])
        v.data = [
            math.cos(rot[0]) * v[0] + math.sin(rot[0]) * v[1],
            math.cos(rot[0]) * v[1] - math.sin(rot[0]) * v[0],
            v[2],
        ]
        v.data = [
            math.cos(rot[1]) * v[0] + math.sin(rot[1]) * v[2],
            v[1],
            math.cos(rot[1]) * v[2] - math.sin(rot[1]) * v[0],
        ]
        v.data = [
            v[0],
            math.cos(-rot[2]) * v[1] + math.sin(-rot[2]) * v[2],
            math.cos(-rot[2]) * v[2] - math.sin(-rot[2]) * v[1],
        ]

        return v

    def crossProduct(self, other):
        if len(self.data) == 3 and len(other.data) == 3:
            newVec = [0, 0, 0]
            newVec[0] = self[1] * other[2] - self[2] * other[1]
            newVec[1] = self[2] * other[0] - self[0] * other[2]
            newVec[2] = self[0] * other[1] - self[1] * other[0]

            return Vector(newVec)

        else:
            self.raiseCrossError()

    def magnitude(self):
        return abs(math.sqrt(sum([x * x for x in self])))

    def normalize(self):
        mag = self.magnitude()
        if mag != 0:
            return Vector([x / mag for x in self])
        else:
            return Vector([0 for _ in range(len(self.data))])

    def dotProduct(self, other):
        product = 0
        for i, j in zip(self, other):
            product += i * j
        return product

    def scale(self, scalar):
        return Vector([x * scalar for x in self.data])

    def correction_to(self, ideal):
        current_in_radians = math.atan2(self[1], -self[0])
        ideal_in_radians = math.atan2(ideal[1], -ideal[0])

        correction = ideal_in_radians - current_in_radians
        if abs(correction) > math.pi:
            if correction < 0:
                correction += 2 * math.pi
            else:
                correction -= 2 * math.pi

        return correction

    def flatten(self):
        return Vector(self.data[:2] + [0])

    def toList(self):
        return self.data

    def lerp(self, otherVector, percent):  # percentage indicated 0 - 1
        percent = clamp(1, 0, percent)
        originPercent = 1 - percent

        scaledOriginal = self.scale(originPercent)
        other = otherVector.scale(percent)
        return scaledOriginal + other

    def cap(self, limit):
        if self.magnitude() > limit:
            self.data = self.normalize().scale(limit).data


def butterZone(vec: Vector, x: float = 800, y: float = 4400):
    return abs(vec.data[0]) < x and abs(vec.data[1]) > y


def steer_handler(angle, rate):
    final = ((35 * (angle + rate)) ** 3) / 20
    return clamp(1, -1, final)


def add_car_offset(agent):
    up = agent.up.scale(agent.defaultOffset[2])
    forward = agent._forward.scale(agent.defaultOffset[0])
    left = agent.left.scale(agent.defaultOffset[1])
    agent.me.location = agent.me.location + up + forward + left
    if not agent.roof_height:
        agent.roof_height = math.floor(
            (agent.me.location + agent.up.scale(agent.carHeight * 0.5))[2]
        )


class ballTouch:
    def __init__(self, touchInfo):
        self.player_name = touchInfo.player_name
        self.hit_location = touchInfo.hit_location
        self.team = touchInfo.team
        self.player_index = touchInfo.player_index
        self.time_seconds = touchInfo.time_seconds

    def __repr__(self):
        valueString = f"""
        player_name = {self.player_name}
        hit_location = {self.hit_location}
        team = {self.team}
        player_index = {self.player_index}
        time_seconds = {self.time_seconds}
        """
        return valueString

    def __eq__(self, other):
        if type(other) != ballTouch:
            # raise ValueError(
            #     f"Can not do comparisan operations of balltouch and {type(other)} objects."
            # )
            return False

        if self.player_name != other.player_name:
            return False

        if self.hit_location != other.hit_location:
            return False

        if self.team != other.team:
            return False

        if self.player_index != other.player_index:
            return False

        if self.time_seconds != other.time_seconds:
            return False

        return True


def convertStructLocationToVector(struct):
    return Vector(
        [
            struct.physics.location.x * 1.0,
            struct.physics.location.y * 1.0,
            struct.physics.location.z * 1.0,
        ]
    )


def convertStructVelocityToVector(struct):
    return Vector(
        [
            struct.physics.velocity.x * 1.0,
            struct.physics.velocity.y * 1.0,
            struct.physics.velocity.z * 1.0,
        ]
    )


def flipHandler(agent, flip_status):
    if flip_status.started:
        if agent.time - flip_status.flipStartedTimer <= 0.10:
            jump = True
        else:
            jump = False
    else:
        jump = True
        flip_status.started = True
        flip_status.flipStartedTimer = agent.time

    if agent.time - flip_status.flipStartedTimer >= 0.15:
        jump = True

    if agent.time - flip_status.flipStartedTimer >= 0.45:
        flip_status.flipDone = True

    return jump


def rotator_to_matrix(our_object):
    r = our_object.rotation
    CR = math.cos(r[2])
    SR = math.sin(r[2])
    CP = math.cos(r[0])
    SP = math.sin(r[0])
    CY = math.cos(r[1])
    SY = math.sin(r[1])

    matrix = [
        Vector([CP * CY, CP * SY, SP]),
        Vector([CY * SP * SR - CR * SY, SY * SP * SR + CR * CY, -CP * SR]),
        Vector([-CR * CY * SP - SR * SY, -CR * SY * SP + SR * CY, CP * CR]),
    ]

    return matrix


def getLocation(_object):
    if hasattr(_object, "data"):
        return _object
    if hasattr(_object, "location"):
        return _object.location
    raise ValueError(
        f"{str(type(_object))} is not a valid input for 'getLocation' function "
    )


def clamp(_max, _min, value):
    if value > _max:
        return _max
    if value < _min:
        return _min
    return value


def sign(x):
    if x <= 0:
        return -1
    else:
        return 1


def distance1D(origin, destination, index):
    return abs(getLocation(origin)[index] - getLocation(destination)[index])


def findEnemyClosestToLocation(agent, location, demo_bias=False):
    if len(agent.enemies) > 0:
        closest = agent.enemies[0]
        cDist = math.inf
        for e in agent.enemies:
            x = math.inf if (demo_bias and e.demolished) else findDistance(e.location, location)
            if x < cDist:
                cDist = x
                closest = e
        return closest, cDist
    else:
        return None, None


def max_speed_of_curvature(c: float) -> float:
    if c >= 0.0069:
        return 0
    if c > 0.00398:
        return (0.0069 - c) / 0.00000584
    if c > 0.00235:
        return 500 + (0.00398 - c) / 0.00000584
    if c > 0.001375:
        return 1000 + (0.00235 - c) / 0.00000584
    if c > 0.0011:
        return 1500 + (0.001375 - c) / 0.0000011
    if c > 0.00088:
        return 1750 + (0.0011 - c) / 0.0000004

    return 2400


def radius_from_local_point(a):
    try:
        a = a.flatten()
        return 1 / (2 * a[1] / a.dotProduct(a))
    except ZeroDivisionError:
        return 0.000000001


def find_arc_distance(angle_degrees, radius):
    circumference = math.pi * 2 * radius
    return circumference * abs(angle_degrees) / 180


def find_curve(agent, global_target):
    x = agent._forward.dotProduct(global_target - agent.me.position)
    y = agent.left.dotProduct(global_target - agent.me.position)
    return (2 * y) / (x * x + y * y)


def get_path_info(local_target, angle):
    radius = radius_from_local_point(local_target)
    c = 1 / radius
    distance = find_arc_distance(angle, radius)
    return abs(radius), abs(c), abs(distance)


def maxSpeedAdjustment(agent, angle, _distance, curve):
    _angle = abs(angle)
    if _angle > 180:
        _angle -= 180

    if (_angle <= 60 or _angle >= 120) and _distance <= agent.ball_size + 30:
        return maxPossibleSpeed

    return max_speed_of_curvature(curve)


def getVelocity(_obj):
    return math.sqrt(sum([x * x for x in _obj]))


def findDistance(origin, destination):
    difference = getLocation(origin) - getLocation(destination)
    return abs(math.sqrt(sum([x * x for x in difference])))


def distance2D(origin, destination):
    _origin = getLocation(origin)
    _destination = getLocation(destination)
    _origin = Vector([_origin[0], _origin[1]])
    _destination = Vector([_destination[0], _destination[1]])
    difference = _origin - _destination
    return abs(math.sqrt(sum([x * x for x in difference])))


def correctAngle(x):
    y = x * 1
    if y > 360:
        y -= 360
    if y < -360:
        y += 360

    if y > 180:
        y = 360
    elif y < -180:
        y += 360

    return y


def localizeVector(target_object, our_object, remote_location=None):
    if remote_location is None:
        x = (getLocation(target_object) - getLocation(our_object.location)).dotProduct(
            our_object.matrix[0]
        )
        y = (getLocation(target_object) - getLocation(our_object.location)).dotProduct(
            our_object.matrix[1]
        )
        z = (getLocation(target_object) - getLocation(our_object.location)).dotProduct(
            our_object.matrix[2]
        )

    else:
        x = (getLocation(target_object) - remote_location).dotProduct(our_object.matrix[0])
        y = (getLocation(target_object) - remote_location).dotProduct(our_object.matrix[1])
        z = (getLocation(target_object) - remote_location).dotProduct(our_object.matrix[2])

    return Vector([x, y, z])


def localizeRotation(target_rotation, agent):
    return Vector(
        [
            target_rotation.dotProduct(agent._forward),
            target_rotation.dotProduct(agent.left),
            target_rotation.dotProduct(agent.up),
        ]
    )


def toLocal(target, our_object):
    return localizeVector(target, our_object)


def driveController(
        agent,
        target,
        arrivalTime,
        expedite=False,
        flippant=False,
        maintainSpeed=False,
):
    tta = clamp(6, 0.001, arrivalTime - agent.time)

    flips_enabled = abs(agent.me.location[1]) <= 5100

    _distance = distance2D(agent.me.location, target)

    localTarget = toLocal(target, agent.me)
    angle = math.atan2(localTarget[1], localTarget[0])
    angle_degrees = math.degrees(angle)
    avoiding = False
    target_to_avoid = Vector([0, 0, 0])
    boost_req = agent.boost_req

    goForward = agent.forward

    if avoiding:
        localTarget = toLocal(target_to_avoid, agent.me)
        angle = math.atan2(localTarget[1], localTarget[0])
        angle_degrees = math.degrees(angle)

    if _distance < 700 or agent.currentSpd < 500 or agent.goalPred is not None:
        goForward = abs(angle_degrees) <= 100


    if maintainSpeed:
        goForward = True

    if not goForward:
        expedite = True
        flips_enabled = True
        angle_degrees -= 180
        if angle_degrees < -180:
            angle_degrees += 360
        if angle_degrees > 180:
            angle_degrees -= 360

        angle = math.radians(angle_degrees)

    path_info = get_path_info(localTarget, angle_degrees)  # radius, c, distance

    idealSpeed = clamp(maxPossibleSpeed, 0, math.ceil(_distance / tta))

    if goForward:
        throttle = 1
    else:
        throttle = -1

    if not goForward:
        if agent.onSurface and arrivalTime > 1.85:
            if (
                    clamp(math.inf, 1, _distance - 160)
                    > clamp(maxPossibleSpeed, agent.currentSpd, agent.currentSpd + 500)
                    * 1.85
            ):
                if abs(angle_degrees) <= clamp(5, 0, _distance / 1000):
                    if not agent.onWall and flips_enabled and goForward == agent.forward:
                        if not goForward:
                            agent.setHalfFlip()
                        else:
                            agent.setJumping(1)

    boost = False

    steer, handbrake = rockSteer(angle, _distance, modifier=300, turnLimit=1)

    if 40 < abs(angle_degrees) < 140:
        if 250 > _distance > 30:
            if agent.currentSpd <= 600:
                if tta < 0.5:
                    agent.setJumping(6, target=target)
        if (
                abs(90 - angle_degrees) * 10 >= _distance > 30
                and agent.currentSpd <= 600
                and not maintainSpeed
                and not agent.demo_monster
        ):
            handbrake = True

    if not goForward and not agent.forward:
        steer = -steer

    if goForward != agent.forward:
        steer = -steer

    if avoiding:
        steer = -steer

    nearTurnLimit = False

    speed_limit = maxSpeedAdjustment(agent, angle_degrees, _distance, path_info[1])
    if abs(angle_degrees) > 90:
        limit = max(agent.currentSpd, 600)
        if limit < speed_limit:
            speed_limit = limit

    if agent.currentSpd > idealSpeed:
        required_decel = idealSpeed - agent.currentSpd
        braking_power = -3500 * tta
        mode = 0
        if nearTurnLimit and not maintainSpeed:
            mode = 1

        if mode == 0:
            if agent.rotationNumber == 1 and (
                    _distance < 50
                    or tta < agent.fakeDeltaTime * 5
                    or nearTurnLimit
                    or (braking_power - (agent.active_decel * 2) * -1) < agent.currentSpd
            ):
                mode = 1
            else:
                mode = 0

        if mode == 0:
            if braking_power + (3500 * agent.fakeDeltaTime * 2) < required_decel:
                throttle = 0.1 if goForward else -0.1
            else:
                throttle = -1 if goForward else 1

        elif mode == 1:
            if required_decel < agent.active_decel:
                throttle = -1 if goForward else 1
            elif required_decel <= agent.coast_decel:
                if (
                        agent.currentSpd
                        - agent.coast_decel * (tta / agent.fakeDeltaTime)
                        <= 0
                ):
                    throttle = 0
                else:
                    throttle = -1 if goForward else 1
            else:
                throttle = 0.1 if goForward else -0.1

        else:
            print("error in drive controller!", agent.time)

    elif agent.currentSpd < idealSpeed:
        if idealSpeed >= agent.currentSpd + boost_req:
            if (
                    agent.me.boostLevel > 0
                    and agent.onSurface
                    and expedite
                    and agent.currentSpd < (maxPossibleSpeed - 50)
                    and goForward
                    and not nearTurnLimit
                    and idealSpeed >= 1000
            ):
                boost = True

        if agent.me.boostLevel > 0 and expedite:
            minFlipSpeed = maxPossibleSpeed - 500
        else:
            minFlipSpeed = 1075

        if agent.currentSpd > minFlipSpeed and flips_enabled:
            if (
                    clamp(math.inf, 1, _distance - 90)
                    > clamp(maxPossibleSpeed, agent.currentSpd, agent.currentSpd + 500)
                    * 1.85
                    or flippant
            ):
                if abs(angle_degrees) <= clamp(5, 0, _distance / 500):
                    if not agent.onWall:
                        if agent.onSurface:
                            if goForward:
                                agent.setJumping(1)
                            else:
                                agent.setHalfFlip()

    else:
        if goForward:
            throttle = 0.1
        else:
            throttle = -0.1

    handbrake_barrier = clamp(0.9, 0.5, agent.currentSpd / maxPossibleSpeed)

    if handbrake:
        if abs(agent.me.avelocity[2]) < handbrake_barrier or agent.forward != goForward:
            handbrake = False
        if agent.currentSpd < speed_limit and not nearTurnLimit:
            handbrake = False

    if handbrake:
        boost = False

    if maintainSpeed:
        handbrake = False
        throttle = 1
        if not agent.demo_monster:
            boost = False

    if boost:
        if not agent.forward or not goForward:
            boost = False

    handbrake = False
    throttle = 1
    if agent.currentSpd < 2200:
        boost = True

    controler = SimpleControllerState()
    controler.steer = steer
    controler.throttle = throttle
    controler.handbrake = handbrake
    controler.boost = boost

    return controler


def Gsteer(angle):
    final = ((10 * angle + sign(angle)) ** 3) / 20
    return clamp(1, -1, final)


def rockSteer(angle, distance, forward=True, modifier=600, turnLimit=1):
    turn = Gsteer(angle)
    # turn = clamp(1,-1,angle*4)
    slide = False
    distanceMod = clamp(10, 0.3, distance / modifier)
    _angle = correctAngle(math.degrees(angle))

    adjustedAngle = _angle / distanceMod
    if abs(turn) >= turnLimit:
        if abs(adjustedAngle) > 90:
            slide = True

    return turn, slide


def findEnemyHits(agent):
    found = False
    if agent.closestEnemyToBall:
        for i in range(0, agent.ballPred.num_slices):
            if i % 5 != 0:
                continue
            pred = agent.ballPred.slices[i]
            if pred.game_seconds - agent.gameInfo.seconds_elapsed <= 0:
                continue
            location = convertStructLocationToVector(pred)

            timeToTarget = enemyArrivalEstimator(
                agent, agent.closestEnemyToBall, location
            )
            if (
                    timeToTarget
                    <= pred.game_seconds - agent.gameInfo.seconds_elapsed
            ):
                agent.enemyBallInterceptDelay = (
                        pred.game_seconds - agent.gameInfo.seconds_elapsed
                )
                agent.enemyTargetVec = location
                found = True
                agent.enemyPredTime = pred.game_seconds
                agent.enemyTargetVel = convertStructVelocityToVector(
                    agent.ballPred.slices[i]
                )
                break

    if not found:
        agent.enemyBallInterceptDelay = 6
        agent.enemyTargetVec = convertStructLocationToVector(
            agent.ballPred.slices[agent.ballPred.num_slices - 2]
        )
        agent.enemyTargetVel = convertStructVelocityToVector(
            agent.ballPred.slices[agent.ballPred.num_slices - 2]
        )
        agent.enemyPredTime = agent.time + 6


def enemyArrivalEstimator(agent, phys_obj, destination):
    distance = clamp(
        math.inf, 0.00001, findDistance(phys_obj.location, destination) - 160,
    )
    moreAccurateEstimation = calcEnemyTimeWithAcceleration(agent, distance, phys_obj)
    return moreAccurateEstimation


def calcEnemyTimeWithAcceleration(agent, distance, enemyPhysicsObject):
    estimatedSpd = abs(enemyPhysicsObject.velocity.magnitude())
    estimatedTime = 0
    distanceTally = 0
    boostAmount = enemyPhysicsObject.boostLevel
    boostingCost = 33.334 * agent.deltaTime
    # print("enemy started")
    while distanceTally < distance and estimatedTime < 6:
        if estimatedSpd < maxPossibleSpeed:
            acceleration = getNaturalAcceleration(
                estimatedSpd, agent.gravity, False
            )
            if boostAmount > 0:
                acceleration += 991.667
                boostAmount -= boostingCost
            if acceleration > 0:
                estimatedSpd += acceleration * agent.deltaTime
            distanceTally += estimatedSpd * agent.deltaTime
            estimatedTime += agent.deltaTime
        else:
            distanceTally += estimatedSpd * agent.deltaTime
            estimatedTime += agent.deltaTime

    return estimatedTime


def lerp(v0, v1, t):  # linear interpolation
    return (1 - t) * v0 + t * v1


def getNaturalAcceleration(currentSpd, gravityValue, onWall):
    normalIncrement = 1440.0 / 1400.0
    topIncrement = 160.0 / 10.0

    if currentSpd <= 1400:
        if not onWall:
            return (1440 - (currentSpd * normalIncrement)) + 160
        else:
            return clamp(
                5000, 0, (1440 - (currentSpd * normalIncrement)) + 160 - gravityValue
            )
    elif currentSpd <= 1410:
        if not onWall:
            return 160 - ((currentSpd - 1400) * topIncrement)
        else:
            return clamp(
                5000, 0, 160 - ((currentSpd - 1400) * topIncrement) - gravityValue
            )
    else:
        return 0


def run_float_simulation(agent, tick_length=3):
    x_limit = 4096 - agent.defaultElevation
    y_limit = 5120 - agent.defaultElevation
    ground_limit = agent.defaultElevation
    ceiling_limit = 2044 - agent.defaultElevation
    aim_direction = None
    simulated_location = agent.me.location.scale(1)
    simulated_velocity = agent.me.velocity.scale(1)
    simulated_time = 0
    agent.last_float_sim_time = agent.time
    sim_frames = []

    while (
            simulated_time < 10
    ):
        simulated_time += agent.fakeDeltaTime * tick_length
        simulated_velocity = simulated_velocity + Vector([0, 0, agent.gravity]).scale(
            (agent.fakeDeltaTime) * tick_length
        )
        if simulated_velocity.magnitude() > 2300:
            simulated_velocity = simulated_velocity.normalize().scale(2300)
        simulated_location = simulated_location + simulated_velocity.scale(
            (agent.fakeDeltaTime) * tick_length
        )
        sim_frames.append([simulated_location, simulated_velocity, agent.time + simulated_time])


        if simulated_location[2] >= ceiling_limit:
            agent.roll_type = 2
            agent.squash_index = 2

            aim_direction = Vector([0, 0, 1])
            break
        if simulated_location[2] <= ground_limit:
            agent.roll_type = 1
            agent.squash_index = 2

            break

        if simulated_location[0] <= -x_limit:
            # on blue's right wall

            agent.squash_index = 0
            if simulated_velocity[1] < 0:
                # need to keep top right
                agent.roll_type = 4

            else:
                # need to keep top left
                agent.roll_type = 3
            break

        if simulated_location[0] >= x_limit:
            # on blue's left wall
            agent.squash_index = 0

            if simulated_velocity[1] < 0:
                # need to keep top left
                agent.roll_type = 3

            else:
                # need to keep top right
                agent.roll_type = 4
            break

        if simulated_location[1] <= -y_limit:
            # on blue's backboard

            if abs(simulated_location[0]) < 893:
                if simulated_location[2] < 642:
                    agent.roll_type = 1
                    agent.squash_index = 2
                    break
            agent.squash_index = 1
            if simulated_velocity[0] < 0:
                # need to keep top left
                agent.roll_type = 3

            else:
                # need to keep top right
                agent.roll_type = 4
            break

        if simulated_location[1] >= y_limit:
            # on orange's backboard

            if abs(simulated_location[0]) < 893:
                if simulated_location[2] < 642:
                    agent.roll_type = 1
                    agent.squash_index = 2
                    break
            agent.squash_index = 1
            if simulated_velocity[0] < 0:
                # need to keep top right
                agent.roll_type = 4

            else:
                # need to keep top left
                agent.roll_type = 3
            break
    if simulated_time >= 10:
        agent.roll_type = 1
        agent.squash_index = 2

    if aim_direction is None:
        agent.aim_direction = Vector([0, 0, -1])
    else:
        agent.aim_direction = aim_direction

    agent.collision_timer = simulated_time
    agent.on_ground_estimate = agent.time + simulated_time
    agent.collision_location = simulated_location
    simulated_velocity.data[agent.squash_index] = 0
    agent.simulated_velocity = simulated_velocity

    agent.wall_landing = agent.roll_type == 1
    agent.sim_frames = sim_frames


def turnController(_angle, turnRate):
    return clamp(1, -1, (_angle + turnRate) * 7)


def point_at_position(agent, position: Vector):
    local_position = toLocal(position, agent.me)

    yaw_angle = math.atan2(local_position[1], local_position[0])
    steer = turnController(yaw_angle, 1)
    yaw = turnController(yaw_angle, -agent.me.rotational_velocity[2] / 4)
    pitch_angle = math.atan2(local_position[2], local_position[0])
    pitch = turnController(pitch_angle, agent.me.rotational_velocity[1] / 4)
    roll = turnController(-agent.me.rotation[2], agent.me.rotational_velocity[0] / 4)

    return steer, yaw, pitch, roll, abs(yaw_angle) + abs(pitch_angle)


def matrixDot(_matrix, vector):
    return Vector(
        [
            _matrix[0].dotProduct(vector),
            _matrix[1].dotProduct(vector),
            _matrix[2].dotProduct(vector),
        ]
    )
