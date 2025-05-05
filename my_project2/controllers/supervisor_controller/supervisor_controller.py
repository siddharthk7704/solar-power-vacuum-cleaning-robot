from controller import Supervisor
import math
import random

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

battery_level = 1.0
charging_rate = 0.01
consumption_rate = 0.005
battery_warning_printed = False


robot_node = robot.getFromDef("SolarVacuumBot")
if robot_node is None:
    print("❌ Error: 'SolarVacuumBot' not found. Check DEF name.")
    exit()

translation_field = robot_node.getField("translation")

dust_names = ["Dust1", "Dust2", "Dust3", "Dust4", "Dust5", "Dust6", "Dust7", "Dust8", "Dust9", "Dust10"]
cleaned_count = 0

spotlight_node = robot.getFromDef("SpotLight")
if spotlight_node is None:
    print("❌ Error: 'SpotLight' not found. Please add DEF name to your light.")
    exit()

charging_size = [0.3, 0.3]

# Obstacles
obstacle_names = ["Obstacle1", "Obstacle2", "Obstacle3"]
obstacles = []
for name in obstacle_names:
    node = robot.getFromDef(name)
    if node:
        pos = node.getField("translation").getSFVec3f()
        obstacles.append([pos[0], pos[1]])
    else:
        print(f"⚠️ Warning: Obstacle '{name}' not found.")

obstacle_radius = 0.12  # Detection range

def get_charging_center():
    loc = spotlight_node.getField("location").getSFVec3f()
    return [loc[0], loc[1]]

def get_distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def is_near_obstacle(pos):
    for obs in obstacles:
        if get_distance([pos[0], pos[1]], obs) < obstacle_radius:
            return True
    return False

def get_alternate_direction(current, target):
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    base_angle = math.atan2(dy, dx)

    angles_to_try = [0, math.pi/4, -math.pi/4, math.pi/2, -math.pi/2, math.pi, -math.pi]  # 0°, ±45°, ±90°, 180°
    step_size = 0.01

    for angle_offset in angles_to_try:
        angle = base_angle + angle_offset
        new_x = current[0] + step_size * math.cos(angle)
        new_y = current[1] + step_size * math.sin(angle)
        if not is_near_obstacle([new_x, new_y]):
            return [new_x, new_y, current[2]]

    # If no clear path, stay still
    return current


def move_toward(target, current):
    global battery_warning_printed

    dx = target[0] - current[0]
    dy = target[1] - current[1]

    if battery_level > 0.5:
        step_size = 0.008
        battery_warning_printed = False  # Reset if battery has recovered
    elif battery_level > 0.1:
        if not battery_warning_printed:
            print("35% battery left, Power Saving Mode ON")
            battery_warning_printed = True
        step_size = 0.005
    else:
        step_size = 0.004

    angle = math.atan2(dy, dx)
    new_x = current[0] + step_size * math.cos(angle)
    new_y = current[1] + step_size * math.sin(angle)
    return [new_x, new_y, current[2]]


def is_in_charging_zone(pos, center):
    x, y = pos[0], pos[1]
    half_w, half_h = charging_size[0] / 2, charging_size[1] / 2
    return (center[0] - half_w <= x <= center[0] + half_w and
            center[1] - half_h <= y <= center[1] + half_h)

charging_mode = False

while robot.step(timestep) != -1:
    if not dust_names:
        print(f"\n🎉 All dust cleaned!")
        print(f"🧹 Total cleaned: {cleaned_count}")
        print(f"🔋 Final battery level: {battery_level:.2f}")
        print("✅ Shutting down cleaning process.")
        break

    current_pos = translation_field.getSFVec3f()

    if charging_mode:
        charging_center = get_charging_center()
        if is_in_charging_zone(current_pos, charging_center):
            battery_level = min(1.0, battery_level + charging_rate)
            print(f"🔋 Charging... Battery: {battery_level:.2f}")
            if battery_level >= 1.0:
                charging_mode = False
                print("✅ Fully charged. Resuming cleaning.")
        else:
            if is_near_obstacle(current_pos):
                print("⛔ Near obstacle during charging. get_alternate_directionping...")
                new_pos = get_alternate_direction(current_pos)
            else:
                new_pos = move_toward(charging_center, current_pos)
            translation_field.setSFVec3f(new_pos)
        continue

    # Dynamic consumption: higher usage when battery is high, lower when low
    max_rate = 0.006
    min_rate = 0.002
    consumption = min_rate + (max_rate - min_rate) * battery_level
    battery_level = max(0.0, battery_level - consumption)

    if battery_level < 0.1:
        print("⚠️ Battery very low. Going to charge.")
        charging_mode = True
        continue

    target_dust = None
    min_distance = float('inf')

    for dust_name in dust_names:
        dust_node = robot.getFromDef(dust_name)
        if dust_node:
            dust_pos = dust_node.getField("translation").getSFVec3f()
            dist = get_distance(current_pos, dust_pos)
            if dist < min_distance:
                min_distance = dist
                target_dust = (dust_name, dust_node, dust_pos)

    if target_dust:
        _, dust_node, dust_pos = target_dust
        if is_near_obstacle(current_pos):
            print("🚧 Avoiding obstacle near dust. get_alternate_directionping...")
            new_pos = get_alternate_direction(current_pos)
        else:
            new_pos = move_toward(dust_pos, current_pos)

        translation_field.setSFVec3f(new_pos)

        if get_distance([new_pos[0], new_pos[1]], dust_pos) < 0.18:
            pull_strength = 1  # 0.0 = no pull, 1.0 = snap to robot
            suction_x = dust_pos[0] + pull_strength * (new_pos[0] - dust_pos[0])
            suction_y = dust_pos[1] + pull_strength * (new_pos[1] - dust_pos[1])
            dust_node.getField("translation").setSFVec3f([suction_x, suction_y, dust_pos[2]])
            robot.step(timestep)
            dust_node.remove()
            dust_names.remove(target_dust[0])
            cleaned_count += 1
            print(f"✅ Cleaned {target_dust[0]} | Total cleaned: {cleaned_count}")
    else:
        print("🛑 No dust detected.")
