max_takeoff_height = 1000  # 1000 cm
target_relative_degrees = 0
target_reached = False
home_reached = False
home_gps_coords = 0
target_gps_coords = 0

home_gps_coords = get_current_gps()

# Takeoff sequence
for i in range(max_takeoff_height):
    if top_lidar_reading() > 100:  # Checks area above drone for greater than 100 cm
        takeoff(i)  # Calls takeoff with current height
    else:
        print("Obstacle Above - Mission ABORTED")
        land_vehicle()  # Calls landing command
        exit(-1)

# Vehicle in air - navigate to target
while not target_reached:
    current_gps = get_current_gps()
    target_relative_degrees, distance = get_target_relative_degrees_and_mag(target_gps_coords)
    print(f"{distance} Meters Away From Target - {100 * (1 - distance/initial_distance)}% Complete")
    
    get_360_obstacles()  # Stores lidar values in obstacle_table in 18 degree groups
    
    direction = find_most_open_18_degrees_favoring_target_relative_degrees()
    
    fly_towards_most_open_18_degrees(50)  # Fly that direction 50 cm
    
    if calculate_distance(current_gps, target_gps_coords) < 100:  # Within 100 cm
        land_vehicle()
        print("Vehicle At TARGET Location")
        deliver_package()  # Opens actuator, then closes actuator
        print("Package DELIVERED")
        target_reached = True

# Return home sequence
target_gps_coords = home_gps_coords  # New goal - fly to home location

# Takeoff sequence (repeated)
for i in range(max_takeoff_height):
    if top_lidar_reading() > 100:  # Checks area above drone for greater than 100 cm
        takeoff(i)
    else:
        print("Obstacle Above - Mission ABORTED")
        land_vehicle()
        exit(-1)

# Navigate home
while not home_reached:
    current_gps = get_current_gps()
    target_relative_degrees, distance = get_target_relative_degrees_and_mag(target_gps_coords)
    print(f"{distance} Meters Away From Home - {100 * (1 - distance/initial_distance)}% Complete")
    
    get_360_obstacles()
    
    direction = find_most_open_18_degrees_favoring_target_relative_degrees()
    
    fly_towards_most_open_18_degrees(50)
    
    if calculate_distance(current_gps, target_gps_coords) < 100:
        land_vehicle()
        print("Vehicle At HOME Location")
        home_reached = True

print("Mission COMPLETED")
exit(0)
