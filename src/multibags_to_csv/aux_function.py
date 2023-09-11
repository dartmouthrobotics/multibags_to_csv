def extract_robot_number(bag_file_name):
    """
    find robot number from the bag file name

    Args:
        bag_file_name (str): full path name
    
    Returns:
        robot_number (int)
    """

    count = 0
    range_interest = []

    if 'rand_' in bag_file_name:
        file_name_start_idx = int(bag_file_name.find('rand_scenario_')) # type str -> int cast

        for i, s in enumerate(bag_file_name[file_name_start_idx:]):
            if s == "_" and count < 3: # upto scenario_ is two
                count += 1
                range_interest.append(i + file_name_start_idx)

        robot_number = bag_file_name[range_interest[1] + 1: range_interest[2]]
        return int(robot_number)

    else:
        file_name_start_idx = int(bag_file_name.find('scenario_')) # type str -> int cast

        for i, s in enumerate(bag_file_name[file_name_start_idx:]):
            if s == "_" and count < 2:
                count += 1
                range_interest.append(i + file_name_start_idx)

        robot_number = bag_file_name[range_interest[0] + 1: range_interest[1]]
        return int(robot_number)