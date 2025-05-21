The enclosed folder contains all logic for the system

    cam_stich.py: This file contains the background script responsible for stiching the pointclouds from our multiple camera units

    read_pico.py/pico_sensor_node.py: Scripts responsible for communication between the ros master and the onboard sensors connected via pico

    point_cloud_parser: This file is responsible for cropping the region of interest(excluding the floor) from the stiched point cloud 

    occupancy_grid.py: This contains the definition of the class object responsible for maintaining and updating the occupancy grid

    long_path_planning.py: This contains all code for running our modified a* algorithm from an occupancy grid

    simulate_path.py: A script that when passed occupancy grid and path information, creates an mp4 simulation of the path
        sim.mp4 is an example file

     orchestrator.py: This is the script that combines the objects and functionalities of all other scripts to orchestrate our demo
        -run from main function calling orchestrator.run_live() with parameters