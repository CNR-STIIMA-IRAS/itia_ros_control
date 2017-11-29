# itia_ros_control

Extension of the ros_control framework (http://wiki.ros.org/ros_control) with different multi-joints controllers

The 'itia_nodelet_hw_interface' is the core of the package. It introduces the possibility to connect different controller just configuring the input and the output thorugh a simple yaml file.

The controllers in the repository are:
- itia_virtual_sensor: feedback estimation of the external forces thorugh only internal sensors and the identified robot model dynamics 
- itia_model_feedforward: robot dynamics model feedforward
- itia_velocity_control: multi-joint velocity controller
- itia_joint_impedance_control: multi-joint immpedance controller
- itia_cart_impedance_control: Cartesian impedance controller
- itia_cascade_control: multi-joint position-velocity controller
- itia_deflection_control: multi-joint defelceion controller (SEA motor)
- itia_driver_mass_controller:
- itia_fake_controller_system: for simulation pruposes
- itia_helios_controller: a feedforward controller based on a FIR filter that smooth and micro-interpolates each control-loop setpoins.


# Developers Contacts

**Authors:** 

- Manuel Beschi (manuel.beschi@itia.cnr.it)
- Nicola Pedrocchi (nicola.pedrocchi@itia.cnr.it)
 
Software License Agreement (BSD License) Copyright (c) 2016, National Research Council of Italy, Institute of Industrial Technologies and Automation. All rights reserved.

# Acknowledge

**This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 637095.**
