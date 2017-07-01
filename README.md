# servo_ax12a
gazebo simulation model for the servo motor ax12a including Geometry, Physics, Electric and SW APIimg 

<src="https://github.com/Roblibs/servo_ax12a/blob/master/media/AX12A.jpg" height=200>


# Status
This project is currently in Step 0

While Steps 1,2 and 3 are the heart of the servo and might in future ebe shared with a generic servo component,
Steps 4 and 5 are much more time consuming, specific to the ax12a but would really bring an added value

# Step 1
- Simple Box including right size and weight
- rotation joint axis with the trurning cylinders
- simple plugin with a topic for direct torque control

# Step 2
- include an electric model that limits the torque depending on the angle speed
- change the interface from direct torque control to current reference with probably a small current loop 

# Step 3
- Position control including approximation of the ax12a internal firmware PID so that the topic can change to a PWM reference signal (just width not binary pulse)

# Step 4
- add more API features provided by the real servo such as torque control, continuous rotation, feedbacks...

# Step 5
(optionally handled at any step)
- Optionnal 3D model for a nice graphics and a more realistic geometric integration

