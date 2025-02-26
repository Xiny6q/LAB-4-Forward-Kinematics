Download link :https://programming.engineering/product/lab-4-forward-kinematics/


# LAB-4-Forward-Kinematics
LAB 4 Forward Kinematics
4.1 Important

Read the entire lab before starting and especially the \Grading” section so you are aware of all due dates and requirements associated with the lab. Hope-fully you are reading this well before your lab section meets as given the com-pressed schedule, it is very important that you arrive at lab well prepared. This semester, the more you do prior to your lab session, the more you will get out of the short time you have with the TA.

4.2 Objectives

The purpose of this lab is to implement the exponential forward kinematics on the UR3e robot to estimate the end-e ector pose given a set of joint angles. In this lab you will:

Solve the exponential forward kinematic equations for the UR3e.

Write a Python function that moves the UR3e to a con guration speci ed by the user.

Compare your exponential forward kinematic estimation with the actual robot movement.

4.3 References

Chapter 4 of Modern Robotics provides details of how to construct an exponential forward kinematic for an open-chain robot.


4.4. TASKS

4.4 Tasks

4.4.1 Theoretical Solution

Find the forward kinematic equations for the UR3e robot using the exponential forward kinematics method. Solve for T06 and record the 6 matrices de ned by e[S1] 1 through e[S6] 6 .

4.4.2 Physical Implementation

The user will provide six joint angles f 1; 2; 3; 4; 5; 6g, all given in degrees.

The angle ranges are as follows:

90 <

1

< 90

180 <

2

< 0

5 <

3

< 140

95 <

4

< 5

115 <

5

< 70

170 <

6

< 170

Note that these ranges are approximate and vary slightly from machine to ma-chine. If you receive a protective stop, note the a ected joint on the teach pendant and make appropriate corrections. It is also worth noting that not all con gurations are achievable or desirable. Some will strike the table or wall or be di cult to measure accurately. These limits are primarily for the real robot and will not a ect the simulation, but it is advised that you make use of them to achieve realistic con gurations.

Once the angles are selected, using the ROS Python program move the joints to these desired angles. Your program should additionally calculate and display the translation matrix T06 that rotates and translates the end e ector’s coordinate frame to the base frame.

4.4.3 Comparison

For any provided set of joint angles f 1; 2; 3; 4; 5; 6g, compare the measured position or the position output from the ROS topic with the value calculated by your Python forward kinematics function.

4.5 Procedure

4.5.1 Theoretical Solution

In exponential forward kinematics (especially for an open chain kinemat-ics), there are three components that are essential for the calculation.


4.5. PROCEDURE

A xed base frame f0

Robot’s \zero” con guration (or some might call it initial position) A well-de ned end-e ector frame f6

Practically, you can de ne the base frame, the end-e ector frame, and zero con guration arbitrarily. As long as you remain consistent with your de nitions, the end results will be identical for the same robot. However, doing so will pose di culties when your TA tries to verify your answer or debug your process. Therefore we encourage you to use the same frame placement and zero con guration for both convenience and safety pur-poses. See Figure 4.1. For convenience when measuring, the base frame, f0, is placed at the middle of robot’s base.


Figure 4.1: \zero” con guration and frame placement for UR3e.

Moreover, please note that in this lab, all the UR3e robots have a pre-de ned zero con guration. The pose shown in Figure 4.1 is NOT the TRUE zero con guration according to the teach pendant. In this pose (Figure 4.1), you would read the following joint angles from the teach pendant:

Joint 1

Joint 2

Joint 3

Joint 4

Joint 5

Joint 6

90.0

0.0

0.0

-90.0

0.0

0.0


4.5. PROCEDURE

In the teach pendant’s zero con guration, (due to how the robot is originally mounted) the robot arm would face the far edge of the table and the wrist joint would point into the table, which is an awkward pose to make measurements and do experiments. Hence, we added some angle o sets in the function script such that:

r e t u r n v a l u e [ 0 ] = t h e t a 1 + ( 0 . 5 PI )

r e t u r n v a l u e [ 1 ] = t h e t a 2

r e t u r n v a l u e [ 2 ] = t h e t a 3

(0.5 PI)

r e t u r n v a l u e [ 3 ] = t h e t a 4

r e t u r n

v a l u e [ 4 ]

=

t h e t a 5

r e t u r n

v a l u e [ 5 ]

=

t h e t a 6

(Attention: Remember in the previous lab, when we enter the angle value on control panel into python code, the value of base and elbow should be switched.)

Any joint angle measurements from the robot that are passed to your ROS program will be corrected to the \zero” con guration shown in Figure 4.1. In other words, now if you set all the joint of UR3e to zero (zero con guration) in ROS:

Joint 1

Joint 2

Joint 3

Joint 4

Joint 5

Joint 6

0.0

0.0

0.0

0.0

0.0

0.0

You would see the UR3e poses as Figure 4.1 showed.

The sole purpose of the forward kinematics algorithm is to estimate the end-e ector’s pose (orientation and position) with respect to the base frame given a set of joint angles. This information can be packed and calculated in a \homogeneous transformation matrix” as taught by Lynch & Park in Modern Robotics:

T ( ) = e[S1] 1 e[S2] 2 ::::::e[Sn 1] n 1 e[Sn] n M

While our objective is to solve for the homogeneous transformation matrix T ( ) of the \motion” caused by certain joint angle con gurations, there are seven matrices we need to specify:

The screw axes S1 to S6 (we have six here because UR3e has six joints) expressed in the base frame, corresponding to the joint mo-tions when the robot is at its zero con guration (see Figure 4.1)

The end-e ector pose M (a 4×4 homogeneous transformation matrix) with respect to the base frame, corresponding to the zero con gura-tion


4.5. PROCEDURE


Figure 4.2: The “right hand rule”

A screw axis S can be represented by S = (!; v), where the ! is the joint axis de ned by the right hand rule (Figure 4.2). v can be computed by v = ! q, where q is the displacement (position) of the joint center with respect to the base frame.

Using the symbolic toolbox of Matlab or Python, nd the closed form solution of T06. You should note how long and complex the solution is due to the contribution of each joint’s motion. Additionally, calculate the 6 matrices de ned by e[S1] 1 through e[S6] 6 .

To aid in your calculations, several gures have been included.

Figure 4.3 shows the screw axes and direction of rotation (use the right rule) for each joint.

Figure 4.4 shows the joint centers to aid in locating your q values. Figure 4.5 shows the dimensions of all of the links of the UR3e.

Figure 4.6 shows the dimensions of the end e ector including the aluminum plate and suction gripper.

4.5.2 Implementation on UR3e

Copy lab4pkg fk into \src” directory of your catkin directory. Don’t forget \source devel/setup.bash” when you open a new terminal. Then from your base catkin directory run \catkin make” and if you receive no errors you copied your lab3 starter code correctly. Also so that ROS


4.5. PROCEDURE

registers this new package \lab4pkg fk”, run \rospack list” and you should see lab4pkg fk as one of the many packages.

You will notice that in lab4pkg fk/scripts there are now three *.py les. lab4 exec.py is the main() code, lab4 func.py de nes the important forward kinematic functions. lab4 header.py de nes the header les. We divide them up in this fashion for clarity. In this lab you will be mainly editing lab4 func.py. You can of course change lab4 exec.py but most of its functionality has already been given to you. Study the code and comments in lab4 exec.py and lab4 func.py to see what the starter code is doing and what parts you are going to need to change. Your job is to add the code in the function Get MS() that correctly populates the six screw axes S1 to S6 as well as the transformation matrix M. The Python code uses the \numpy” module to create and multiply matrixes; meanwhile, the matrix exponential \expm()” is achieved by including the \Scipy” module. Then, with the S and M values you populated, write the code in function lab fk() that generates and prints the homogeneous transformation matrix T ( ).

Once your code is nished, run it using \rosrun lab4pkg fk lab4 exec.py [theta1] [theta2] [theta3] [theta4] [theta5] [theta6]” with all angles in degrees – e.g. rosrun lab4pkg py lab4 exec.py 0 90 45 0 -90 105. Remember that in another command prompt you should have rst run roscore and drivers using \roslaunch ur robot driver ur3e bringup.launch”.

For in-person students, you should measure the x,y,z position of the end-e ector using the provided ruler and square. For students using the sim-ulation, it is not possible to measure this way, so we must use another method. A simple way is to use some of the ROS commands we learned before: \rostopic echo /gripper/position -n 1″. These values are be-ing calculated di erently and so there will be small di erences between this value and your calculations.

You should verify that your code works by selecting a variety of poses that will test the full range of motion. Your TA will not be providing you test angles, so you should make use of the joint limits provided in Section 4.4.2.

4.5.3 Comparison

Your TA will select two sets of joint angles f 1; 2; 3; 4; 5; 6g when you demonstrate your working ROS program.

Run your ROS node and move the UR3e to each of these con gurations. With a ruler and square (or the appropriate ROS command for the simu-lation), measure the x,y,z position vector of the center of the gripper for each set of angles. Call these vectors r1 and r2 for the rst and second sets of joint angles, respectively.


4.6. REPORT

Compare these measurements to the translation vector calculated by your ROS node. Call these calculated vectors d1 and d2.

For each set of joint angles, Calculate the error between the measured and calculated kinematic solutions. We will consider the error to be the magnitude of the distance between the measured center of the gripper and

the location predicted by the forward kinematic equation:

q

error1 = kr1 d1k = (r1x d1x)2 + (r1y d1y)2 + (r1z d1z)2:

A similar expression holds for error2.

4.6 Report

Each student will submit a lab report using the guidelines given in the \ECE

How to Write a Lab Report” document. Please be aware of the following:

Lab reports will be submitted online at Blackboard.

The report will be due 2 weeks after your lab session for Lab 3. Exact times and dates can be seen on Blackboard.

Your lab report should include the following:

Explain how you determined S1 thru S6 and M – include gures as needed. Include a gure that shows all frames and axes used.

Include a table of all ! and q used and the v derived.

Figures should be your own creation and not copies of the lab manual. Use Matlab or Python’s symbolic toolbox to generate the nal homoge-

neous transform T06 as a function of ( 1; 2; 3; 4; 5; 6). As this is too cumbersome to include in your report, include e[S1] 1 through e[S6] 6 in your report (do not use screenshots).

Substitute in your given joint angles into your symbolic solution to verify it.

For each test point, include:

{ The given ( 1; 2; 3; 4; 5; 6)

{ The calculated d and measured r vectors { The scalar error

{ The output of your symbolic solution Include a brief discussion of sources of error.

Unless your TA gives other guidance, include the following as appendices:

Your lab4 func.py code (and lab4 exec.py if it was edited). Your Matlab or Python code used to calculate the symbolic T06.


4.7. DEMONSTRATION

4.7 Demonstration

Demonstrations of your working code will either be done in-person or over Zoom. They will be done live (i.e. no recordings) with your lab section TA. Demos are due 2 weeks after your lab session.

The default method of demonstrating your work will be via the simulation. While we want to encourage in-person students to run their code on the real robot, due to time limitations, this may be di cult to do with your TA by the due date. You can always run it for your own satisfaction at another time.

4.7.1 Demo Process

Your TA will require you to run your program twice, each time with a di erent set of joint variables.

4.8 Grading

80 points, successful demonstration. 20 points, individual report.

4.9 Preparation for Lab 5

Lab 5 will cover inverse kinematics. In class you will mostly focus on numerical inverse kinematics, but we will be using analytical inverse kinematics – i.e. we will use geometry and constraints to nd a closed form solution to the problem. Suggested Reading:

Chapter 6 of Modern Robotics provides multiple examples of inverse kine-matics solutions. Especially Section 6.1.

Chapter 3 of Robot Modeling and Control, by M. Spong, S. Hutchinson and M Vidyasagar (Wiley and Sons, 2005). Especially Section 3.3.3. (A version of this text is available online.)


4.9. PREPARATION FOR LAB 5



4.9. PREPARATION FOR LAB 5


Figure 4.5: Approximate Dimensions of the UR3e in mm. *EACH UR3e IS SLIGHTLY DIFFERENT DUE TO ITS MANUFACTURE TOLERANCE.


4.9. PREPARATION FOR LAB 5


Figure 4.6: Approximate Dimensions of the attached end e ector plate and suction gripper in mm.
