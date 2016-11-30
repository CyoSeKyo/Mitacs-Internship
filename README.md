# Mitacs-Internship
Vehicle-Routes-Prediction-on-Bayesian-Classifier

This code uses to use history data of vehicles' routes to predict one specific vehicle's location after given time. This solution is created
by myself after I first gained the basic knowledge of meachine learning. Compared with other researchers, my solution seems to simple and 
the effect is not satisfying. However, it's my first attempt that enabled me to make contributions to the our project.

Roughly speaking, my algorithm has following steps:
1. Training the Bayes Classifier and Properties of each roads. (Details can be seen in 'pseudo code of first code.pdf' and 'mathematics model.pdf')
2. For given vehicle, use Bayes Classifier to predict next street that the vehicle will go.
3. Based on properties of next street, get the location of vehicle. If vehicle leaves the street in given time, decline the time used in 
this street and goto to step2.

And I tested the code using the data created in Sumo:
Given the time T, the average distance between prediction and reality is 0.3 * T * v(v is the velocity of the vehicle).
