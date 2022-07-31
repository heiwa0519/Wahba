## Implementations of the major developments in 3D-attitude determination of spacecraft.

#### Original problem
<br/>
<p align="center">
  <img src="https://github.com/risherlock/Wahba/blob/master/docs/wahba_original_problem.PNG" width="800">
</p>

#### Implementations
1. [TRIAD: A Passive System for Determining the Attitude of a Satellite (1964)][black1964]
2. [Davenport's q method: A Vector Approach to the Algebra of Rotations with Applications (1968)][davenport1968]
3. [SVD method: Attitude Determination using Vector Observations and Singular Value Decomposition (1968)][markley1968]
4. [QUEST: Three-axis Attitude Determination from Vector Observations (1981)][shuster1981]
5. [FOAM: Attitude Determination using Vector Observations, A Fast Optimal Matrix Algorithm (1993)][markley1993]
6. [An Analytic Solution to Wahba's Problem (2013)][yang2013]
7. [Attitude Determination using Newton's Method on Riemannian Manifold (2015)][yang2015]
8. [FLAE: Fast Linear Quaternion Attitude Estimator Using Vector Observations (2017)][wu2017_newton] (Newton's method)
9. [FLAE: Fast Linear Quaternion Attitude Estimator Using Vector Observations (2017)][wu2017_symbolic] (Symbolic method)

#### Technical report
My note [*Mathematical Introduction to Attitude Determination*][mathemtical_wahba] documents the introduction to *attitude determination* (in constrast to the *attitude estimation* which utilizes estimation algoriths like Kalman Filter) of spacecraft and general approaches to the solution of Wahba's problem. It, however, does not contain the detail of any of above algorithms (because respective papers does this in more rigour then I could ever do). More emphasis is give on the analysis of the problem from the mathematical point of view. 

If you are interested in Kalman filter based attitude estimation algorithms for spacecrafts, you might like [Attitude-Estimation](https://github.com/risherlock/Attitude-Estimation).

[black1964]: https://github.com/risherlock/Wahba/blob/master/matlab/algorithms/triad1964.m
[davenport1968]: https://github.com/risherlock/Wahba/blob/master/matlab/algorithms/davenport1968.m
[markley1968]: https://github.com/risherlock/Wahba/blob/master/matlab/algorithms/svd1968.m
[shuster1981]: https://github.com/risherlock/Wahba/blob/master/matlab/algorithms/quest1981.m
[markley1993]: https://github.com/risherlock/Wahba/blob/master/matlab/algorithms/foam1993.m
[yang2013]: https://github.com/risherlock/Wahba/blob/master/matlab/algorithms/yang_analytical2013.m
[yang2015]: https://github.com/risherlock/Wahba/blob/master/matlab/algorithms/yang_manifold2015.m
[wu2017_newton]: https://github.com/risherlock/Wahba/blob/master/matlab/algorithms/flae_newton2017.m
[wu2017_symbolic]: https://github.com/risherlock/Wahba/blob/master/matlab/algorithms/flae_symbolic2017.m
[mathemtical_wahba]: https://github.com/risherlock/Wahba
