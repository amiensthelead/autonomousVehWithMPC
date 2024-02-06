# autonomousVehWithMPC
Originally, my control input was the steering angle. Then I have came across to the academical paper "[12] V. P. Mark Misin, “LPV MPC Control of an Autonomous Aerial Vehicle,” in 2020 28th Mediterranean Conference on Control and Automation (MED), France, 2020". Thanks to that paper, I have changed my control input from steering angle to change of steering angle. Therefore, my cost function now also tries to minimize the change of steering angle. In conclusion, the new control input gives better simulation results in return of unpredicted real-life actuation problems of the project.

My references list:

[1] Y. Ding, “ShuffleAI,” 14 11 2021. [Online]. Available: 
https://www.shuffleai.blog/blog/Simple_Understanding_of_Kinematic_Bicycle_
Model.html.
[2] Y. Z. J. L. Jiayu Fan, «A hierarchical control strategy for reliable lane changes 
considering optimal path and lane-changing time point,» Wiley, 2023.
[3] U. Chipengo, “Research Gate,” IEEE, November 2018. [Online]. Available: 
https://www.researchgate.net/figure/Advanced-driver-assistance-systemsADAS-for-active-passive-safety-comfort-functionality_fig1_328920450.
[4] D. L. Guillaume Baffet, “Researchgate,” October 2008. [Online]. Available: 
https://www.researchgate.net/publication/221786406_Estimation_of_TireRoad_Forces_and_Vehicle_Sideslip_Angle. [Accessed January 2024].
[5] The MathWorks, Inc., “Understanding the Model Predictive Control,” 2019. 
[Online]. Available: 
https://www.youtube.com/playlist?list=PLn8PRpmsu08ozoeoXgxPSBKLyd4YEH
ww8.
[6] M. Ulusoy, “Youtube,” MathWorks Inc., 2019. [Online]. Available: 
https://www.youtube.com/playlist?list=PLn8PRpmsu08ozoeoXgxPSBKLyd4YEH
ww8. [Accessed 2023].
[7] P. F. E. D. A. B. Miguel Cerdeira, “Researchgate,” July 2018. [Online]. Available: 
https://www.researchgate.net/publication/326274980_Reset_Controller_Desig
n_Based_on_Error_Minimization_for_a_Lane_Change_Maneuver. [Accessed 
January 2024].
[8] J. Krejsa, “Ackermann mobile robot chassis with independent rear wheel 
drives,” in Power Electronics and Motion Control Conference (EPE/PEMC), Brno, 
October 2010. 
[9] Wikipedia, “Wikipedia,” September 2023. [Online]. Available: 
https://en.wikipedia.org/wiki/Ackermann_steering_geometry. [Accessed 
January 2024].
[10] H. R. S. C. W. W. Gang Liu, “The 3-DoF bicycle model with the simplified 
piecewise linear tire model,” in Proceedings 2013 International Conference on 
Mechatronic Sciences, Electric Engineering and Computer (MEC), Shengyang, 
2013. 
65
[11] D. H. P. K. T. S. Adam Owczarkowski, “Dynamic modeling and simulation of a 
bicycle stabilized by LQR control,” in 2016 21st International Conference on 
Methods and Models in Automation and Robotics (MMAR), Miedzyzdroje, 
Poland, 2016. 
[12] V. P. Mark Misin, “LPV MPC Control of an Autonomous Aerial Vehicle,” in 2020 
28th Mediterranean Conference on Control and Automation (MED), France, 
2020. 
[13] B. B. R. S. R. M. D. D. S. Goran S. Vorotovic, «Determination of Cornering 
Stiffness,» FME Transactions, pp. 66-71, July 2013. 
[14] O. S. Yurii Mariiash, “Implementation of model predictive control in a 
programmable logic controller,” in Modeling, control and information 
technologies: Proceedings of VI International scientific and practical conference, 
Ukraine, 2023. 
[15] Q. L. S. W. L. G. Yonghua Huang, Dynamic modeling and analysis of a frontwheel drive bicycle robot moving on a slope, Hong Kong: IEEE International 
Conference on Automation and Logistics, 2010. 
[16] B. S. Yanggu Zheng, “A Real-Time Nonlinear MPC for Extreme Lateral 
Stabilization of Passenger Vehicles,” in 2019 IEEE International Conference on 
Mechatronics (ICM), Ilmenau, Germany, 2019. 
[17] M. P. Sorawuth Vatanashevanopakorn, “Steering control based balancing of a 
bicycle robot,” in 2011 IEEE International Conference on Robotics and 
Biomimetics, Karon Beach, Thailand, 2011. 
[18] Y. Q. Sisi Pan, “Optimal control via weighted congestion game with linear cost 
functions,” in 2017 36th Chinese Control Conference (CCC), Dalian, China, 2017. 
[19] B. Özece, “MATLAB-Simulink Dersleri (Sistem Dinamiği Uygulamaları),” Udemy, 
2021. [Online]. Available: https://www.udemy.com/course/matlab-simulinkdersleri-temelden-ileri-seviyeye/.
[20] Z. J. T. C. X. J. Lei Li, “Optimal Model Predictive Control for Path Tracking of 
Autonomous Vehicle,” in 2011 Third International Conference on Measuring 
Technology and Mechatronics Automation, Shanghai, China, 2011. 
[21] L. W. C. M. Guofei Xiang, “Trajectory Tracking Control of Transformer Inspection 
Robot Using Distributed Model Predictive Control,” in College of Electrical 
Engineering, Sichuan University, Chengdu, China, 2023. 
66
[22] A. B. M. F. D. H. F. Borrelli, "An MPC/hybrid system approach to traction 
control," IEEE Transactions on Control Systems Technology, pp. 541-562, 24 
April 2006. 
[23] C. U. Dogruer, “Model Predictive Control Parameterized in Terms of Orthogonal 
Polynomials,” in IEEE 11th International Conference on Systems and Control, 
Sousse, Tunisia, 2023. 
[24] T.-D. C. Chih-Keng Chen, “Modeling and Model Predictive Control for a BicycleRider System,” in 2015 2nd International Conference on Information Science and 
Control Engineering, Shanghai, China, 2015. 
[25] M. B. T. Ai-Buraiki, Model predictive control design approach for autonomous 
bicycle kinematics stabiliza-tion, Palermo, Italy: 22nd Mediterranean 
Conference on Control and Automation, 2014. 
[26] MathWorks, Inc., “Specify Cost Function for Nonlinear MPC,” [Online]. 
Available: https://www.mathworks.com/help/mpc/ug/specify-cost-functionfor-nonlinear-mpc.html.
[27] The MathWorks, Inc., “Quadratic programming - MATLAB quadprog,” [Online]. 
Available: 
https://www.mathworks.com/help/optim/ug/quadprog.html?lang=en

Thank you for your interest in my project.
