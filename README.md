# Autonomous Mobile Robots
## Course Structure :space_invader:
    
- Section 0 :alien:	
    - Introduction [:books:](lectures/amr_introduction.pdf) 
- Section 1 :alien:
	- Motion Control  [:books:](lectures/amr_motion.pdf) 
		<ul>
			<li>Kinematics of wheeled mobile robots: internal, external, direct, and inverse </li>
				<ul>
				<li>Differential drive kinematics</li>
				<li>Bicycle drive kinematics</li>
				<li>Rear-wheel bicycle drive kinematics</li>
				<li>Car(Ackermann) drive kinematics</li>
				</ul>
			<li>Wheel kinematics constraints: rolling contact  and lateral slippage </li>
			<li>Wheeled Mobile System Control: pose and orientation
				<ul>
				<li>Control to reference pose</li>
				<li>Control to reference pose via an intermediate point</li>
				<li>Control to reference pose via an intermediate direction</li>
				<li>Control by a straight line and a circular arc</li>
				<li>Reference path control</li>
				</ul>
			</li>
		</ul>
	- Dubins path planning [:books:](lectures/amr_dubins_path_planning.pdf) 
- Section 2 :alien:	
    - Bayesian Filter [:books: ](lectures/amr_bayesian_filter.pdf)  
	 	<ul>
	      <li>Basic of Probability</li>
	      <li>Probabilistic Generative Laws</li>
	      <li>Estimation from Measurements</li>
	      <li>Estimation from Measurements and Controls</li>
	    </ul>

	- Particle Filter [:books: ](lectures/amr_particle_filter.pdf) 
		<ul>
			<li> A Taxonomy of Particle Filter </li>
			<li> Bayesian Filter </li>
			<li> Monte Carlo Integration (MCI) </li>
			<li> Particle Filter </li>
			<li> Importance Sampling </li>
			<li> Particle Filter Algorithm </li>
		</ul>

	- Robot localization   [:books: ](lectures/amr_robot_localization.pdf) 
		<ul>
			<li> A Taxonomy of Localization Problems </li>
			<li> Markov localization  </li>
			<li> Environment Sensing </li>
			<li> Motion in the Environment </li>
			<li> Localization in the Environment </li>
			<li> EKF localization with known correspondence </li>
			<li> Particle filter localization with known correspondence </li>
		</ul>
- Section 3 :alien:
	- State Estimation with Kalman filter [:books:](lectures/amr_kalman_filter.pdf)
		<ul>
			<li> Gaussian Distribution</li>
			<li> One Dimensional Kalman Filter</li>
			<li> Multivariate Density Function</li>
			<li> Marginal Density Function</li>
			<li> Multivariate Normal Function</li>
			<li> Two Dimensional Gaussian</li>
			<li> Multiple Random Variable</li>
			<li> Multidimensional Kalman Filter</li>
			<li> Sensor Fusion</li>
			<li> Linearization, Taylor Series Expansion, Linear Systems</li>
			<li> Extended Kalman Filter (EKF)</li>
			<li> Comparison between KF and EKF</li>
		</ul>

- Section 4 :alien:
	- Perception [:books:](lectures/amr_vision.pdf)
		<ul>
			<li> Monocular Vision</li>
				<ul>
					<li>Pinhole Camera Model</li>
					<li>Image Plane, Camera Plane, Projection Matrix</li>
					<li>Projective transformation</li>
					<li>Finding Projection Matrix using Direct Linear Transform (DLT)</li>
					<li>Camera Calibration</li>
				</ul>
			<li> Stereo Vision </li>
				<ul>
					<li>Simple Stereo, General Stereo</li>
					<li>Some homogeneous properties</li>
					<li>Epipolar Geometry</li>
					<li>Essential matrix, Fundamental matrix</li>
					<li>Camera Calibration</li>
				</ul>
			<li> Depth Estimation </li>
		</ul>