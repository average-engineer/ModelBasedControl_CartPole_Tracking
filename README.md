# ModelBasedControl_CartPole_Tracking
Model based Control System Design for tracking/trajectory following of a cart with inverted pole system.


**Control Objective**: Both the pole and the cart have to follow a partiular desired trajectory, thus the desired state vector is non-zero. The code right now works only for the the case of both the cart and the pole being actuated, for the underactuated system (cart actuated but pole unactuated), the code still shows some erraneous results and needs to be worked upon. The faulty code can be accessed from the branch `SingleAct`.

**Controller Used**: PD Controller

**Control Strategy used**: Control Partitioning (Model Based Control).

**MODEL ARCHITECTURE**:

![sys](https://github.com/average-engineer/ModelBasedControl_CartPole_Tracking/blob/main/Results/MBC%20Arch.jpg)

*The major distinction here from the [regulation problem](https://github.com/average-engineer/ModelBasedControl_CartPole) is that the dynamics of the system can#t be linearlised about the unstable equilibrium point for the pole. Thus, Non-Linearity becomes an issue*

The model based approach has a massive advantage in this case since the actuation consists of the model parameters. Due to that, we can formally formulate a linear error (difference between the current states and the desired states) dynamics governing system of ODEs, which can be computed using state space representation for the error (`ErrorDynamics_SSR.m`).

Two cases are discussed in the repository:
* When the cart has to be regulated and the pole is to be tracked (*Case 1*)
* When the cart has to be tracked and the pole is to be regulated (*Case 1*)

Disturbance rejection is also studied where 3 kinds of distrubances to the cart are introduced:
* Harmonic disturbance force
* Impulse Disturbance force
* Static Disturbance Force

<h2>SOME RESULTS</h2>

* Case 1 (Undisturbed System):

![sys](https://github.com/average-engineer/ModelBasedControl_CartPole_Tracking/blob/main/Results/Case1%20(Regulated%20Cart%2C%20Tracked%20Pole)/PolePosn_NoDisturbance.png)

![sys](https://github.com/average-engineer/ModelBasedControl_CartPole_Tracking/blob/main/Results/Case1%20(Regulated%20Cart%2C%20Tracked%20Pole)/ActuatorEffort_NoDisturbance.png)

* Case 1 (Under Impulse Loading at 5 seconds):

![sys](https://github.com/average-engineer/ModelBasedControl_CartPole_Tracking/blob/main/Results/Case1%20(Regulated%20Cart%2C%20Tracked%20Pole)/CartPosn_ImpulseLoading.png)

![sys](https://github.com/average-engineer/ModelBasedControl_CartPole_Tracking/blob/main/Results/Case1%20(Regulated%20Cart%2C%20Tracked%20Pole)/PolePosn_ImpulseLoading.png)

* Case 2 (Under Harmonic Loading):

![sys](https://github.com/average-engineer/ModelBasedControl_CartPole_Tracking/blob/main/Results/Case2%20(Tracked%20Cart%2C%20Regulated%20Pole)/PolePosn_HarmonicLoading.png)





