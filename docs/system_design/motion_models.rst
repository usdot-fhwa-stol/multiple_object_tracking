Motion models
=============

Accurately tracking objects' motion is critical to perception tasks. Unfortunately, not all objects move the same way.
Cyclists are agile enough to weave in and out of traffic, while buses must move carefully to avoid overturning. Our
multiple object tracking library provides several motion models that collectively represent all types of agents you
might encounter on the roads.


Constant turn-rate and velocity (CTRV)
--------------------------------------

As its name implies, this model assumes the agent moves with a constant linear velocity (technically, speed) and
rotates at a constant rate. We represent the agent's state under this motion model with five-element state vector
:math:`\boldsymbol{z}`, defined as

.. math::

    \boldsymbol{z} \triangleq \begin{bmatrix}x & y & v & \psi & \dot{\psi}\end{bmatrix}

where :math:`x` and :math:`y` are the agent's position, `v` is its scalar linear velocity, :math:`\psi` is its yaw
angle, and :math:`\dot{\psi}` is its yaw rate (scalar angular velocity). The CTRV model is nonlinear, and the following
system of equations define its state transformation:

.. math::

    \begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{v} \\ \dot{\theta} \\ \ddot{\psi} \end{bmatrix} =
    \begin{bmatrix}
        v \cos(\psi) \\
        v \sin(\psi) \\
        0 \\
        \dot{\psi} \\
        0
    \end{bmatrix}

For a given time :math:`t_{k}`, we can analytically integrate the differential equations to determine the agent's next
state at time :math:`t_{k + 1}`. This equation is

.. math::

    \begin{bmatrix} x_{k + 1} \\ y_{k + 1} \\ v_{k + 1} \\ \psi_{k + 1} \\ \dot{\psi}_{k + 1} \end{bmatrix} =
    \begin{bmatrix} x_{k} \\ y_{k} \\ v_{k} \\ \psi_{k} \\ \dot{\psi}_{k} \end{bmatrix} +
    \begin{bmatrix}
        \frac{v_{k}}{\dot{\psi}_{k}} \left[\phantom{-} \sin(\psi_{k} + \dot{\psi}_{k} \Delta t) - \sin(\psi_{k})\right] \\
        \frac{v_{k}}{\dot{\psi}_{k}} \left[- \cos(\psi_{k} + \dot{\psi}_{k} \Delta t) + \cos(\psi_{k})\right] \\
        0 \\
        \dot{\psi}_{k} \Delta t \\
        0
    \end{bmatrix}

where :math:`\Delta t \triangleq t_{k + 1} - t_{k}`.

While the above equation suffices when the agent is rotating, it is undefined when the agent isn't rotating. In this
scenario, we must use an alternative equation:

.. math::

    \begin{bmatrix} x_{k + 1} \\ y_{k + 1} \\ v_{k + 1} \\ \psi_{k + 1} \\ \dot{\psi}_{k + 1} \end{bmatrix} =
    \begin{bmatrix} x_{k} \\ y_{k} \\ v_{k} \\ \psi_{k} \\ \dot{\psi}_{k} \end{bmatrix} +
    \begin{bmatrix}
        v_{k} \cos(\psi_{k}) \Delta t \\
        v_{k} \sin(\psi_{k}) \Delta t \\
        0 \\
        0 \\
        0
    \end{bmatrix}


API design
----------

All motion models follow the same design pattern. State vectors have the name :code:`<Abbreviation>State`, and
covariance matrices have the name :code:`<Abbreviation>Covariance`. Additionally, each motion model provides a
:code:`nextState(...)` function implementation. For example, the CTRV model comprises :code:`CtrvState` and
:code:`CtrvCovariance` structs and a :code:`nextState(...)` function.
