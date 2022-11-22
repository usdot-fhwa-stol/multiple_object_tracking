Unscented transform
===================

When predicting motion, the most significant errors come from uncertainty. We don't know exactly how an object will
move, so we predict its movements using a model. While it won't predict the object's new state exactly, the prediction
will be a sufficient approximation for our application. Another source of uncertainty comes from the sensor's
measurements themselves. While we think of the sensors reporting object states, they actually report a probability
distribution over possible states. The "state" in this case is really the distribution's expected value, and the
associated covariance matrix represents the likelihood of the true state being something else.

Instead of predicting a single future state, we want to predict the future probability distribution over states. To
make things more complicated, the object's motion model most likely nonlinear. We can't find the new distribution using
analytical methods. Instead, our library uses a numerical approximation method called the
`unscented transform (UT) <https://en.wikipedia.org/wiki/Unscented_transform>`_.

We summarize the UT process in a few steps:

1. Generate sigma points.
2. Propagate each sigma point to a specified time using the motion model.
3. Approximate the future mean object state using the sigma points.
4. Approximate the future object state covariance using the sigma points.

Generating sigma points
-----------------------

Sigma points represents a sampling over the object state's distribution. We could sample the distribution (*i.e.*,
generate sigma points) using one of many methods. We need to generate at least :math:`n + 1` points, where :math:`n` is
the number of state variables. Generating :math:`2n + 1` point is common.

Our library generates :math:`2n + 1` points using the following formula (alternative formulas exist too):

.. math::

    \begin{align}
        s_{0} &= \boldsymbol{x}_{k} \\
        s_{i} &= \boldsymbol{x}_{k} + \sqrt{n \boldsymbol{P}_{k, [i]}} \quad \text{for } i = 1, \dots, n \\
        s_{i} &= \boldsymbol{x}_{k} - \sqrt{n \boldsymbol{P}_{k, [i - n]}} \quad \text{for } i = n, \dots, 2n + 1 \\
    \end{align}

Notation :math:`\boldsymbol{x}_{k}` and :math:`\boldsymbol{P}_{k}` represents the object's state vector
:math:`\boldsymbol{x}` and covariance matrix :math:`\boldsymbol{P}`, respectively, at time step :math:`k`. Notation
:math:`[i]` represents :math:`\boldsymbol{P}`\'s :math:`i^\text{th}` column. We collect each sigma point into a set
:math:`\mathcal{S} \triangleq \{s_{0}, s_{1}, \dots, s_{2n + 1}\}`.


Propagating sigma points
------------------------

Propagating sigma points is identical to propagating a state vector because really they are state vectors. We create a
set of propagated sigma points using the specified motion model :math:`f` and a time step :math:`\Delta t`

.. math::

    \mathcal{S}^+ \triangleq \{ f(s_{i}, \Delta t) \mid i = 0, \dots, 2n + 1\} = \{s^+_0, s^+_1, \dots, s^+_{2n + 1}\}

The :math:`+` superscript notation indicates (the set of) predicted sigma points.


Approximating state and covariance
----------------------------------

The propagated sigma points are samples from the predicted state distribution, and we use them to approximate the
predicted mean state and covariance. We approximate the predicted mean state using the standard sample mean formula:

.. math::

    \hat{\boldsymbol{x}}_{k + 1} = \frac{1}{n} \sum^{2n + 1}_{i = 0} s^+_i

where :math:`\hat{\boldsymbol{x}}_{k + 1}` is the predicted object state vector for time step :math:`k + 1`.

Similarly, we calculate the predicted state covariance by

.. math::

    \hat{\boldsymbol{P}}_{k + 1} = \frac{1}{n} \sum^{2n + 1}_{i = 0} \left(s^+_i - \hat{\boldsymbol{x}}_{k + 1}\right)^2

API design
----------

The prediction functions perform the same tasks regardless of the motion model, but each motion model provides its own
state vector and covariance matrix types. There is no single state vector or covariance matrix base type, so we use
template functions in our library.

.. code-block:: cpp

    template <typename StateType, typename CovarianceType>
    auto generateSigmaPoints(
        const StateType& state,
        const CovarianceType& covariance,
        std::size_t number
    ) -> std::unordered_set<StateType>;

.. code-block:: cpp

    template <typename StateType>
    auto approximateMean(
        const std::unordered_set<StateType>& sigma_points
    ) -> StateType;

.. code-block:: cpp

    template <typename StateType, typename CovarianceType>
    auto approximateCovariance(
        const std::unordered_set<StateType>& sigma_points,
        const StateType& mean
    ) -> CovarianceType;
