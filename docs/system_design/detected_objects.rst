Detected objects
================

Detecting objects is critical to perception, and sharing those objects is critical to multiple object tracking. Without
the former, you couldn't see, and without the latter, your friends couldn't see. Detected objects move with different
motion models depending on their type. Think about how you move when walking versus how your car moves when driving. We
support the following object types, each with a different assumed motion model:

* vehicle
* person
* cyclist


API design
----------

We represent detected objects using the :code:`DetectedObject` struct, which contains the object's measured state, its
measured state covariance, and the time at which it was measured. Different detected objects have different motion
models (*i.e.*, state vectors and covariance matrices) so we implemented :code:`DetectedObject` as a templated struct.

.. code-block:: cpp

    template <typename StateType, typename CovarianceType>
    struct DetectedObject
    {
        units::time::second_t timestamp;
        StateType state;
        CovarianceType covariance;
    };

With a templated approach, we can easily define new specialized :code:`DetectedObject` types by specifying the state
vector and covariance matrix types.

.. code-block:: cpp

    using Vehicle = DetectedObject<CtrvState, CtrvCovariance>;
    using Person = DetectedObject<CvState, CvCovariance>;
    using Cyclist = DetectedObject<CtraState, CtraCovariance>;

We normally process :code:`DetectedObject`\s collectively, so we need some way to store them in a single collection.
However, each :code:`DetectedObject` specialization is a distinct type, meaning there is no common base type to which
we can cast them. Therefore, we will use :code:`std::variant` and create a type alias for the variant set containing
all the supported :code:`DetectedObject` specializations.

.. code-block:: cpp

    using DetectedObject = std::variant<Vehicle, Person, Cyclist>;

Now we can create an iterable type (again, using a type alias) that we can use to store all of our supported detected
object types. Let's call it :code:`DetectedObjectList`.

.. code-block:: cpp

    using DetectedObjectList = boost::static_vector<DetectedObject, 200>;

To help ensure stable run times, we artificially cap the number of :code:`DetectedObject`\s that we keep track of (in
this case, :code:`200`).
Since the C++ standard library does not have a variable-sized container type with a fixed capacity, we use one provided
by the `Boost.Container library <https://www.boost.org/doc/libs/1_80_0/doc/html/container.html>`_ called
|boost::static_vector|_.

.. |boost::static_vector| replace:: ``boost::static_vector``
.. _boost::static_vector: https://www.boost.org/doc/libs/1_80_0/doc/html/boost/container/static_vector.html
