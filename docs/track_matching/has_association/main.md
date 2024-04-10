# `multiple_object_tracking::HasAssociation`

Defined in `<multiple_object_tracking/track_matching.hpp>`

```cpp
class HasAssociation;
```

This class is a function object that can be used as a unary predicate to check
if a specified object (track or detection) has an association within a given
association map.

## Member functions

|                                       |                                     |
| ------------------------------------- | ----------------------------------- |
| [(constructor)][constructor_doc]  | constructs the `HasAssociation`     |
| (destructor) _(implicitly declared)_  | destructs the `HasAssociation`      |
| [`operator()`][operator_call_doc]     | checks if object has an association |

[constructor_doc]: constructor.md
[operator_call_doc]: operator_call.md
