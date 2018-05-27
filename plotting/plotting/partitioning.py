from transducer.infrastructure import Transducer


class Partitioning(Transducer):
    def __init__(self, reducer, predicate):
        super().__init__(reducer)
        self._predicate = predicate
        self._stored = []

    def step(self, result, item):
        if self._predicate(item):  # Encountered delimiter
            # yield stored sequence
            if self._stored:
                rv = self._reducer(result, self._stored)
            else:
                rv = result
            # start new sequence
            self._stored = [item]
            return rv
        else:  # Encountered non-delimiter item
            # append to existing sequence
            self._stored.append(item)
            return result

    def complete(self, result):
        if self._stored:
            result = self._reducer.step(result, self._stored)
        return self._reducer.complete(result)


def partitioning(predicate):
    """If predicate returns true, a new sequence (list) will be started with that item"""

    def partitioning_transducer(reducer):
        return Partitioning(reducer, predicate)

    return partitioning_transducer
