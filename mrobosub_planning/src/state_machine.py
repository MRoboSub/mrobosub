class Outcome:
    def __hash__(self):
        return hash(str(self))

    def __str__(self):
        return self.__class__.__name__
    

def make_outcome(name, *fields, **default_fields):
    class SubOutcome(Outcome):
        def __init__(self, **kwargs):
            for f in fields:
                setattr(self, f, None)
            for df in default_fields:
                setattr(self, df, default_fields[df])

            for kw in kwargs:
                setattr(self, kw, kwargs[kw])

    SubOutcome.__name__ = name
    return SubOutcome


if __name__ == '__main__':
    Timeout = make_outcome('Timeout')
    NotThere = make_outcome('NotThere', error=15)
    ReachDepth = make_outcome('ReachDepth', depth=78)

    to = Timeout()
    nt = NotThere(error=12)
    rd = ReachDepth()

    print(to)
    print(nt.__dict__)
    print(nt)
    print(rd.__dict__)



