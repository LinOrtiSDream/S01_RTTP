class Chained:
    def __init__(self, obj):
        self.obj = obj

    def __repr__(self):
        return repr(self.obj)

    def __str__(self):
        return str(self.obj)

    def __setitem__(self, key, value):
        self.obj[key] = value

    def __getattr__(self, name):
        attr = getattr(self.obj, name)
        if callable(attr):
            def selfie(*args, **kw):
                # Call the method just for side-effects, return self.
                _ = attr(*args, **kw)
                return self
            return selfie
        else:
            return attr