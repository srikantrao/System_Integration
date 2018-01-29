
class LowPassFilter(object):
    def __init__(self, alpha, beta):
        """
        alpha and beta should add up to 1
        :param alpha: Exponential Weight 1
        :param beta: Exponential Weight 2
        """
        self.a = alpha
        self.b = beta

        self.last_val = 0.
        self.ready = False

    def get(self):
        return self.last_val

    def filt(self, val):
        if self.ready:
            val = self.a * val + self.b * self.last_val
        else:
            self.ready = True

        self.last_val = val
        return val
