

class Number:

    def __init__(self, value):

        self.value = value




class NaturalNumber(Number):

    def __init__(self, value):
        super(self.__class__, self).__init__(value)
        self.__class__.__bases__


class IntegerNumber(NaturalNumber):


a = Number()
b = NaturalNumber("MY_Natural")
c =