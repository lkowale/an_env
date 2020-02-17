import pandas as pd

glob_contours_list = ['123451', '678906']


class BaseObject(object):

    clade_parameters = []

    def __init__(self,):
        self.occurrences = pd.DataFrame()

    @classmethod
    def describe_contour(cls, contour):
        return []

    @classmethod
    def get_parameters(cls):
        return []

    @classmethod
    def contours_to_dataframe(cls, contours_list):
        values = [cls.describe_contour(contour) for contour in contours_list]
        df = pd.DataFrame(values, columns=cls.get_parameters())
        return df

    @classmethod
    def contours_to_list(cls, contours_list):
        values = [cls.describe_contour(contour) for contour in contours_list]
        return values


class Blob(BaseObject):

    clade_parameters = ['c_x', 'c_y', 'radius']

    def __init__(self):
        super(Blob, self).__init__()

    @classmethod
    def describe_contour(cls, contour):
        parent_value = super(Blob, cls).describe_contour(contour)
        value = parent_value + [contour[0], contour[1], contour[2]]
        return value

    @classmethod
    def get_parameters(cls):
        parent_value = super(Blob, cls).get_parameters()
        value = parent_value + ['c_x', 'c_y', 'radius']
        return value


class RotatedRectangle(Blob):

    clade_parameters = ['width', 'height', 'angle']

    def __init__(self):
        super(RotatedRectangle, self).__init__()

    @classmethod
    def describe_contour(cls, contour):
        parent_value = super(RotatedRectangle, cls).describe_contour(contour)
        value = parent_value + [contour[3], contour[4], contour[5]]
        return value

    @classmethod
    def get_parameters(cls):
        parent_value = super(RotatedRectangle, cls).get_parameters()
        value = parent_value + ['width', 'height', 'angle']
        return value


square = RotatedRectangle()
sq_df = square.__class__.contours_to_dataframe(glob_contours_list)
print(sq_df)
