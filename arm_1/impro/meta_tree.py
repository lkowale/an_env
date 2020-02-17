class ModuleMetaClass(type):
    """ Metaclass that is instantiated once for each class """
    _metaclass_instances = None

    def __init__(cls, name, bases, attrs):

        if cls._metaclass_instances is None:
            # First instance is ModuleBaseClass
            cls._parent_class = None
            cls._metaclass_instances = [type(None)]
        else:
            # parent class is the previously declared class
            cls._parent_class = cls._metaclass_instances[-1]

            # if not at the top of the tree, then we are our parent's child
            if cls._parent_class != type(None):
                cls._parent_class._child_class = cls

            # store this class in the list of classes
            cls._metaclass_instances.append(cls)

        # no child class yet
        cls._child_class = None

        # call our base (meta) class init
        super(cls, cls).__init__(name, bases, attrs)


class ModuleBaseClass(object):
    __metaclass__ = ModuleMetaClass
    """ Base class for each of the derived classes in our tree """

    def __init__(self, name, parent):
        assert isinstance(parent, self._parent_class)
        self.name = name
        self._parent = parent

        if self._child_class is not None:
            self._children = {}

            # child class variable plural is used to add a common name
            plural = getattr(self._child_class, '_plural')
            if plural is not None:
                setattr(self, plural, self._children)

        # add self to our parents collection
        if parent is not None:
            parent._add_child(self)

        # add an access attribute for each of the nodes above us in the tree
        while parent is not None:
            setattr(self, type(parent).__name__.lower(), parent)
            parent = parent._parent

    def _add_child(self, child):
        assert isinstance(child, self._child_class)
        assert child.name not in self._children
        self._children[child.name] = child

# --------------------------------

class Module(ModuleBaseClass):
    def __init__(self, name, lang):
        super(self.__class__, self).__init__(name, None)
        assert lang in ['fr', 'en']
        self.lang = lang

class Submodule(ModuleBaseClass):
    _plural = 'submodules'

class Ability(ModuleBaseClass):
    _plural = 'abilities'

class Template(ModuleBaseClass):
    _plural = 'templates'

    def __init__(self, name, ability):
        super(self.__class__, self).__init__(name, ability)
        self.lang = module.lang

# --------------------------------

module = Module('module1', 'fr')
sub_module1 = Submodule('sub_module1', module)
sub_module2 = Submodule('sub_module2', module)
ability = Ability('ability1', sub_module1)
template = Template('template1', ability)
print(template.lang)