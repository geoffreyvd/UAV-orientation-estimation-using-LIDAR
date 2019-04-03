#!/usr/bin/env python3
from inspect import getargspec

class AutoInit(type):
  def __new__(meta, classname, supers, classdict):
    classdict['__init__'] = autoInitDecorator(classdict['__init__'])
    return type.__new__(meta, classname, supers, classdict)

def autoInitDecorator (toDecoreFun):
  def wrapper(*args):
    
    # ['self', 'first_name', 'last_name', 'birth_date', 'sex', 'address']
    argsnames = getargspec(toDecoreFun)[0]
    
    # the values provided when a new instance is created minus the 'self' reference
    # ['Jonh', 'Doe', '21/06/1990', 'male', '216 Caledonia Street']
    argsvalues = [x for x in args[1:]]
    
    # 'self' -> the reference to the instance
    objref = args[0]
    
    # setting the attribute with the corrisponding values to the instance
    # note I am skipping the 'self' reference
    for x in argsnames[1:]:
     	objref.__setattr__(x,argsvalues.pop(0))
    
  return wrapper

class extractedLine(metaclass=AutoInit):
    '''
    line notation class
    '''
                #(x1,y1,x2,y2,index1,index2,amountOfDataPoints, perpendicularRadian)

    def __init__(self, x1, y1, x2, y2, x1Raw, y1Raw, x2Raw, y2Raw, index1, index2, amountOfDataPoints, perpendicularRadian):
        pass
