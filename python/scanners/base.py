'''
Abstract base class defining the Scanner interface
'''
from abc import ABC, abstractmethod

class Scanner(ABC):
    '''
    Abstract scanner interface for confocal microscope operation.
    
    Subclasses implement the run() method to perform their scan and return data.
    '''
    @abstractmethod
    def run(self):
        '''
        Execute the scan. Returns data specific to the scan type.
        '''
        pass