# -*- coding: utf-8 -*-
# this file config some fascinating algorithm which is still under progressing and testing
# coming soon but not ready for the moment

import os
import platform


def DoNothing(**kwargs):
    print('DO NOTHING', [d[0] for d in kwargs.items()])
    return []


abs_path = os.path.dirname(__file__)
fasci_file = os.path.join(abs_path, 'fascinating_algo.py')

if os.path.exists(fasci_file):
    from fascinating_algo import *


class FascinatingConfig:
    @property
    def SingleCamOnCall(self):
        if os.path.exists(fasci_file):
            return SingleCamFascinate
        else:
            return DoNothing
    
    @property
    def ConditionedReflex(self):
        if os.path.exists(fasci_file):
            return Euler2Motion
        else:
            return DoNothing


fasci_config = FascinatingConfig()