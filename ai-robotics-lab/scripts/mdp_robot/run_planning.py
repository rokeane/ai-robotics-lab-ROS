#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec  6 12:27:56 2019

@author: cchanel
"""
import mdptoolbox
import run_planning_model_generation as model

## Solving MDP problem
# instanciate solver class with model and solving parameters
vi = mdptoolbox.mdp.ValueIteration(model.transFunction, model.rewFunction, 0.95, 0.01, 100)
# run solver
vi.run()
# print policy vector
print(vi.policy)

# simulating policiy

