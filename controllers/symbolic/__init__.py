"""Symbolic controller (lecture: Symbolic control, Saoud & Belamfedel Alaoui).

Step 1  world.py, dynamics.py, abstraction.py : grid + robot -> finite transition system S = (Q, P, F)
Step 2  synthesis.py            : reach-avoid controller with the fixed-point algorithm (Pre)
Step 3  automaton.py            : "visit A then B, always avoid C" via product with an automaton (memory)
Step 4  controller.py           : plug into common/ (Controller interface, simulator, metrics)
"""
