# creatureSim
Physics simulation of creatures interacting with environment

## Download Instructions
1. Download the repo
2. Run main.py

Requires numpy 1.17 or newer (honestly, older probably works too)

Displaying animations requires [VPython](https://vpython.org/) 7.5 or newer


## Concepts
- A *creature* is an object that can move.
- A *physState* is the physical location of the creature and all its limbs.
- A *pose* is the relative state of all the objects the creature can move. For example, standing up is a pose.
- A *walkCycle* is a repeating list of poses that the creature uses to move

## Major Tasks TODO
- learning algorithm for walkCycles
- speed up physics
- consistently measure execution times

## Known ~~Bugs~~ Surprise Features
- strange positioning before and after animations
- no friction between body and ground
- creatures do not have souls

---
Niraj did this
