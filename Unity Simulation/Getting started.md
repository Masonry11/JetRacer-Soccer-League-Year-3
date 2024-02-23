Unity ML-Agents Trainers
The mlagents Python package is part of the ML-Agents Toolkit. mlagents provides a set of reinforcement and imitation learning algorithms designed to be used with Unity environments. The algorithms interface with the Python API provided by the mlagents_envs package. See here for more information on mlagents_envs.

The algorithms can be accessed using the: mlagents-learn access point. See here for more information on using this package.

Installation
Install the mlagents package with:

python -m pip install mlagents==1.0.0
Usage & More Information
For more information on the ML-Agents Toolkit and how to instrument a Unity scene with the ML-Agents SDK, check out the main ML-Agents Toolkit documentation.

Limitations
Resuming self-play from a checkpoint resets the reported ELO to the default value.
