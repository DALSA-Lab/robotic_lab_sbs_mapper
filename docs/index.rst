Robitic SBS Plate Mapper
========================

Welcome to the documentation for the Robotic SBS Plate Mapper; a toolbox for enabling localisation of SBS/SLAS microplates in a modular environment.
Developed at **DTU's Arena for Life Science Automation (DALSA)** [1]_, the project aim to bridge the gap between hardware, software and lab integration.


Background
----------
Originally developed as a direct extension of the modular DALSA table, the toolbox aim to help define the spatial layout of a dynamic workspace.
For a modular laboratry it is insufficient to rely on hardcoded microplate locations.
The localisation is based on ArUco markers due to their wide adoption and low computational complexity, yet still providing great accuracy.



Overview
--------
The software is written in **Python**, built on the **Open Source Computer Vision Library (OpenCV)** [2]_, to provide a toolbox with the required modules to perform camera calibration, hand-eye calibration and ArUco marker detection. A collection of C++ tools are provided as supplementary tools.
Here, you'll find an overview of the submodules and their source-code documentation, supplementary tools, working examples with demo vidoes and notes on integration.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   architecture/index.rst
   usage/examples/index.rst
   integration/index.rst
   code/index.rst


.. [1] Official DALSA website: https://dalsa.dtu.dk
.. [2] OpenCV Org website: https://opencv.org