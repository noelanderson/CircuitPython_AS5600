Introduction
============


.. image:: https://readthedocs.org/projects/circuitpython-as5600/badge/?version=latest
    :target: https://circuitpython-as5600.readthedocs.io/
    :alt: Documentation Status



.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord


.. image:: https://github.com/noelanderson/CircuitPython_AS5600/workflows/Build%20CI/badge.svg
    :target: https://github.com/noelanderson/CircuitPython_AS5600/actions
    :alt: Build Status


.. image:: https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json
    :target: https://github.com/astral-sh/ruff
    :alt: Code Style: Ruff

CircuitPython helper library for the AMS AS5600 12-bit on-axis magnetic rotary position sensor


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_
or individual libraries can be installed using
`circup <https://github.com/adafruit/circup>`_.

Installing from PyPI
=====================
.. note:: This library is not available on PyPI yet. Install documentation is included
   as a standard element.

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/circuitpython-as5600/>`_.
To install for current user:

.. code-block:: shell

    pip3 install circuitpython-as5600

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install circuitpython-as5600

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .venv
    source .env/bin/activate
    pip3 install circuitpython-as5600

Installing to a Connected CircuitPython Device with Circup
==========================================================

Make sure that you have ``circup`` installed in your Python environment.
Install it with the following command if necessary:

.. code-block:: shell

    pip3 install circup

With ``circup`` installed and your CircuitPython device connected use the
following command to install:

.. code-block:: shell

    circup install as5600

Or the following command to update an existing version:

.. code-block:: shell

    circup update

Usage Example
=============

.. code-block:: python

    import time
    import board
    import busio
    from as5600 import AS5600

    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = AS5600(i2c)

    while True:
        print("Angle: ", sensor.angle)
        time.sleep(1)

Documentation
=============

Class Diagram for library

.. figure:: https://raw.githubusercontent.com/noelanderson/CircuitPython_AS5600/master/uml/as5600.svg
   :alt: Class Diagram

API documentation for this library can be found on `Read the Docs <https://circuitpython-as5600.readthedocs.io/>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/noelanderson/CircuitPython_AS5600/blob/HEAD/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.
