Installation
============

Webots
------
Download the Webots R2023a from this `link <https://cyberbotics.com/>`_ according to the operation system of your laptop.

- For Ubuntu 20.04 & 22.04, you will download the `webots_2023a_amd64.deb <https://github.com/cyberbotics/webots/releases/download/R2023a/webots_2023a_amd64.deb>`_. To install the deb file, run the following command in your terminal:

.. code-block:: console

	$ sudo dpkg -i install webots_2023a_amd64.deb

- For Windows 10, you will download and install the `webots-R2023a_setup.exe <https://github.com/cyberbotics/webots/releases/download/R2023a/webots-R2023a_setup.exe>`_.
- For macOS, you will download and install the `webots-R2023a.dmg <https://github.com/cyberbotics/webots/releases/download/R2023a/webots-R2023a.dmg>`_.

Conda Environment
-----------------
Conda environment helps you manage the pip library installation. Conda is also popular for deep learning and reinforement learning. Follow the steps on `Conda website <https://conda.io/projects/conda/en/latest/user-guide/install/index.html>`_ to install it.

The project code
----------------
First, you need to clone this repository.

.. code-block:: console

	$ git clone https://github.com/dronecourse-epfl/crazy-practical-2023.git

Then, you can create a new conda environment with command:

.. code-block:: console

	$ conda create -n drone python=3.8
	$ conda activate drone

And go to the crazy-practical-2023 directory to install the dependencies with command:

.. code-block:: console

	$ pip install -r requirements.txt

Finally, you can run the drone simulation with command:

.. code-block:: console

	$ webots worlds/crazyflie_world_epfl_lis.wbt

Here you will see the webots software as follows without any errors in the console pane.

.. image:: webots_start.png
  :width: 650
  :alt: webots start