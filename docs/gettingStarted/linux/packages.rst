Install from packages
=====================

apt
---

The apt repository contains packages for both x86-64 and arm64.

Install the GPG key:

.. code-block:: bash

	wget -qO - https://repo.myriadrf.org/lime-microsystems-public.gpg | gpg --dearmor | sudo tee /etc/apt/keyrings/lime-microsystems-public.gpg > /dev/null

Add source for your distribution:

.. code-block:: bash

	echo "deb [signed-by=/etc/apt/keyrings/lime-microsystems-public.gpg] https://repo.myriadrf.org/apt stable main" | sudo tee /etc/apt/sources.list.d/repo.myriadrf.org.list

Update apt sources and install limesuiteng:

.. code-block:: bash

	sudo apt-get update
	sudo apt-get install limesuiteng
