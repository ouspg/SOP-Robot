update-overview-diagram:
	python scripts/overview.py

vup:
	vagrant up --provider=hyperv

vsh:
	vagrant ssh

build-all:
	colcon build

vagrant-reprov:
	vagrant reload --provision
