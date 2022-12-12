# Based on https://github.com/mgruhler/vagrants
# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|
  # Every Vagrant virtual environment requires a box to build off of.
  config.vm.box = "generic/ubuntu2004"
  config.vm.hostname = "vagrant-ros"
  config.vm.define "vagrant-ros"

  config.vm.provider "virtualbox" do |vb|
    # Don't boot with headless mode
    vb.gui = true

    vb.customize ["modifyvm", :id, "--memory", "4096"]
    vb.customize ["modifyvm", :id, "--cpus", "4"]
  #  vb.customize ["modifyvm", :id, "--graphicscontroller", "vboxvga"]
  #  vb.customize ["modifyvm", :id, "--accelerate3d", "on"]
  #  vb.customize ["modifyvm", :id, "--ioapic", "on"]
  #  vb.customize ["modifyvm", :id, "--vram", "128"]
  #  vb.customize ["modifyvm", :id, "--hwvirtex", "on"]
  #  vb.customize ["modifyvm", :id, "--uartmode1", "file", File::NULL]
  end

  config.vm.provider "vmware_desktop" do |v|
    v.gui = true
  end

  # Sync folders
  config.vm.synced_folder "./", "/workspace/"

  config.vm.provision :shell, path: "vagrant-scripts/bootstrap.sh"
end