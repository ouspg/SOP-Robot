# Based on https://github.com/mgruhler/vagrants
# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|
  # Every Vagrant virtual environment requires a box to build off of.
  config.vm.box = "generic/ubuntu2004"
  config.vm.hostname = "vagrant-ros"
  config.vm.define "vagrant-ros"
  config.vm.box_download_options = {"ssl-no-revoke" => true}

  config.vm.provider "virtualbox" do |vb|
    # Don't boot with headless mode
    vb.gui = true

    vb.customize ["modifyvm", :id, "--memory", "4096"]
    vb.customize ["modifyvm", :id, "--cpus", "4"]
    vb.customize ["modifyvm", :id, "--graphicscontroller", "vmsvga"]
  #  vb.customize ["modifyvm", :id, "--accelerate3d", "on"]
  #  vb.customize ["modifyvm", :id, "--ioapic", "on"]
  #  vb.customize ["modifyvm", :id, "--vram", "128"]
  #  vb.customize ["modifyvm", :id, "--hwvirtex", "on"]
  #  vb.customize ["modifyvm", :id, "--uartmode1", "file", File::NULL]
  end

  config.vm.provider "libvirt" do |vi|
    vi.graphics_type = "spice"
    vi.memory =  6144
    vi.cpus = 3
  end

  config.vm.provider "vmware_desktop" do |v|
    v.gui = true
  end

  # Sync folders
  config.vm.synced_folder ".", "/vagrant", type: "nfs", nfs_udp: false

  # config.vm.provision :shell, path: "vagrant-scripts/bootstrap.sh"
  config.vm.provision "ansible_local" do |ansible|
    ansible.playbook = "ansible-scripts/playbook.yml"
    # ansible.raw_arguments = "--tags current"
    ansible.verbose = true
  end
end
