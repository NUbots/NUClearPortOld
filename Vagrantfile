# -*- mode: ruby -*-
# vi: set ft=ruby :

require 'socket'

Vagrant.configure("2") do |config|

  # Determine the base box to use by checking the hostname.
  # Add your hostname to the list to opt-out of nubots-14.04.
  if ['nubots-b767f2s', 'Ne', 'Jake', 'reasearchlaptop'].include?(Socket.gethostname)
    # # 'precise32' was used prior to 2014-04-30.
    # config.vm.box = "precise32"
    # config.vm.box_url = "http://files.vagrantup.com/precise32.box"

    # The NUbots changed to 'trusty32' on 2014-04-30.
    config.vm.box = "trusty32"
    config.vm.box_url = "https://cloud-images.ubuntu.com/vagrant/trusty/current/trusty-server-cloudimg-i386-vagrant-disk1.box"
  else
    # The nubots-14.04 box is generated by Packer.
    # Run ./b create_box virtualbox to create it.
    config.vm.box = "nubots-14.04"
  end

  # Create a public network, which generally matched to bridged network.
  # Bridged networks make the machine appear as another physical device on
  # your network.
  # config.vm.network :public_network

  # Enable provisioning with Puppet stand alone.  Puppet manifests
  # are contained in a directory path relative to this Vagrantfile.
  config.vm.provision :puppet do |puppet|
    puppet.manifests_path = "puppet/manifests"
    puppet.module_path = "puppet/modules"
    puppet.manifest_file  = "site.pp"
  end

  # config.vm.provider "virtualbox" do |v|
  #   v.gui = true
  # end

  # Define the NUClearPort development VM, snd make it the primary VM
  # (meaning that a plain `vagrant up` will only create this machine)
  config.vm.define "nuclearportvm", primary: true do |nuclearport|
    nuclearport.vm.hostname = "nuclearportvm.nubots.net"

    nuclearport.vm.network :private_network, ip: "192.168.33.77"

    nuclearport.vm.network :forwarded_port, guest: 12000, host: 12000
    nuclearport.vm.network :forwarded_port, guest: 12001, host: 12001

    # Add hostname here if running NUbugger on the VM
    if ['Ne', 'jordan-XPS13', 'taylor-ubuntu'].include?(Socket.gethostname) # NUbugger Port

      nuclearport.vm.network :forwarded_port, guest: 9090, host: 9090
    end

    if ['s24'].include?(Socket.gethostname) # NUbugger Port
      nuclearport.vm.network :public_network, bridge: "WiFi"
    end

    if ['taylor-ubuntu'].include?(Socket.gethostname) # NUbugger Port
      nuclearport.vm.network :public_network, bridge: "wlan0"
    end

    # Syntax: "path/on/host", "/path/on/guest"
    # nuclearport.vm.synced_folder ".", "/home/vagrant/nubots/NUClearPort"

    # Note: Use NFS for more predictable shared folder support.
    #   The guest must have 'apt-get install nfs-common'
    nuclearport.vm.synced_folder ".", "/home/vagrant/nubots/NUClearPort"

    # Share NUbugger repository with the VM if it has been placed in the same
    # directory as the NUClearPort repository
    if File.directory?("../NUbugger")
      nuclearport.vm.synced_folder "../NUbugger", "/home/vagrant/nubots/NUbugger"
    end
  end
end
