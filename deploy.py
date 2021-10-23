#!/usr/bin/env python3

import os
import argparse
import subprocess
import yaml
import getpass
import remote

from paramiko import SSHClient, AutoAddPolicy
import re

foldername = os.path.split(os.getcwd())[1]
localUsername = getpass.getuser()

#Functions
def add_destination_to_config(config, username, address, port, machineName):
    machineConfig = {'username':username, 'address':address, 'port':port}

    if config is None:
        config = {'machines':{machineName:machineConfig}, 'default_machine':machineName}
    else:
        config['machines'][machineName] = machineConfig

    return config

def copy_project(ssh, remoteDir):
    ssh.exec_command('mkdir -p ' + remoteDir)
    subprocess.run(["rsync", "--delete", "--times", "-r", "--progress", "-e", f"ssh -p {port}", 'src', 'ros_distro.sh', 'install_ros.sh', 'build.sh', f"{username}@{address}:{remoteDir}", "--exclude", "ssam_simulation"])

def build_project(ssh): remote.exec_command_and_print_output(ssh, 'cd ' + f"{foldername}/{localUsername}" + ' && ./build.sh -j1')

def install_ros(ssh): remote.exec_command_and_print_output(ssh, 'cd ' + f"{foldername}/{localUsername}" + ' && ./install_ros.sh ros-base')





#Parse arguments
parser = argparse.ArgumentParser(description='Deploy and build code to remote hardware')

destinationGroup = parser.add_mutually_exclusive_group()
destinationGroup.add_argument('-d',
                    '--destination',
                    metavar='USER@ADDRESS[:PORT]',
                    action='store',
                    help='address and user to connect to')

destinationGroup.add_argument('-m',
                    '--machine',
                    action='store',
                    help='saved machine to connecto to')

extraFunctionGroup = parser.add_mutually_exclusive_group()
extraFunctionGroup.add_argument('--install',
                    action='store_true',
                    help='installs ROS on the remote hardware (also deploys the project)')

extraFunctionGroup.add_argument('--save',
                    action='store',
                    metavar='MACHINE NAME',
                    help='save remote machine config')

args = parser.parse_args()

username = None
address = None
port = None

configFileContents = remote.try_read_config_file()

#Get username, address and port
try:
    username, address, port = remote.try_retrieve_destination(args.destination, args.machine)
except Exception as e:
    print(e)
    exit()

#Save and exit early
machineNameToSave = args.save
if machineNameToSave is not None:
    configFileContents = add_destination_to_config(configFileContents, username, address, port, machineNameToSave)
    remote.write_config_file(configFileContents)

    print(f"Saved {username}@{address}:{port} as {machineNameToSave}")
    exit()

print(f'Connecting to {username}@{address}:{port}...')
ssh = remote.open_ssh(address, port, username, True)
if ssh is None:
    print(f"Could not connect to {username}@{address}:{port}.")
    exit()

print(f'Copying project contents...')
copy_project(ssh, f'~/{foldername}/{localUsername}')

if args.install: install_ros(ssh)

print(f'Building project...')
build_project(ssh)

remote.close_ssh(ssh)