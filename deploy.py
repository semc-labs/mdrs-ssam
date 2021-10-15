#!/usr/bin/env python3

import os
import argparse
import subprocess
import yaml
import getpass

from paramiko import SSHClient, AutoAddPolicy
import re

foldername = os.path.split(os.getcwd())[1]
configFilename = "deploy_config.yaml"
localUsername = getpass.getuser()

#Functions
def try_read_config_file():
    if os.path.isfile(configFilename):
        with open(configFilename, "r") as stream:
            try:
                return yaml.safe_load(stream) 
            except:
                return None

def get_destination_from_config(config, machineName):
    if machineName is None and 'default_machine' in config:
        machineName = config['default_machine']

    machine = config['machines'].get(machineName)
    if machine is not None:
        username = machine['username']
        address = machine['address']
        port = machine['port']

        return username, address, port
    else:
        raise Exception(f'Machine {machineName} was not found')

def add_destination_to_config(config, username, address, port, machineName):
    machineConfig = {'username':username, 'address':address, 'port':port}

    if config is None:
        config = {'machines':{machineName:machineConfig}, 'default_machine':machineName}
    else:
        config['machines'][machineName] = machineConfig

    return config

def write_config_file(config):
    
    with open(configFilename, 'w', encoding='utf8') as outfile:
        yaml.dump(config, outfile, default_flow_style=False, allow_unicode=True)
        
def try_parse_destination_string(destination):
    regexResult = re.findall(r'(.+)@([a-zA-Z0-9.]+)\:?([0-9]+)?', destination)
    if len(regexResult) != 1:
        raise Exception(f"Invalid format for destination argument. Expected user@address[:port], got {destination}")
    
    destinationElements = regexResult[0]
    username = destinationElements[0]
    address = destinationElements[1]
    try:
        port = int(destinationElements[2])
    except:
        port = 22

    return username, address, port

def open_ssh(address, port, username):
    ssh = SSHClient()
    ssh.load_system_host_keys()
    ssh.set_missing_host_key_policy(AutoAddPolicy())
    
    try:
        ssh.connect(address,  port=port, username=username, key_filename=f'{os.path.expanduser("~")}/.ssh/id_rsa')
    except:
        return None

    return ssh

def close_ssh(ssh):
    ssh.close()

def copy_project(ssh, remoteDir):
    ssh.exec_command('mkdir -p ' + remoteDir)
    subprocess.run(["rsync", "--delete", "--times", "-r", "--progress", "-e", f"ssh -p {port}", 'src', 'ros_distro.sh', 'install_ros.sh', 'build.sh', f"{username}@{address}:{remoteDir}", "--exclude", "ssam_simulation"])

def exec_command_and_print_output(ssh, command):
    ssh_stdout = ssh.exec_command(command, get_pty=True)[1]
    for line in iter(ssh_stdout.readline, ""):
        print(line, end="")

def build_project(ssh): exec_command_and_print_output(ssh, 'cd ' + f"{foldername}/{localUsername}" + ' && ./build.sh -j1')

def install_ros(ssh): exec_command_and_print_output(ssh, 'cd ' + f"{foldername}/{localUsername}" + ' && ./install_ros.sh ros-base')





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

configFileContents = try_read_config_file()

#Get username, address and port
if args.destination is not None:
    try:
        username, address, port = try_parse_destination_string(args.destination)
    except Exception as e:
        print(e)
        exit()

elif configFileContents is not None:
    try:
        username, address, port = get_destination_from_config(configFileContents, args.machine)
    except Exception as e:
        if args.machine is not None:
            print(e)
            exit()

#Save and exit early
machineNameToSave = args.save
if machineNameToSave is not None:
    configFileContents = add_destination_to_config(configFileContents, username, address, port, machineNameToSave)
    write_config_file(configFileContents)

    print(f"Saved {username}@{address}:{port} as {machineNameToSave}")
    exit()

print(f'Connecting to {username}@{address}:{port}...')
ssh = open_ssh(address, port, username)

#Key authentication failed - generate key and try again
if ssh is None:
    keyDir = f'{os.path.expanduser("~")}/.ssh'
    
    if not os.path.exists(f"{keyDir}/id_rsa"): os.system(f"ssh-keygen -m PEM -N '' -f {keyDir}/id_rsa")

    os.system(f"cat {keyDir}/id_rsa.pub | ssh -p {port} {args.destination} 'cat >> .ssh/authorized_keys'")

ssh = open_ssh(address, port, username)
if ssh is None:
    print(f"Could not connect to {username}@{address}:{port}.")
    exit()

print(f'Copying project contents...')
copy_project(ssh, f'~/{foldername}/{localUsername}')

if args.install: install_ros(ssh)

print(f'Building project...')
build_project(ssh)

close_ssh(ssh)