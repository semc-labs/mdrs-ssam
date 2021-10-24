#!/usr/bin/env python3

import argparse
import ipaddress
import os
import re
import remote
import rosgraph
import subprocess
import threading

rosPort = 11311

#Helper functions
def get_local_ip_for_address(address):
    iprout = subprocess.run(["ip", "route", "get", address], stdout=subprocess.PIPE)
    head = subprocess.run(["head", "-1"], input=iprout.stdout, capture_output=True)
    output = head.stdout.decode('ascii')
    
    regexResult = re.findall(r'.*src (\S+).*', output)
    if len(regexResult) == 0:
        return None

    return regexResult[0]

def get_address_type(address):
    try:
        return type(ipaddress.ip_address(address))
    except:
        hosts = open('/etc/hosts','r')
        for line in hosts:
            elements = line.split()

            if len(elements) < 2:
                continue

            if elements[1] == address:
                try:
                    return type(ipaddress.ip_address(elements[0]))
                except:
                    break
    return None

def setup_local_env(address):
    envVars = os.environ.copy()
    envVars["ROS_MASTER_URI"] = f"http://{address}:{rosPort}"
    envVars.pop("ROS_HOSTNAME", None)
    envVars["ROS_REMOTE"] = "on"

    if get_address_type(address) is ipaddress.IPv4Address:
        envVars["ROS_IP"] = get_local_ip_for_address(address)
    else:
        envVars["ROS_IPV6"] = "on"
 
    return envVars



#Threads
def local_terminal_thread(runEvent, address):
    while not rosgraph.is_master_online(f"http://{address}:{rosPort}"):
        if not runEvent.is_set():
            return
        continue

    process = subprocess.Popen(["gnome-terminal", "--disable-factory"], env=setup_local_env(address))

    while runEvent.is_set():
        continue

    process.kill()

def local_launch_thread(runEvent, address, package, launchfile, *launchArgs):
    while not rosgraph.is_master_online(f"http://{address}:{rosPort}"):
        if not runEvent.is_set():
            return
        continue

    processArgs = ["/home/ubuntu/share/mdrs-ssam/launch.sh", package, launchfile]
    if launchArgs is not None: processArgs.extend(list(launchArgs))

    process = subprocess.Popen(processArgs, env = setup_local_env(address), stdout=subprocess.DEVNULL)

    while runEvent.is_set():
        continue

    process.kill()

def ssh_thread(username, address, port, package, launchfile, *launchArgs):
    ssh = remote.open_ssh(address, port, username, True)
    if ssh is None:
        print(f"Failed to connect to {username}@{address}:{port}")
        exit()

    launchCommand = f"source mdrs-ssam/vlad/devel/setup.bash \n export ROS_MASTER_URI=http://{address}:{rosPort} \n export ROS_IP={address}"
                    
    if get_address_type(address) is ipaddress.IPv6Address:
        launchCommand += "\n export ROS_IPV6=on"

    launchCommand += f"\n roslaunch {package} {launchfile}"

    if launchArgs is not None:
        for arg in launchArgs:
            launchCommand += f" {arg}"

    remote.exec_command_and_print_output(ssh, launchCommand)         

    remote.close_ssh(ssh)



#Main
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Launch nodes on remote machine and local machine')
    parser.add_argument('package', help='package containing launch file')
    parser.add_argument('launchfile', nargs='+', help='launch file')
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

    parser.add_argument('-l',
                        '--locallaunch',
                        action='store',
                        nargs='+',
                        help='launch file to launch locally')
    
    parser.add_argument('-t',
                        '--terminal',
                        action='store_true',
                        help='launch an extra terminal with the environment setup to connect to the remote ros master')

    args = parser.parse_args()
    
    try:
        username, address, sshPort = remote.try_retrieve_destination(args.destination, args.machine)
    except Exception as e:
        print(e)
        exit()

    terminalThread = None
    localLaunchThead = None

    runEvent = threading.Event()
    runEvent.set()
    
    if args.terminal: terminalThread = threading.Thread(target=local_terminal_thread, args=(runEvent, address))
    if args.locallaunch: localLaunchThead = threading.Thread(target=local_launch_thread, args=(runEvent, address, *args.locallaunch))

    sshThread = threading.Thread(target=ssh_thread, args=(username, address, sshPort, args.package, *args.launchfile))
    sshThread.daemon = True

    sshThread.start()
    if terminalThread is not None: terminalThread.start()
    if localLaunchThead is not None: localLaunchThead.start()

    try:
        while True:
            continue
    except KeyboardInterrupt:
        runEvent.clear()

    if terminalThread is not None: terminalThread.join()
    if terminalThread is not None: localLaunchThead.join()