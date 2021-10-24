import ipaddress
import os
import re
import yaml

from paramiko import SSHClient, AutoAddPolicy

configFilename = "deploy_config.yaml"

def try_read_config_file():
    if os.path.isfile(configFilename):
        with open(configFilename, "r") as stream:
            try:
                return yaml.safe_load(stream) 
            except:
                return None

def write_config_file(config):
    with open(configFilename, 'w', encoding='utf8') as outfile:
        yaml.dump(config, outfile, default_flow_style=False, allow_unicode=True)

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

def try_retrieve_destination(destination, machineName):
    config = try_read_config_file()

    if destination is not None:
        try:
            return try_parse_destination_string(destination)
        except Exception as e:
            raise e

    elif config is not None:
        try:
            return get_destination_from_config(config, machineName)
        except Exception as e:
            raise e

def open_ssh(address, port, username, autoCopyKey = False):
    ssh = SSHClient()
    ssh.load_system_host_keys()
    ssh.set_missing_host_key_policy(AutoAddPolicy())
    
    try:
        ssh.connect(address,  port=port, username=username, key_filename=f'{os.path.expanduser("~")}/.ssh/id_rsa')
        return ssh
    except:
        if not autoCopyKey: return None
        keyDir = f'{os.path.expanduser("~")}/.ssh'
        
        if not os.path.exists(f"{keyDir}/id_rsa"): os.system(f"ssh-keygen -m PEM -N '' -f {keyDir}/id_rsa")

        os.system(f"cat {keyDir}/id_rsa.pub | ssh -p {port} {username}@{address} 'cat >> .ssh/authorized_keys'")
        try:
            ssh.connect(address,  port=port, username=username, key_filename=f'{os.path.expanduser("~")}/.ssh/id_rsa')
            return ssh
        except:
            return None

def close_ssh(ssh):
    ssh.close()

def exec_command_and_print_output(ssh, command):
    ssh_stdout = ssh.exec_command(command, get_pty=True)[1]
    for line in iter(ssh_stdout.readline, ""):
        print(line, end="")