# -*- coding: utf-8 -*-
"""
Robot Deployment & Synchronization Script
This script automatically syncs your modified files to the Jetson Nano (192.168.0.175)
and restarts the cleaner service.

It automatically detects your physical network adapter to bypass local VPN/TUN proxies (e.g. Clash).
"""

import os
import sys
import socket
import paramiko
from stat import S_ISDIR

# --- Configuration ---
ROBOT_IP = "192.168.0.175"
SSH_PORT = 22
USERNAME = "nano"
PASSWORD = "nano"
REMOTE_DIR = "/workspace/cleaner"
RESTART_SERVICE = True
SERVICE_NAME = "cleaner"

# List of files to check/sync (or set to None to sync all python & json files tracked by git)
FILES_TO_SYNC = [
    "main.py",
    "util.py",
    "GPSuse.py",
    "Ntrip2Uart3.py",
    "RTKDataManager.py",
    "ntrip_runtime.py",
    "rtk_correction.py",
    "service.py",
    "config.json"
]

def get_physical_ip():
    """Finds the local physical IP that can route to the gateway to bypass VPN TUN interface."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Connect to gateway (doesn't send actual packet)
        s.connect(("192.168.0.1", 80))
        ip = s.getsockname()[0]
        s.close()
        # If it returns the loopback or TUN IP, we fall back
        if ip.startswith("198.18.") or ip.startswith("127."):
            return None
        return ip
    except Exception:
        return None

def main():
    print("=" * 60)
    print("           ROBOT DEPLOYMENT & SYNC UTILITY            ")
    print("=" * 60)
    
    # 1. Detect physical IP to bypass VPN proxy
    local_ip = get_physical_ip()
    if local_ip:
        print(f"[*] Detected physical local IP: {local_ip} (Bypassing VPN proxy)")
    else:
        print("[!] Warning: Could not detect physical local IP. Using default routing.")
        local_ip = None

    # 2. Check if host is reachable
    print(f"[*] Connecting to {ROBOT_IP}:{SSH_PORT}...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        if local_ip:
            # Create a socket bound to physical IP to bypass TUN/Clash Meta proxy
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.bind((local_ip, 0))
            sock.settimeout(5)
            sock.connect((ROBOT_IP, SSH_PORT))
            ssh.connect(ROBOT_IP, port=SSH_PORT, username=USERNAME, password=PASSWORD, sock=sock, timeout=10)
        else:
            ssh.connect(ROBOT_IP, port=SSH_PORT, username=USERNAME, password=PASSWORD, timeout=10)
            
        print("[+] Successfully connected to the robot!")
    except socket.timeout:
        print(f"\n[-] Error: Connection to {ROBOT_IP} timed out.")
        print("    Please check:")
        print("    1. Is the robot powered on?")
        print(f"    2. Is your computer connected to the same Wi-Fi network (e.g. 'robot171')?")
        print("    3. Does the Wi-Fi router have 'AP Isolation' (无线隔离) enabled?")
        sys.exit(1)
    except Exception as e:
        print(f"\n[-] Failed to connect: {e}")
        print("    If you have a VPN/Clash running, try disabling it or double checking the IP.")
        sys.exit(1)

    # 3. Perform file synchronization
    try:
        sftp = ssh.open_sftp()
        print(f"[*] Verifying remote directory '{REMOTE_DIR}'...")
        try:
            sftp.chdir(REMOTE_DIR)
        except IOError:
            print(f"[*] Remote directory '{REMOTE_DIR}' does not exist. Creating it...")
            # Create directory recursively
            parts = REMOTE_DIR.split('/')
            path = ""
            for part in parts:
                if not part:
                    continue
                path += "/" + part
                try:
                    sftp.stat(path)
                except IOError:
                    sftp.mkdir(path)
            sftp.chdir(REMOTE_DIR)

        # Upload files
        print("[*] Uploading files...")
        for filename in FILES_TO_SYNC:
            if not os.path.exists(filename):
                print(f"    [!] Skipping {filename} (local file not found)")
                continue
            
            print(f"    -> Syncing {filename}...")
            sftp.put(filename, filename)
            
        sftp.close()
        print("[+] File synchronization completed successfully!")

    except Exception as e:
        print(f"[-] SFTP File sync failed: {e}")
        ssh.close()
        sys.exit(1)

    # 4. Restart remote service
    if RESTART_SERVICE:
        print(f"[*] Restarting '{SERVICE_NAME}' service on the robot...")
        try:
            # We run systemctl restart using sudo. To handle password prompt if required, we use sudo -S
            command = f"echo '{PASSWORD}' | sudo -S systemctl restart {SERVICE_NAME}"
            stdin, stdout, stderr = ssh.exec_command(command)
            
            # Read output
            out = stdout.read().decode('utf-8').strip()
            err = stderr.read().decode('utf-8').strip()
            
            # Check status of the service
            status_cmd = f"systemctl status {SERVICE_NAME} --no-pager"
            stdin_s, stdout_s, stderr_s = ssh.exec_command(status_cmd)
            status_out = stdout_s.read().decode('utf-8').strip()
            
            print("[+] Service restarted!")
            print("-" * 40)
            print("Service Status:")
            for line in status_out.split('\n')[:10]: # Print first 10 lines of status
                print(f"  {line}")
            print("-" * 40)
            
        except Exception as e:
            print(f"[-] Failed to restart service: {e}")

    ssh.close()
    print("[+] Deployment completed successfully.")
    print("=" * 60)

if __name__ == '__main__':
    main()
