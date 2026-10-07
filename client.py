import socket
import time

ROBOT_HOST = "robot.local"  # Resolves via mDNS
ROBOT_PORT = 5000

def main():
    print(f"Connecting to {ROBOT_HOST}:{ROBOT_PORT}...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(15.0)

    try:
        sock.connect((ROBOT_HOST, ROBOT_PORT))
        print(f"[+] Successfully connected to {ROBOT_HOST}!")
        print("Type commands ('sleep', 'X=0.1'). Type 'exit' to quit.\n")

        sock.settimeout(None) # Set back to blocking for interactive prompt

        while True:
            cmd = input("Command > ").strip()
            if not cmd:
                continue
            if cmd.lower() == 'exit':
                break

            # Send command to robot server
            sock.sendall((cmd + "\n").encode('utf-8'))

            # Wait for ACK response from robot
            response = sock.recv(1024).decode('utf-8')
            print(f"Robot Response: {response.strip()}")

    except socket.gaierror:
        print(f"[-] Could not resolve hostname '{ROBOT_HOST}'. Make sure mDNS/Bonjour is enabled on your PC and the robot is powered on.")
    except socket.error as e:
        print(f"[-] Connection failed: {e}")
    finally:
        sock.close()
        print("Connection closed.")

if __name__ == "__main__":
    main()