import socket
import sys

def send_udp_message(message="hello", host="35.0.26.73", port=9999):

    """
    Sends a single UDP message to the specified host and port.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        bytes_sent = sock.sendto(message.encode('utf-8'), (host, port))
        print(f"Sent {bytes_sent} bytes to {host}:{port}")
    finally:
        sock.close()

if __name__ == '__main__':
    # Optionally allow overriding via command‑line args:
    # python udp_sender.py "hello" 127.0.0.1 9999
    msg = sys.argv[1] if len(sys.argv) > 1 else 'hello'
    dest_host = sys.argv[2] if len(sys.argv) > 2 else '127.0.0.1'
    dest_port = int(sys.argv[3]) if len(sys.argv) > 3 else 9999
    send_udp_message(msg, dest_host, dest_port)
