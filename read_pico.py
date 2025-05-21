import requests
import time

PICO_IP = "192.168.137.152"
url    = f"http://{PICO_IP}/data"

# initialize with zeros (or None, or whatever makes sense)
last_data = {"front": 0.0, "back": 0.0, "left": 0.0}

while True:
    try:
        res = requests.get(url, timeout=1)
        res.raise_for_status()            # HTTP-level errors → RequestException
        data = res.json()                 # JSON errors → ValueError

        # if we get here, data is valid; update cache
        last_data = data

    except ValueError:
        # JSON decoding failed; keep last_data silently
        pass

    except requests.RequestException as e:
        # network/HTTP error
        print("Request failed:", e)

    # always print whatever is in last_data
    print(f"Front: {last_data['front']:.2f} cm")
    print(f"Back : {last_data['back']:.2f} cm")
    print(f"Left : {last_data['left']:.2f} cm")
    print("-" * 30)

    time.sleep(1)
