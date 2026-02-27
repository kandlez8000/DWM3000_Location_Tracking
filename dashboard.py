import serial
import threading
import time
import re
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mpimg
import numpy as np
from scipy.optimize import least_squares
import os

# ---------------- CONFIGURATION ---------------- #
SERIAL_PORT = "COM25"  # Your Gateway's COM port
BAUD_RATE = 115200

ROOM_WIDTH = 435   # cm (Matches your 434.34cm width)
ROOM_HEIGHT = 239  # cm (Matches your 238.76cm length)
IMAGE_FILE = "floorplan.png"  

# Physical Anchor Coordinates (in cm). 
# Add or remove lines here based on how many are plugged in!
ANCHOR_POSITIONS = {
    1: (0.0, 0.0),            # Bottom Left
    2: (434.3, 0.0),          # Bottom Right
    3: (434.3, 238.8),        # Top Right
    4: (0.0, 238.8),        # Top Left (Uncomment when you add Anchor 4!)
}

# ------------------------------------------------ #
# Global variables
latest_data = {} 
data_lock = threading.Lock()
running = True
path_x, path_y = [], []

def trilaterate(distances_dict, anchor_dict):
    """Dynamically calculates position using however many anchors are available."""
    # Only use anchors we have BOTH coordinates and live distances for
    valid_ids = [aid for aid in distances_dict.keys() if aid in anchor_dict]
    
    if len(valid_ids) < 3:
        return None # Math requires at least 3

    dists = [distances_dict[aid] for aid in valid_ids]
    anchors = [anchor_dict[aid] for aid in valid_ids]

    def equations(p):
        x, y = p
        eqs = []
        for i in range(len(anchors)):
            eqs.append(np.sqrt((x - anchors[i][0])**2 + (y - anchors[i][1])**2) - dists[i])
        return eqs

    initial_guess = np.mean(anchors, axis=0)

    try:
        result = least_squares(equations, initial_guess, method="lm")
        return result.x if result.success else None
    except Exception:
        return None

def serial_reader():
    """Reads raw string data from the ESP32 Gateway via USB"""
    global latest_data, running
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Listening on {SERIAL_PORT}...")
    except Exception as e:
        print(f"Serial Error: {e}")
        running = False
        return

    # Regex to grab "A1: 154.23 cm"
    regex = re.compile(r'A(\d+): ([\d\.]+) cm')

    while running:
        try:
            line = ser.readline().decode("utf-8").strip()
            if not line or "Tag ID" not in line:
                continue
            
            matches = regex.findall(line)
            new_data = {}
            for match in matches:
                a_id = int(match[0])
                dist = float(match[1])
                new_data[a_id] = dist
            
            with data_lock:
                latest_data = new_data
                
        except Exception:
            pass
    ser.close()

# ---------------- PLOTTING SETUP ---------------- #
fig, ax = plt.subplots(figsize=(10, 8))

# Attempt to load background image
if os.path.exists(IMAGE_FILE):
    bg_img = mpimg.imread(IMAGE_FILE)
    img_extent = [0, ROOM_WIDTH, 0, ROOM_HEIGHT]
    ax.imshow(bg_img, extent=img_extent, origin="lower", alpha=0.6, zorder=-1)
else:
    print(f"[WARNING] {IMAGE_FILE} not found. Drawing blank grid.")

anchor_texts = []
colors = ['g', 'b', 'm', 'c', 'y', 'orange', 'purple', 'brown']

# Plot the known anchors dynamically
for i, (aid, coords) in enumerate(ANCHOR_POSITIONS.items()):
    c = colors[i % len(colors)]
    ax.plot(coords[0], coords[1], marker='^', color=c, markersize=12, label=f"Anchor {aid}")
    txt = ax.text(coords[0], coords[1] + 10, f"A{aid}", color=c, ha="center", va="bottom", fontweight='bold')
    anchor_texts.append(txt)

(tag_dot,) = ax.plot([], [], "ro", markersize=10, label="Tag Position")
(path_line,) = ax.plot([], [], "b-", alpha=0.5, linewidth=2, label="Tag Path")

ax.set_xlim(0 - 20, ROOM_WIDTH + 20)
ax.set_ylim(0 - 20, ROOM_HEIGHT + 20)
ax.set_aspect("equal")
ax.grid(True, linestyle="--", alpha=0.7)
ax.legend(loc="upper right")
ax.set_title("Real-time UWB Tracking (Dynamic Anchors)", pad=20)
ax.set_xlabel("X Position (cm)")
ax.set_ylabel("Y Position (cm)")

# ---------------- ANIMATION FUNCTION ---------------- #
def update(frame):
    global latest_data, path_x, path_y

    with data_lock:
        if len(latest_data) >= 3:
            pos = trilaterate(latest_data, ANCHOR_POSITIONS)
            
            if pos is not None:
                x_cm, y_cm = pos
                tag_dot.set_data([x_cm], [y_cm])
                
                path_x.append(x_cm)
                path_y.append(y_cm)
                
                # Keep only the last 50 points so the screen doesn't get totally scribbled on
                if len(path_x) > 50:
                    path_x.pop(0)
                    path_y.pop(0)
                path_line.set_data(path_x, path_y)

    return tag_dot, path_line, *anchor_texts

# Start the background Serial thread
serial_thread = threading.Thread(target=serial_reader, daemon=True)
serial_thread.start()

# Start the Matplotlib animation loop
ani = animation.FuncAnimation(fig, update, interval=100, blit=True, cache_frame_data=False)

try:
    plt.show()
except KeyboardInterrupt:
    running = False
    serial_thread.join()