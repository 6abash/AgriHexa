# AgriHexa - Satellite Terrain Path Planner
#
# Ground-station tool: takes a satellite/aerial image of a field, builds
# a terrain-cost grid from it (vegetation vs. slope/roughness), lets the
# operator click multiple waypoints on the image, runs A* between them
# to find the route with the least incline/decline and roughest terrain,
# then streams the resulting move commands to the ESP32 over TCP and
# animates the mission live.
#
import numpy as np
import heapq
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap
import cv2
import socket
import time

# =========================================================================
# PREMIUM AGRICULTURAL COLOUR PALETTE
# =========================================================================
AGRI_TERRAIN_COLORS = ["#52a13b", "#e0b034", "#8b5a2b", "#3d3d3d"]
agri_cmap = LinearSegmentedColormap.from_list("AgriTerrain", AGRI_TERRAIN_COLORS)

# =========================================================================
# PHASE 1 & 2: IMAGE RECOGNITION AND MAP GENERATION
# =========================================================================
def generate_synthetic_satellite_view(filename="field_map.jpg", size=512):
    """Fallback: synthesize a plausible satellite view when no real image is found."""
    print("Generating a synthetic RGB satellite photo...")
    X, Y = np.meshgrid(np.arange(size), np.arange(size))
    dx, dy = size * 0.9, size * 0.3
    t = np.clip((X * dx + Y * dy) / (dx**2 + dy**2), 0.0, 1.0)
    dist_greater = np.sqrt((X - t*dx)**2 + (Y - t*dy)**2)
    ridge = np.exp(-(dist_greater**2) / (2.0 * (size * 0.15)**2))

    np.random.seed(42)
    octave_1 = cv2.resize(np.random.rand(8, 8), (size, size), interpolation=cv2.INTER_CUBIC)
    octave_2 = cv2.resize(np.random.rand(32, 32), (size, size), interpolation=cv2.INTER_CUBIC)
    noise = 0.7 * octave_1 + 0.3 * octave_2

    height_map = ridge * (0.2 + 0.8 * noise)
    height_map = (height_map - np.min(height_map)) / (np.max(height_map) - np.min(height_map))

    sat_img = np.zeros((size, size, 3), dtype=np.uint8)
    for r in range(size):
        for c in range(size):
            h = height_map[r, c]
            if h < 0.25: sat_img[r, c] = [34, 120, 40]
            elif h < 0.55: sat_img[r, c] = [50, 140, 190]
            else: sat_img[r, c] = [100, 100, 100]

    sat_img_blurred = cv2.GaussianBlur(sat_img, (15, 15), 0)
    rough_texture = (np.random.randn(size, size, 3) * 10).astype(np.int16)
    final_sat = np.clip(sat_img_blurred.astype(np.int16) + rough_texture, 0, 255).astype(np.uint8)
    cv2.imwrite(filename, final_sat)
    print(f"Satellite photo saved as '{filename}'!")

def process_screenshot(image_path, grid_size=60):
    """Loads a satellite image and converts it into a terrain-cost grid.

    Cost is driven by vegetation coverage (HSV green mask -> lower cost)
    and local roughness (Sobel gradient magnitude -> higher cost), so A*
    naturally prefers flat, vegetated ground over steep or built terrain.
    """
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if img is None:
        generate_synthetic_satellite_view(image_path)
        img = cv2.imread(image_path, cv2.IMREAD_COLOR)

    img_resized = cv2.resize(img, (grid_size, grid_size))
    img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)

    gray = cv2.cvtColor(img_resized, cv2.COLOR_BGR2GRAY)
    sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    roughness = np.sqrt(sobel_x**2 + sobel_y**2)
    roughness_norm = (roughness - np.min(roughness)) / (np.max(roughness) - np.min(roughness) + 1e-5)

    hsv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 30, 30])
    upper_green = np.array([85, 255, 255])
    green_mask = cv2.inRange(hsv, lower_green, upper_green) / 255.0

    base_cost = 40.0 - (35.0 * green_mask)
    added_penalty = roughness_norm * 70.0
    grid_cost = np.clip(base_cost + added_penalty, 1.0, 99.0)
    grid_cost[grid_cost > 75] = 99  # treat as impassable (steep incline/decline, obstacle)

    return np.round(grid_cost, 1), img_rgb

# ==========================================
# PHASE 3 & 4: PATHFINDING AND COMMANDS
# ==========================================
def heuristic(a, b):
    return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def a_star_search(grid, start, goal):
    """8-connected A* over the terrain-cost grid; cells >= 99 are impassable."""
    neighbors = [(0,1),(0,-1),(1,0),(-1,0), (1,1), (-1,-1), (1,-1), (-1,1)]
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        current_f, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                terrain_cost = grid[neighbor[0], neighbor[1]]
                if terrain_cost >= 99: continue

                tentative_g_score = g_score[current] + terrain_cost
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
    return []

def collect_interactive_waypoints(grid, img_rgb):
    """Lets the operator click waypoints directly on the satellite image."""
    plt.ion() if hasattr(plt, 'ion') else None
    fig, ax = plt.subplots(figsize=(11, 8))

    ax.imshow(img_rgb, origin='upper')
    cost_map = ax.imshow(grid, cmap=agri_cmap, origin='upper', alpha=0.45)

    ax.set_title("Interactive Satellite Planner: Place Waypoints (Press ENTER to finish)", fontsize=10, weight='bold')
    fig.colorbar(cost_map, ax=ax, label="Terrain Difficulty Cost (1 - 99)")

    clicks = plt.ginput(n=-1, timeout=0, show_clicks=True)
    plt.close(fig)
    if hasattr(plt, 'ioff'): plt.ioff()

    waypoints = []
    for i, (x, y) in enumerate(clicks):
        row = max(0, min(int(round(y)), grid.shape[0] - 1))
        col = max(0, min(int(round(x)), grid.shape[1] - 1))
        waypoints.append((row, col))
    return waypoints

def translate_path_to_commands(full_path):
    """Converts a grid path into the F/B/L/R/FR/FL/BR/BL command set the ESP32 understands."""
    commands = []
    for i in range(len(full_path) - 1):
        curr, nxt = full_path[i], full_path[i+1]
        d_row, d_col = nxt[0] - curr[0], nxt[1] - curr[1]

        if d_row == -1 and d_col == 0: commands.append("F")
        elif d_row == 1 and d_col == 0: commands.append("B")
        elif d_row == 0 and d_col == -1: commands.append("L")
        elif d_row == 0 and d_col == 1: commands.append("R")
        elif d_row == -1 and d_col == 1: commands.append("FR")
        elif d_row == -1 and d_col == -1: commands.append("FL")
        elif d_row == 1 and d_col == 1: commands.append("BR")
        elif d_row == 1 and d_col == -1: commands.append("BL")
        else: commands.append("S")
    return commands

# =========================================================================
# PHASE 5: TELEMETRY STREAMER (Direct to ESP32 Only)
# =========================================================================
def execute_mission_and_animate(grid, img_rgb, path_segments, waypoints, commands, master_path, esp_ip, port=80, mock_hardware=False):
    # Safety Check: Prevent IndexError if path is completely empty
    if not master_path:
        print("\n[CRITICAL ERROR]: The computed route is empty. Simulation aborted.")
        return

    client_socket = None
    if not mock_hardware:
        print("\n" + "="*60)
        print(f"[Wi-Fi Client]: Attempting strict TCP connection to ESP32 at {esp_ip}:{port}...")
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.settimeout(5.0) # 5 seconds to find the ESP32
            client_socket.connect((esp_ip, port))
            client_socket.setblocking(False) # Non-blocking for the visual loop
            print("[Wi-Fi Client]: Connected successfully! Hardware link established.")
            print("="*60 + "\n")
        except Exception as e:
            print("\n" + "!"*60)
            print(f"[FATAL CONNECTION ERROR]: Could not connect to ESP32 at {esp_ip}:{port}")
            print(f"Details: {e}")
            print("\nTROUBLESHOOTING CHECKLIST:")
            print("1. Is your laptop on the same Wi-Fi network as the robot?")
            print("2. Is the ESP32 powered on?")
            print(f"3. Does the ESP32 serial monitor confirm its IP is EXACTLY {esp_ip}?")
            print("!"*60 + "\n")
            print("Halting execution. Fix the network connection and restart the script.")
            return # Exit safely without crashing the IDE

    # Initialize Visuals
    plt.ion()
    fig, (ax_sat, ax_cost) = plt.subplots(1, 2, figsize=(16, 8))

    ax_sat.imshow(img_rgb, origin='upper')
    cost_img = ax_cost.imshow(grid, cmap=agri_cmap, origin='upper')
    fig.colorbar(cost_img, ax=ax_cost)

    colors = ['cyan', 'magenta', 'orange', 'red']
    path_lines_sat, path_lines_cost = [], []
    for i, path in enumerate(path_segments):
        if path:
            py, px = [p[0] for p in path], [p[1] for p in path]
            col = colors[i % len(colors)]
            line_sat, = ax_sat.plot(px, py, color=col, linewidth=4, alpha=0.8)
            line_cost, = ax_cost.plot(px, py, color=col, linewidth=4, alpha=0.8)
            path_lines_sat.append(line_sat); path_lines_cost.append(line_cost)

    for i, pt in enumerate(waypoints):
        ax_sat.scatter(pt[1], pt[0], color='lime' if i==0 else 'red', s=150, zorder=5)
        ax_cost.scatter(pt[1], pt[0], color='lime' if i==0 else 'red', s=150, zorder=5)

    twin_sat = ax_sat.scatter(master_path[0][1], master_path[0][0], color='yellow', s=250, marker='H', edgecolor='black', zorder=10)
    twin_cost = ax_cost.scatter(master_path[0][1], master_path[0][0], color='yellow', s=250, marker='H', edgecolor='black', zorder=10)

    fig.canvas.draw()
    plt.show(block=False)

    idx = 0
    try:
        while idx < len(commands):
            cmd = commands[idx]
            print(f"Step {idx+1}/{len(commands)}: Sending '{cmd}'... ", end="", flush=True)

            ack_received = False
            start_wait = time.time()
            step_duration = 0.40 if mock_hardware else 5.0 # 5s timeout waiting for ACK

            if not mock_hardware:
                try:
                    client_socket.sendall(f"{cmd}\n".encode('utf-8'))
                except socket.error as e:
                    print(f"\n[CRITICAL]: Connection dropped mid-mission! Error: {e}")
                    break

            while (time.time() - start_wait) < step_duration:
                # Poll TCP for ACK from ESP32
                if not mock_hardware:
                    try:
                        data = client_socket.recv(1024).decode('utf-8').strip()
                        if "ACK" in data:
                            ack_received = True
                            break
                    except socket.error:
                        pass

                # Keep graphics responsive
                fig.canvas.flush_events()
                time.sleep(0.01)

            if mock_hardware:
                print("Simulated.")
                idx += 1
            else:
                if ack_received:
                    print("ACK Confirmed.")
                    idx += 1
                else:
                    print("\n[Hardware Timeout]: ESP32 did not send an ACK. Retrying step...")

            if idx < len(master_path):
                next_pos = master_path[idx]
                twin_sat.set_offsets([[next_pos[1], next_pos[0]]])
                twin_cost.set_offsets([[next_pos[1], next_pos[0]]])
                fig.canvas.draw_idle()

    except KeyboardInterrupt:
        print("\n[EMERGENCY STOP]: Aborting!")
        if not mock_hardware and client_socket:
            client_socket.sendall("S\n".encode('utf-8'))
    finally:
        if not mock_hardware and client_socket:
            client_socket.sendall("S\n".encode('utf-8'))
            client_socket.close()
        plt.ioff()
        print("\nMission finished. Closing sockets.")
        plt.show()

# ==========================================
# EXECUTION ENTRY POINT
# ==========================================
if __name__ == "__main__":
    terrain_grid, satellite_rgb = process_screenshot("field_map.jpg", 60)
    waypoints = collect_interactive_waypoints(terrain_grid, satellite_rgb)
    if len(waypoints) < 2: waypoints = [(5, 5), (25, 45), (52, 12)]

    path_segments = []
    mission_success = True

    for i in range(len(waypoints) - 1):
        leg_path = a_star_search(terrain_grid, waypoints[i], waypoints[i+1])
        if leg_path:
            path_segments.append(leg_path)
        else:
            print(f"\n[WARNING]: Path between node {chr(65+i)} and {chr(65+i+1)} is completely blocked!")
            mission_success = False

    master_path = []
    if mission_success and path_segments:
        for i, segment in enumerate(path_segments):
            master_path.extend(segment if i == 0 else segment[1:])

        movement_commands = translate_path_to_commands(master_path)

        # Static IP of the ESP32 on the field network (see /firmware)
        esp32_ip = "192.168.0.51"

        execute_mission_and_animate(
            terrain_grid, satellite_rgb, path_segments, waypoints,
            movement_commands, master_path, esp_ip=esp32_ip, port=80,
            mock_hardware=False # Set to True to dry-run without a live Wi-Fi socket
        )
    else:
        print("\n[CRITICAL ERROR]: Valid path could not be found. Please restart the script and click valid terrain.")
