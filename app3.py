from flask import Flask, render_template, jsonify
from flask_cors import CORS
import requests
import time
from threading import Thread, Lock
from datetime import datetime

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})

# Configuration
BASE_ROVER_API = "https://roverdata2-production.up.railway.app/api"
DASHBOARD_UPDATE_INTERVAL = 2  # seconds

# Global state
rover_state = {
    "session_id": None,
    "status": "disconnected",
    "battery": 0,
    "position": {"x": 0, "y": 0},
    "sensor_data": {
        "ultrasonic": {"distance": 0, "detection": False},
        "ir": {"reflection": False},
        "rfid": {"tag_detected": False},
        "accelerometer": {"x": 0.0, "y": 0.0, "z": 0.0}
    },
    "mission_log": ["System initialized - Ready to connect"],
    "survivor_log": [],
    "autonomous_mode": False,
    "last_communication": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    "recharging": False,
    "movement_lock": False
}
state_lock = Lock()

def log_event(event):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with state_lock:
        rover_state["mission_log"].append(f"{timestamp} - {event}")
        rover_state["last_communication"] = timestamp
        if len(rover_state["mission_log"]) > 100:
            rover_state["mission_log"] = rover_state["mission_log"][-100:]

def log_survivor(position):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with state_lock:
        survivor_entry = {
            "timestamp": timestamp,
            "position": position,
            "id": len(rover_state["survivor_log"]) + 1
        }
        rover_state["survivor_log"].append(survivor_entry)
        if len(rover_state["survivor_log"]) > 50:
            rover_state["survivor_log"] = rover_state["survivor_log"][-50:]

def start_rover_session():
    try:
        url = f"{BASE_ROVER_API}/session/start"
        response = requests.post(url, timeout=5)
        if response.status_code == 200:
            data = response.json()
            session_id = data.get("session_id")
            if session_id:
                with state_lock:
                    rover_state["session_id"] = session_id
                    rover_state["status"] = "idle"
                log_event("Rover session started successfully")
                print(session_id)
                return True
        log_event(f"Failed to start session: HTTP {response.status_code}")
        return False
    except Exception as e:
        log_event(f"Error starting session: {str(e)}")
        return False

def api_call_with_retry(url, method='get', max_retries=3):
    for attempt in range(max_retries):
        try:
            if method == 'get':
                response = requests.get(url, timeout=5)
            else:
                response = requests.post(url, timeout=5)
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            log_event(f"API call attempt {attempt + 1} failed: {str(e)}")
            time.sleep(1)
    return None

def autonomous_navigation():
    while True:
        with state_lock:
            if not rover_state["session_id"] or rover_state["movement_lock"]:
                time.sleep(1)
                continue
            session_id = rover_state["session_id"]
            rover_state["movement_lock"] = True

        try:
            # Fetch sensor data
            sensor_data = api_call_with_retry(
                f"{BASE_ROVER_API}/rover/sensor-data?session_id={session_id}"
            )
            if not sensor_data:
                continue

            # Update state with new sensor data
            with state_lock:
                rover_state["position"]["x"] = sensor_data.get("position", {}).get("x", 0)
                rover_state["position"]["y"] = sensor_data.get("position", {}).get("y", 0)
                rover_state["battery"] = sensor_data.get("battery_level", 0)
                rover_state["recharging"] = sensor_data.get("recharging", False)
                
                # Update sensor data
                rover_state["sensor_data"]["ultrasonic"] = sensor_data.get("ultrasonic", {})
                rover_state["sensor_data"]["ir"] = sensor_data.get("ir", {})
                rover_state["sensor_data"]["rfid"] = sensor_data.get("rfid", {})
                rover_state["sensor_data"]["accelerometer"] = sensor_data.get("accelerometer", {})

                battery = rover_state["battery"]
                ultrasonic_distance = rover_state["sensor_data"]["ultrasonic"].get("distance")
                ir_detected = rover_state["sensor_data"]["ir"].get("reflection", False)
                rfid_detected = rover_state["sensor_data"]["rfid"].get("tag_detected", False)

            # Log sensor data
            log_event(f"Sensor update - Battery: {battery}% | Distance: {ultrasonic_distance}cm | IR: {ir_detected} | RFID: {rfid_detected}")

            # Battery & Communication Handling
            if battery < 5 and not rover_state["recharging"]:
                log_event("Battery critically low! Initiating recharge sequence")
                recharge_rover(session_id)
                continue

            # Survivor Detection
            if rfid_detected:
                current_position = rover_state["position"].copy()
                log_event(f"Survivor detected at position ({current_position['x']}, {current_position['y']})")
                log_survivor(current_position)
                # Continue moving after logging survivor
                time.sleep(1)  # Brief pause to avoid duplicate detections
                move_rover(session_id, "forward")
                continue

            # Obstacle Avoidance
            if (ultrasonic_distance is not None and ultrasonic_distance < 20) or ir_detected:
                log_event(f"Obstacle detected [Distance: {ultrasonic_distance}cm, IR: {ir_detected}]")
                avoid_obstacle(session_id)
            else:
                # Default Movement
                move_rover(session_id, "forward")
                log_event("Moving forward")

        except Exception as e:
            log_event(f"Navigation error: {str(e)}")
        finally:
            with state_lock:
                rover_state["movement_lock"] = False

        time.sleep(2)

def avoid_obstacle(session_id):
    log_event("Starting obstacle avoidance maneuver")
    stop_rover(session_id)
    time.sleep(1)

    # Try turning right
    move_rover(session_id, "right")
    time.sleep(2)
    
    # Check if obstacle still present
    sensor_data = api_call_with_retry(
        f"{BASE_ROVER_API}/rover/sensor-data?session_id={session_id}"
    )
    if sensor_data and (sensor_data["ultrasonic"].get("distance", 100) < 20 or sensor_data["ir"].get("reflection", False)):
        # Try turning left if right didn't work
        move_rover(session_id, "left")
        time.sleep(2)
        
        sensor_data = api_call_with_retry(
            f"{BASE_ROVER_API}/rover/sensor-data?session_id={session_id}"
        )
        if sensor_data and (sensor_data["ultrasonic"].get("distance", 100) < 20 or sensor_data["ir"].get("reflection", False)):
            # Move backward if both directions blocked
            move_rover(session_id, "backward")
            time.sleep(2)

def recharge_rover(session_id):
    with state_lock:
        rover_state["status"] = "charging"
        rover_state["recharging"] = True
    log_event("Starting recharge sequence")
    
    stop_rover(session_id)
    api_call_with_retry(
        f"{BASE_ROVER_API}/rover/charge?session_id={session_id}",
        method='post'
    )

    while True:
        sensor_data = api_call_with_retry(
            f"{BASE_ROVER_API}/rover/sensor-data?session_id={session_id}"
        )
        if not sensor_data:
            continue

        battery = sensor_data.get("battery_level", 0)
        with state_lock:
            rover_state["battery"] = battery
        log_event(f"Recharging... Battery: {battery}%")

        if battery >= 80:
            log_event("Battery recharged to 80%. Resuming movement.")
            with state_lock:
                rover_state["status"] = "idle"
                rover_state["recharging"] = False
            return
        
        time.sleep(5)

def move_rover(session_id, direction):
    response = api_call_with_retry(
        f"{BASE_ROVER_API}/rover/move?session_id={session_id}&direction={direction}",
        method='post'
    )
    if response:
        with state_lock:
            rover_state["status"] = f"moving_{direction}"
        return True
    return False

def stop_rover(session_id):
    response = api_call_with_retry(
        f"{BASE_ROVER_API}/rover/stop?session_id={session_id}",
        method='post'
    )
    if response:
        with state_lock:
            rover_state["status"] = "idle"
        return True
    return False

@app.after_request
def add_header(response):
    response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '-1'
    return response

@app.route('/')
def dashboard():
    return render_template('dashboard.html')

@app.route('/api/state')
def get_state():
    with state_lock:
        return jsonify(rover_state)

@app.route('/api/connect', methods=['POST'])
def connect():
    try:
        if start_rover_session():
            Thread(target=autonomous_navigation, daemon=True).start()
            return jsonify({
                "success": True,
                "message": "Rover connected and autonomous mode started",
                "session_id": rover_state["session_id"]
            })
        return jsonify({
            "success": False,
            "message": "Failed to start rover session"
        }), 400
    except Exception as e:
        return jsonify({
            "success": False,
            "message": str(e)
        }), 500

if __name__ == '__main__':
    Thread(target=autonomous_navigation, daemon=True).start()
    app.run(host='0.0.0.0', port=5000, debug=True)