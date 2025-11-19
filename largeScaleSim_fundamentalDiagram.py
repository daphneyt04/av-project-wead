import numpy as np
import matplotlib.pyplot as plt

# ============================================================
#  1. CONTROLLERS
# ============================================================

def wead_controller(sn, vn):
    # Parameters
    STOPPED_SPEED_TH = 1.0
    STOPPED_MIN_GAP = 5.0
    STOPPED_ACCEL = 1.0
    MAX_ACCEL = 2.5
    MAX_BRAKE = -2.5
    CRUISE_SPEED_LIMIT = 36.0

    # State 1: Stopped logic
    if vn <= STOPPED_SPEED_TH:
        if sn > STOPPED_MIN_GAP:
            return STOPPED_ACCEL
        else:
            return 0.0

    # State 2: Moving logic
    time_gap = sn / vn if vn > 0 else float('inf')

    if time_gap <= 3:
        return MAX_BRAKE
    elif time_gap < 4:
        return (2.5 * time_gap) - 10
    elif time_gap <= 5:
        return 0.0
    elif time_gap < 6:
        return (2.5 * time_gap) - 12.5
    elif time_gap >= 6:
        return MAX_ACCEL if vn <= CRUISE_SPEED_LIMIT else 0.0

    return 0.0

def profacc_controller(sn, vn, alpha=0.5, tau=1.2):
    return alpha * (sn - tau * vn)


# ============================================================
#  2. SIMULATION CORE
# ============================================================

def get_leader_velocity(t, profile_type, target_speed=20.0):
    """
    Generates different leader profiles for different tests.
    """
    if profile_type == "constant":
        # Ramps up to target_speed and holds it. Used for Fundamental Diagram.
        if t < 10:
            return (target_speed / 10.0) * t
        else:
            return target_speed
            
    elif profile_type == "sine_wave":
        # Oscillates around 20 m/s. Used for Stability/Energy analysis.
        base_speed = 20.0
        return base_speed + 3.0 * np.sin(0.3 * t)
    
    elif profile_type == "stop_and_go":
        # Brakes to a stop then recovers.
        if t < 10: return 20.0
        elif t < 15: return max(0, 20.0 - 4.0 * (t-10))
        elif t < 25: return 0.0
        else: return min(20.0, (t-25) * 2.0)
        
    return 20.0

def simulate_platoon(num_wead_cars, profile_type="constant", target_speed=20.0):
    """
    Simulates a 12-car platoon (1 Leader + 11 Followers).
    Returns time, positions, velocities, and accelerations.
    """
    dt = 0.05
    T = 60 # Duration
    N = int(T/dt)
    t = np.linspace(0, T, N)
    
    # 12 Cars: Index 0 is Leader, 1-11 are Followers
    total_cars = 12
    
    # Assign controllers (Interspersed logic)
    # Default everyone to PROFACC
    controllers = ["LEADER"] + ["PROFACC"] * (total_cars - 1)
    
    # Intersparse WEAD cars
    # We map 'num_wead_cars' across the 11 followers
    followers_indices = list(range(1, total_cars))
    if num_wead_cars > 0:
        spacing = len(followers_indices) / num_wead_cars
        for i in range(num_wead_cars):
            idx = int(i * spacing)
            if idx < len(followers_indices):
                real_idx = followers_indices[idx]
                controllers[real_idx] = "WEAD"

    # State arrays
    x = np.zeros((total_cars, N))
    v = np.zeros((total_cars, N))
    a = np.zeros((total_cars, N))
    
    # Initial spacing
    init_spacing = 20.0
    for i in range(total_cars):
        x[i, 0] = -init_spacing * i
        v[i, 0] = 0 # Start from stop to let them stabilize

    # Main Loop
    for k in range(1, N):
        # 1. Update Leader
        v[0, k] = get_leader_velocity(t[k], profile_type, target_speed)
        x[0, k] = x[0, k-1] + v[0, k] * dt
        
        # 2. Update Followers
        for i in range(1, total_cars):
            sn = x[i-1, k-1] - x[i, k-1] # Gap to car in front
            vn = v[i, k-1]               # Own velocity
            
            if controllers[i] == "WEAD":
                acc = wead_controller(sn, vn)
            else:
                acc = profacc_controller(sn, vn)
            
            a[i, k] = acc
            v[i, k] = max(0, v[i, k-1] + acc * dt) # No reversing
            x[i, k] = x[i, k-1] + v[i, k] * dt
            
    return t, x, v, a


# ============================================================
#  3. ANALYSIS FUNCTIONS
# ============================================================

def run_fundamental_diagram_analysis():
    """
    Generates the Flow (q) vs Density (k) curve.
    Varies the leader speed to find equilibrium gaps.
    """
    print("Running Fundamental Diagram Analysis...")
    
    # Test speeds (m/s) - we use this to sweep through densities
    target_speeds = np.arange(2, 38, 2) 
    
    penetration_scenarios = {
        "0% WEAD": 0,
        "50% WEAD": 5, 
        "100% WEAD": 11
    }
    
    plt.figure(figsize=(10, 6))
    
    for label, num_wead in penetration_scenarios.items():
        densities = [] # k (veh/km)
        flows = []     # q (veh/hr)
        
        for v_target in target_speeds:
            # Run simulation with constant speed
            _, x, v, _ = simulate_platoon(num_wead, profile_type="constant", target_speed=v_target)
            
            # Take the average of the last 10 seconds (steady state)
            # Exclude leader from average
            last_steps = int(10 / 0.05) # Last 10 seconds
            
            # Calculate average gap in the platoon
            # Gap = (Position of Car 1 - Position of Car 11) / 10 gaps
            platoon_length = np.mean(x[1, -last_steps:] - x[11, -last_steps:])
            avg_gap = platoon_length / 10.0
            
            # Calculate Density k (veh/km)
            # k = 1000 / (avg_gap + car_length)
            car_length = 5.0
            k = 1000.0 / (avg_gap + car_length)
            
            # Calculate Flow q (veh/hr)
            # q = k * v * 3.6
            # Use actual average velocity of followers, not target speed
            avg_v_platoon = np.mean(v[1:, -last_steps:])
            q = k * avg_v_platoon * 3.6
            
            densities.append(k)
            flows.append(q)
            
        plt.plot(densities, flows, '-o', linewidth=2, label=label)
        
    plt.title("Fundamental Diagram: Flow vs. Density")
    plt.xlabel("Density (veh/km)")
    plt.ylabel("Flow (veh/hr)")
    plt.legend()
    plt.grid(True)
    plt.show()

def run_stability_and_energy_analysis():
    """
    Analyzes String Stability and Energy Consumption varying penetration rate.
    Uses a Sine Wave disturbance.
    """
    print("Running Stability & Energy Analysis...")
    
    # We test every possible number of WEAD cars (0 to 11)
    wead_counts = range(0, 12)
    penetration_rates = [ (n / 11.0) * 100 for n in wead_counts ]
    
    stability_ratios = []
    energies = []
    
    for n in wead_counts:
        # Run with Sine Wave disturbance
        _, _, v, a = simulate_platoon(n, profile_type="sine_wave")
        
        # --- Metric 1: String Stability ---
        # Ratio of StdDev(Velocity) of Last Car / First Follower
        # > 1 means unstable (amplified), < 1 means stable (dampened)
        std_first = np.std(v[1, :]) # Car 1
        std_last = np.std(v[11, :]) # Car 11
        
        ratio = std_last / std_first if std_first != 0 else 0
        stability_ratios.append(ratio)
        
        # --- Metric 2: Energy Consumption ---
        # Sum of squared acceleration for all followers (proxy for fuel)
        # Energy = Sum(a^2) * dt
        total_accel_sq = np.sum(a[1:, :]**2) * 0.05
        energies.append(total_accel_sq)
        
    # Plot Stability
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.plot(penetration_rates, stability_ratios, '-o', color='purple')
    plt.axhline(y=1.0, color='r', linestyle='--', label='Stability Limit')
    plt.title("String Stability vs. Penetration Rate")
    plt.xlabel("% WEAD Vehicles")
    plt.ylabel("Stability Ratio (Std_Last / Std_First)")
    plt.legend()
    plt.grid(True)
    
    # Plot Energy
    plt.subplot(1, 2, 2)
    plt.plot(penetration_rates, energies, '-s', color='green')
    plt.title("Total Energy vs. Penetration Rate")
    plt.xlabel("% WEAD Vehicles")
    plt.ylabel("Energy Proxy (Sum of a^2)")
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Run the full analysis suite
    run_fundamental_diagram_analysis()
    run_stability_and_energy_analysis()