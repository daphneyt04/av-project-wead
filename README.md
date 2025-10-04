# av-project-wead
autonomous vehicles controller by william, daphney, ava, and ellie

# Assumptions 
Normal following speed (Vn) = 11 m/s?
Range for cmd_accel = [-3.0, 1.5] | Can go to -6.0 in specific instances


# Single Vehicle Simulations
* 1.1 Lead vehicle suddenly stops
    - (Rapid decrease in space gap)
    - Validate no DIV 0 error
    - Expecting negative ouput
* 1.2 Vehicle is stationary, yet space/time gap is too low
    - (Vn = 0, Time Gap < Tau)
    - Should NOT reverse
* 1.3 Space/time gap is greater than desired
    - (Vn = anything, Time Gap > Tau)
* 1.4 Space/time gap is smaller than desired
    - (Vn = anything, Time Gap < Tau)
* 1.5 Lead vehicle disappears?    
    - (Vn = anything, Sn = undefined, Vn-1 = undefined)
    - Does controller react if no inputs?
1.6

# Multi Vehicle Simulations
* 2.1 Lead vehicle is stationary but ego vehicle is moving 
    - (Vn > 0, Vn-1 > 0)  sn (may test against multiple), vn - 1 = 0) 
* 2.2 Ego vehicle is stationary but time/space gap is too low
    - (Vn = 0, Time Gap < Tau)
    - Should NOT reverse
* 2.3 Lead car decelerates (simulates arriving at traffic wave)
    - (Vn > 0, delta Vn-1 < 0)
* 2.4 Ego vehicle and lead vehicle collide
    - (Time/Space Gap < 0)
    - Should throw error
* 2.5 Lead car accelerates 
    - (Increase in space/time gap, Vn = anything, delta Vn-1 > 0)
    - Add Vnmax to counter capped desired speed?
* 2.6 Lead car exits lane (cut-out)
    - Instantaneous increase in space/time gap
    - (Vn > 0, Vn-1 = anything)
* 2.7 Lead vehicle enters lane (cut-in)
    - Instantaneous decrease in space/time gap
    - (Vn > 0, Vn-1 = anything)
* 2.8 Ego vehicle car enters different lane
    - (Vn > 0, Vn-1 = anything)
* 2.9 Lead vehicle disappears?    
    - (Vn = anything, Sn = undefined, Vn-1 = undefined)
    - Does controller react if no inputs?
* 2.10

# AI Usage
