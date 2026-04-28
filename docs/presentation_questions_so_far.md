# Presentation Questions So Far

Questions asked after: "Its time for the presentation. Let me ask a few questions to understand things."

1. What is the hardware we're using?
2. How are we talking to the motors to move and stop?
3. Look aat the code as well and tell me if the code reflects this?
4. Explain the rqt graph to me please
5. How does the follow me function work? It has to get input from somewhere right?
6. Also, is there a map for the cart to follow?
7. What happens if the person is on the side? How does the cart and its sensors know that it has to turn left?

## Sensor Key Points (Quick Answers)

### Sensor Stack

1. UWB (4-anchor setup): tracks user position relative to the cart for follow-me.
2. LiDAR (2D front scan): detects obstacles and enforces safety speed reduction/stop.
3. Ultrasonic sensors (left and right): short-range redundancy for close obstacles.
4. IR distance sensors: extra short-range obstacle checks.
5. Wheel encoders: odometry feedback for movement estimation.

### UWB Talking Points

1. Four UWB ranges are converted into x, y position using trilateration.
2. A Kalman filter smooths noisy UWB estimates before control.
3. UWB is the primary person-tracking input for follow-me.
4. If UWB signal is stale/lost, the cart stops instead of guessing.

### LiDAR Talking Points

1. LiDAR is the main safety sensor for frontal obstacles.
2. Safety zones scale speed: emergency stop, slow zone, caution zone, normal zone.
3. If front path is blocked, steering bias prefers the side with more clearance.
4. LiDAR handles obstacle avoidance, not identity-level person tracking.

### Sensor Fusion Behavior

1. UWB says where the person is.
2. Follow controller converts that into linear and angular motion commands.
3. LiDAR/short-range sensors modify or block those commands for safety.
4. Final command is published only after safety checks.

### High-Value Viva Answers

1. How does the cart know to turn left?
   If person position is left of cart center, heading error is positive, so angular command turns left.
2. Why not LiDAR-only follow?
   LiDAR sees objects; UWB provides a direct target reference to the followed person.
3. Is this map-based navigation?
   Not in current implementation. This is reactive follow-me plus local obstacle safety.
