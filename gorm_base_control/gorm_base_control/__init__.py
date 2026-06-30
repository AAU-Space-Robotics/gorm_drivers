# Publisher for motor telemetry
self.motor_state_pub = self.create_publisher(
    Float64MultiArray,
    '/motor_states',
    10
)

# Publish every 100 ms
self.motor_state_timer = self.create_timer(
    0.5,
    self.publish_motor_states
)