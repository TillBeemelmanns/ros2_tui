#!/usr/bin/env python3
"""
Simple dummy publisher for basic testing with interesting patterns
Publishes basic topics with jitter, sine waves, and variance for compelling plots
"""

import rclpy
from rclpy.node import Node
import math
import time
import random
import numpy as np
from std_msgs.msg import String, Int32, Float64, Bool


class DummyPublisher(Node):
    """Simple publisher with interesting timing patterns for testing"""
    
    def __init__(self):
        super().__init__('dummy_publisher')
        
        # Create publishers with different patterns
        self.string_pub = self.create_publisher(String, '/test/string_topic', 10)
        self.int_pub = self.create_publisher(Int32, '/test/int_topic', 10)
        self.float_pub = self.create_publisher(Float64, '/test/float_topic', 10)
        self.bool_pub = self.create_publisher(Bool, '/test/bool_topic', 10)
        
        # Additional interesting topics with different frequencies
        self.sine_wave_pub = self.create_publisher(Float64, '/test/sine_wave', 10)
        self.noisy_signal_pub = self.create_publisher(Float64, '/test/noisy_signal', 10)
        self.step_function_pub = self.create_publisher(Float64, '/test/step_function', 10)
        self.burst_pub = self.create_publisher(Int32, '/test/burst_data', 10)
        
        # Timers with different frequencies and patterns
        self.counter = 0
        self.start_time = time.time()
        
        # Main timer - 10 Hz
        self.create_timer(0.1, self.publish_high_freq)
        
        # Medium frequency - 2 Hz  
        self.create_timer(0.5, self.publish_medium_freq)
        
        # Low frequency with bursts - 0.2 Hz base
        self.create_timer(5.0, self.publish_low_freq_burst)
        
        # Variable timing timer
        self.create_timer(0.2, self.publish_variable_timing)
        
        self.get_logger().info('Enhanced dummy publisher started with multiple timing patterns')
    
    def add_timing_jitter(self, base_delay_ms=1.0, jitter_ms=2.0):
        """Add realistic timing jitter to simulate real-world conditions"""
        jitter = np.random.normal(0, jitter_ms / 1000.0)
        delay = max(0.0001, (base_delay_ms / 1000.0) + jitter)
        time.sleep(delay)
    
    def publish_high_freq(self):
        """High frequency publisher with sine wave and jitter - great for Hz plots"""
        self.add_timing_jitter(1.0, 3.0)  # 1ms base + 3ms jitter
        self.counter += 1
        elapsed = time.time() - self.start_time
        
        # Sine wave with multiple harmonics for interesting patterns  
        sine_value = (
            3.0 * math.sin(elapsed * 0.5) +           # Slow wave
            1.5 * math.sin(elapsed * 2.0) +           # Medium wave 
            0.5 * math.sin(elapsed * 5.0) +           # Fast wave
            np.random.normal(0, 0.2)                  # Noise
        )
        
        msg = Float64()
        msg.data = sine_value
        self.sine_wave_pub.publish(msg)
    
    def publish_medium_freq(self):
        """Medium frequency with interesting step patterns"""
        self.add_timing_jitter(5.0, 8.0)  # 5ms base + 8ms jitter
        elapsed = time.time() - self.start_time
        
        # Create step function with occasional spikes
        step_level = int(elapsed / 10) % 4  # Change level every 10 seconds
        base_value = [10.0, 25.0, 5.0, 40.0][step_level]
        
        # Add occasional spikes (10% chance)
        spike = 0
        if random.random() < 0.1:
            spike = random.uniform(50, 100)
        
        # Add small sine wave modulation
        modulation = 3.0 * math.sin(elapsed * 0.8)
        
        msg = Float64()
        msg.data = base_value + spike + modulation + np.random.normal(0, 1.0)
        self.step_function_pub.publish(msg)
        
        # Also publish traditional int/string/bool with patterns
        string_msg = String()
        string_msg.data = f"Pattern #{self.counter} - Level {step_level}"
        self.string_pub.publish(string_msg)
        
        int_msg = Int32()
        int_msg.data = int(base_value + modulation)
        self.int_pub.publish(int_msg)
        
        bool_msg = Bool()
        # Create interesting boolean pattern based on sine waves
        bool_pattern = math.sin(elapsed * 0.3) + 0.5 * math.sin(elapsed * 1.2)
        bool_msg.data = bool_pattern > 0.2
        self.bool_pub.publish(bool_msg)
    
    def publish_low_freq_burst(self):
        """Low frequency publisher with occasional bursts"""
        elapsed = time.time() - self.start_time
        
        # Decide if this is a burst period (20% chance)
        is_burst = random.random() < 0.2
        
        if is_burst:
            # Publish multiple messages rapidly during burst
            for i in range(random.randint(5, 15)):
                self.add_timing_jitter(2.0, 1.0)  # Small jitter during bursts
                msg = Int32()
                msg.data = 1000 + i + int(100 * math.sin(elapsed + i))
                self.burst_pub.publish(msg)
        else:
            # Single message during normal period
            self.add_timing_jitter(10.0, 20.0)  # High jitter during normal
            msg = Int32() 
            msg.data = int(50 * math.sin(elapsed * 0.1)) + np.random.randint(-10, 10)
            self.burst_pub.publish(msg)
    
    def publish_variable_timing(self):
        """Publisher with highly variable timing - interesting delay patterns"""
        elapsed = time.time() - self.start_time
        
        # Calculate complex delay pattern with sine waves
        base_jitter = 5.0  # 5ms base
        sine_jitter = 15.0 * math.sin(elapsed * 0.2)  # Slow sine variation
        fast_jitter = 5.0 * math.sin(elapsed * 3.0)   # Fast sine variation  
        random_spike = 0
        
        # Occasional long delays (2% chance)
        if random.random() < 0.02:
            random_spike = random.uniform(50, 200)  # 50-200ms spike
        
        total_jitter = base_jitter + sine_jitter + fast_jitter + random_spike
        self.add_timing_jitter(max(1.0, total_jitter), 3.0)
        
        # Publish noisy signal with trends
        trend = 0.1 * elapsed  # Slow upward trend
        seasonal = 10.0 * math.sin(elapsed * 0.05)  # Very slow seasonal pattern
        noise = np.random.normal(0, 2.0)
        
        msg = Float64()
        msg.data = trend + seasonal + noise
        self.noisy_signal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = DummyPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Simple dummy publisher stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()