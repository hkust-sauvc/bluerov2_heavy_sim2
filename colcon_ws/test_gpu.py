import pycuda.driver as cuda
import pycuda.autoinit

def test_gpu():
    device = cuda.Device(0)  # First GPU device
    print(f"Device Name: {device.name()}")
    print(f"Compute Capability: {device.compute_capability()}")
    print(f"Total Memory: {device.total_memory() / 1e9:.2f} GB")

test_gpu()