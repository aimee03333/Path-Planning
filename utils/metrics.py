# utils/metrics.py
import time
import tracemalloc

def measure_performance(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        tracemalloc.start()
        result = func(*args, **kwargs)
        current, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        end_time = time.time()
        return result, end_time - start_time, peak / 1024
    return wrapper