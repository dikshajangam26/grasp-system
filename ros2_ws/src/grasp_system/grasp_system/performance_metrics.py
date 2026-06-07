import time
from contextlib import contextmanager
from collections import defaultdict

class PerformanceMonitor:
    def __init__(self):
        self.timings = defaultdict(list)
    
    @contextmanager
    def time_block(self, name):
        start = time.perf_counter()
        try:
            yield
        finally:
            elapsed = (time.perf_counter() - start) * 1000  # Convert to ms
            self.timings[name].append(elapsed)
    
    def get_stats(self, name):
        times = self.timings.get(name, [])
        if not times:
            return {'mean': 0, 'min': 0, 'max': 0, 'all': []}
            
        return {
            'mean': sum(times) / len(times),
            'min': min(times),
            'max': max(times),
            'all': times  # Add the raw list here
        }
    
    def print_report(self):
        for name, stats in self.get_stats().items():
            print(f"{name}: {stats['mean']:.2f}ms (min: {stats['min']:.2f}, max: {stats['max']:.2f})")