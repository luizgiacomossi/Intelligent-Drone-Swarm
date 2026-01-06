
import time

class MeasurementManager:
    def __init__(self, num_agents):
        self.num_agents = num_agents
        self.metrics = {
            "start_time": time.time(),
            "end_time": None,
            "success": False,
            "total_distance": 0.0,
            "reallocations": [], # list of (time, section_id)
            "sections_searched_per_drone": [0] * num_agents,
            "section_costs": [], # list of costs paid
            "failure_reason": "Timeout", # Default reason if mission ends without success
            "crashed_count": 0,
            "failed_count": 0
        }
        
    def start_mission(self):
        self.metrics["start_time"] = time.time()
        
    def end_mission(self, success=False, reason=None):
        self.metrics["end_time"] = time.time()
        self.metrics["success"] = success
        if reason:
            self.metrics["failure_reason"] = reason
            
    def update_distance(self, distance):
        self.metrics["total_distance"] += distance
        
    def increment_sections_searched(self, drone_id):
        if 0 <= drone_id < self.num_agents:
            self.metrics["sections_searched_per_drone"][drone_id] += 1
            
    def record_reallocation(self, current_time, section_id):
        self.metrics["reallocations"].append((current_time, section_id))
        
    def record_cost(self, cost):
        self.metrics["section_costs"].append(cost)
        
    def update_error_metrics(self, crashed_list, health_status, battery_late_set):
        self.metrics["crashed_count"] = sum(crashed_list)
        failed = 0
        for i in range(self.num_agents):
            if not crashed_list[i]:
                # Failed = Not crashed AND (Bad Health OR Late Battery)
                if health_status[i] != 0 or i in battery_late_set:
                    failed += 1
        self.metrics["failed_count"] = failed
        
    def merge_market_metrics(self, market_metrics):
        self.metrics.update(market_metrics)
        
    def get_metrics_summary(self):
        duration = 0
        if self.metrics["end_time"]:
            duration = self.metrics["end_time"] - self.metrics["start_time"]
        else:
            duration = time.time() - self.metrics["start_time"]
            
        return {
            "success": self.metrics["success"],
            "reason": self.metrics["failure_reason"],
            "duration": duration,
            "total_distance": self.metrics["total_distance"],
            "crashed": self.metrics["crashed_count"],
            "failed": self.metrics["failed_count"]
        }
