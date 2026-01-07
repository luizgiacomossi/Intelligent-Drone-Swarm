# market.py
import numpy as np
from config import STARTING_POINTS, SECTION_BASE_VALUE, FORCE_BUY_COST, DYNAMIC_PRICE_FACTOR

class MarketSystem:
    """
    Market system where drones buy unsearched sections and gain reward points
    equal to the section's worth once the section has been searched.
    """

    def __init__(self, num_drones, area, starting_points=STARTING_POINTS, base_value=SECTION_BASE_VALUE):
        self.num_drones = num_drones
        self.area = area
        self.base_value = base_value
        self.points = {i: starting_points for i in range(num_drones)}
        self.transactions = []  # market logs
        self.phase = "initial"  # 'initial' or 'dynamic'

        # Each section has its state
        self.sections = [
            {
                "id": s.id,
                "pos": np.array(s.position),
                "owner": None,
                "value": base_value,
                "available": True,
                "searched": False,
                "reallocated": False
            }
            for s in self.area.sections
        ]
        
        # Metrics state
        self.released_sections = {} # section_id -> timestamp when released
        self.reallocations = [] # list of (latency, section_id)
        self.costs_paid = [] # list of prices paid
        self.successful_recoveries = 0 # count of released sections that were finished
        
        # New Metrics
        self.cumulative_bids = 0
        self.total_auctions = 0 # count of auction events (per section)
        self.wasted_attempts = 0 # count of attempts to claim searched sections

    def compute_dynamic_price(self, drone_pos, section_pos):
        """Price increases with distance (only in dynamic phase)."""
        dist = np.linalg.norm(drone_pos[:2] - section_pos[:2])
        return round(self.base_value * (1 + dist / DYNAMIC_PRICE_FACTOR), 2)

    def force_buy_section(self, drone_id, cost=2):
        """
        Force the given drone to buy one random available section,
        ignoring distance and costing a specified point penalty.
        """
        available = [s for s in self.sections if not s["searched"] and s["owner"] is None]
        if not available:
            print(f"[Market] No available sections for forced purchase.")
            return False

        sec = np.random.choice(available)
        sec["owner"] = drone_id
        sec["available"] = False

        # Deduct penalty points
        self.points[drone_id] = self.points.get(drone_id, 0) - cost

        # Reflect in SearchArea
        for s in self.area.sections:
            if s.id == sec["id"]:
                s.assigned_drone = drone_id

        self.transactions.append(
            f"[Penalty] drone {drone_id} forcibly bought Section {sec['id']} for {cost} pts (late battery)."
        )
        print(f"[Market] Drone {drone_id} bought section {sec['id']} with penalty {cost} points.")
        return True


    def open_market(self, drone_positions):
        """
        Initial market phase: all sections cost the same.
        drones take turns buying until they run out of points.
        """
        print("Opening initial market...")
        self.phase = "initial"
        equal_price = self.base_value

        turn = 0
        while any(s["available"] and not s["searched"] for s in self.sections):
            drone_id = turn % self.num_drones
            affordable = [
                s for s in self.sections
                if s["available"] and not s["searched"] and self.points[drone_id] >= equal_price
            ]
            if not affordable:
                turn += 1
                if turn > self.num_drones * 2:
                    break
                continue

            chosen = np.random.choice(affordable)
            self.buy_section(drone_id, chosen["id"], equal_price)
            turn += 1

        self.transactions.append("Initial market completed (equal pricing).")

    # ---------------------------------------------------------------------
    def enable_dynamic_pricing(self):
        """Enable distance-based dynamic pricing."""
        if self.phase != "dynamic":
            self.phase = "dynamic"
            self.transactions.append("Market switched to dynamic pricing.")

    # ---------------------------------------------------------------------
    def dynamic_update(self, drone_positions, current_time=0):
        """Let drones buy newly available sections during mission."""
        if self.phase != "dynamic":
            self.enable_dynamic_pricing()

        bids = []
        for s in self.sections:
            if not s["available"] or s["searched"]:
                continue
            for drone_id, pos in enumerate(drone_positions):
                price = self.compute_dynamic_price(pos, s["pos"])
                price = self.compute_dynamic_price(pos, s["pos"])
                bids.append((s["id"], drone_id, price))
                self.cumulative_bids += 1
                
            self.total_auctions += 1

        bids.sort(key=lambda b: b[2])

        for section_id, drone_id, price in bids:
            sec = next(x for x in self.sections if x["id"] == section_id)
            if not sec["available"] or sec["searched"]:
                continue
            if self.points[drone_id] < price:
                continue
            self.buy_section(drone_id, section_id, price, current_time)

    # ---------------------------------------------------------------------
    def buy_section(self, drone_id, section_id, price=None, current_time=0):
        """Buy a section if available and enough points."""
        sec = next(s for s in self.sections if s["id"] == section_id)
        if not sec["available"] or sec["searched"]:
            return False
        if price is None:
            price = sec["value"]
        if self.points[drone_id] < price:
            return False

        sec["owner"] = drone_id
        sec["available"] = False
        sec["value"] = price
        self.points[drone_id] -= price
        self.costs_paid.append(price)
        
        if section_id in self.released_sections:
            latency = current_time - self.released_sections[section_id]
            self.reallocations.append(latency)
            del self.released_sections[section_id] # Claimed, remove from released pending
            sec["reallocated"] = True



        # Reflect in SearchArea
        for s in self.area.sections:
            if s.id == section_id:
                s.assigned_drone = drone_id

        self.transactions.append(
            f"drone {drone_id} bought Section {section_id} for {price:.2f} pts."
        )
        return True

    # ---------------------------------------------------------------------
    def reward_and_remove_section(self, drone_id, section_id):
        """
        Called when an drone completes searching a section.
        The drone earns reward points equal to the section's value.
        The section is removed permanently from the market.
        """
        sec = next(s for s in self.sections if s["id"] == section_id)
        if sec["searched"]:
            self.wasted_attempts += 1
            return False  # already done

        reward = sec["value"]
        self.points[drone_id] += reward
        sec["searched"] = True
        sec["available"] = False
        sec["owner"] = None
        
        if sec.get("reallocated", False):
            self.successful_recoveries += 1


        # Update SearchArea to reflect it's searched
        for s in self.area.sections:
            if s.id == section_id:
                s.searched = True
                s.assigned_drone = None

        self.transactions.append(
            f"🏁 drone {drone_id} completed Section {section_id}, earned {reward:.2f} pts (removed from market)."
        )
        return True

    # ---------------------------------------------------------------------
    def release_drone_sections(self, drone_id, current_time=0):
        """
        Called when an drone crashes or emergency lands.
        All sections that were owned (and not yet searched)
        become available again on the market.
        """
        for sec in self.sections:
                sec["owner"] = None
                sec["available"] = True
                sec["reallocated"] = False # Reset status until bought again
                self.released_sections[sec["id"]] = current_time

                # Reflect in SearchArea
                for s in self.area.sections:
                    if s.id == sec["id"]:
                        s.assigned_drone = None
                        s.searched = False
                self.transactions.append(
                    f"drone {drone_id} released Section {sec['id']} "
                    f"(available again after emergency landing)."
                )

    # ---------------------------------------------------------------------
    def get_market_status(self):
        """Formatted market status for GUI display."""
        lines = []
        lines.append(f"Market phase: {self.phase.upper()}\n")
        lines.append("drone Balances:")
        for a in range(self.num_drones):
            lines.append(f"  drone {a}: {self.points[a]:.2f} pts")

        lines.append("\nSection Ownership:")
        for s in self.sections:
            if s["searched"]:
                state = "✅ Done"
            elif not s["available"]:
                state = f"Owned by drone {s['owner']}"
            else:
                state = "Available"
            lines.append(f"  Section {s['id']}: {state} (Value {s['value']:.2f})")

        if self.transactions:
            lines.append("\nRecent Transactions:")
            lines += [f"  {t}" for t in self.transactions[-5:]]

        return "\n".join(lines)

    def get_metrics(self):
        avg_latency = np.mean(self.reallocations) if self.reallocations else 0.0
        avg_cost = np.mean(self.costs_paid) if self.costs_paid else 0.0
        return {
            "reallocation_time": avg_latency,
            "cost_efficiency": avg_cost,
            "recovery_count": self.successful_recoveries,
            "total_bids": self.cumulative_bids,
            "total_auctions": self.total_auctions,
            "wasted_attempts": self.wasted_attempts
        }
