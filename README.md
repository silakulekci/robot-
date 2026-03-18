# robot-
# Robot Vacuum Explorer — Grid-Based SLAM Simulation

An interactive browser simulation of a robot vacuum that explores an unknown environment from scratch, builds a map by bumping into walls, and returns home when the entire area is mapped.

---

## Live Demo

Open `robot_exploration.html` in any browser — no installation required.

---

## What Real Robot Vacuums Use

Modern robot vacuums (Roomba, Roborock, etc.) rely on two core technologies:

**LiDAR (Light Detection and Ranging)**
A spinning laser tower on top of the robot fires thousands of laser pulses per second in every direction. By measuring the time each pulse takes to return, the robot builds a millimeter-accurate 2D map of its surroundings. This is the gold standard — highly accurate but expensive.

**V-SLAM (Visual SLAM)**
A camera points upward and tracks feature points on the ceiling and walls. As the robot moves, the angular shift of these reference points is used to estimate distance and position. Cheaper than LiDAR but sensitive to lighting conditions.

Both approaches run a **SLAM (Simultaneous Localization and Mapping)** algorithm — the robot solves two problems at the same time: *"Where am I?"* and *"What does the map look like?"*

For exploration, real vacuums use **Frontier-Based Exploration** — they continuously navigate toward the boundary between known free space and unknown territory until no boundary remains.

---

## What This Simulation Uses

We implement the same concepts with simplified, math-based equivalents suitable for a research prototype or academic study.

### Sensor — Ray Casting instead of LiDAR

Real LiDAR fires physical laser pulses. We simulate the same behavior mathematically: at every step the robot casts virtual rays in all directions within a fixed range. Each ray travels cell by cell using **Bresenham's Line Algorithm** until it hits a wall. Cells the ray passes through are marked as free; the cell where it stops is marked as occupied.

```
Robot → ray → ray → ray → [WALL] ✓
         ↓free  ↓free       ↓occupied
```

### Map — Occupancy Grid with Bayesian (Log-Odds) Updates

Instead of storing a simple 0/1 grid, each cell holds a **log-odds probability** of being occupied:

| Log-odds value | Meaning |
|---|---|
| 0.0 | Unknown (starting state) |
| Negative | Probably free |
| Positive | Probably occupied |

Every time a sensor reading hits a cell, the log-odds value is updated:
- Ray passed through → subtract 0.55 (more likely free)
- Ray hit wall → add 1.4 (more likely occupied)

This is **Bayesian filtering** — repeated measurements reduce uncertainty. A cell seen as a wall 5 times in a row becomes very confidently marked as a wall.

We simplified the full SLAM problem by assuming the robot's own position is always known. In a real system, localization (figuring out where you are) is itself a hard problem solved simultaneously with mapping.

### Exploration Strategy — Frontier-Based Exploration

This is implemented exactly as in real systems.

A **frontier** is any unknown cell that is adjacent to at least one known-free cell. The robot always navigates toward the nearest frontier using **BFS (Breadth-First Search)**. When there are no frontiers left, the entire reachable area has been mapped.

```
[ free ][ free ][ ??? ]   ← frontier = boundary between free and unknown
[ free ][ ROBOT][ ??? ]
[ free ][ free ][ ??? ]
```

### Return to Home — BFS Shortest Path

Once coverage reaches ~96%, the robot switches to return phase. It runs BFS on its own built map to find the shortest path back to the starting position — just like a real vacuum returning to its charging dock.

---

## Visual Guide

| Color | Meaning |
|---|---|
| Dark green circle | Robot |
| Light green square | Start / home point |
| Teal | Visited and cleaned |
| Light mint | Mapped free space |
| Gray | Unknown (not yet reached) |
| Orange | Frontier (next target) |
| Blue | Return path home |
| Dark | Wall / obstacle |

---

## Algorithm Flow

```
START
  │
  ▼
Scan sensors (ray casting) → update occupancy grid (Bayesian)
  │
  ▼
Find frontiers (unknown cells bordering free cells)
  │
  ├─ Frontiers exist? → BFS to nearest frontier → move one step → loop
  │
  └─ No frontiers / coverage ≥ 96%?
        │
        ▼
      BFS shortest path back to HOME → move step by step
        │
        ▼
      Arrived at HOME → DONE
```

---

## Controls

| Button | Action |
|---|---|
| Start / Pause | Begin or pause the simulation |
| Step | Advance exactly one step |
| Step x10 | Advance ten steps at once |
| New map | Generate a new random maze and reset |
| Speed slider | Control animation speed (1 = slowest, 9 = fastest) |

---

## Concepts Used

| Concept | Field | Used for |
|---|---|---|
| Occupancy Grid | Robotics / Probabilistic Mapping | Representing the map |
| Log-Odds Bayesian Update | Probability Theory | Reducing sensor noise |
| Bresenham's Line Algorithm | Computer Graphics | Ray casting through grid cells |
| Frontier-Based Exploration | Autonomous Navigation | Deciding where to go next |
| BFS (Breadth-First Search) | Graph Theory | Pathfinding to frontier and home |

---

## Built With

- HTML5 Canvas — rendering
- Vanilla JavaScript — all logic, no external libraries

---

## References

- Thrun, S., Burgard, W., Fox, D. — *Probabilistic Robotics* (2005)
- Yamauchi, B. — *A Frontier-Based Approach for Autonomous Exploration* (1997)
- Bresenham, J. — *Algorithm for computer control of a digital plotter* (1965)
