#!/usr/bin/env python3
import heapq
import itertools


class AStarGridPlanner:
    def __init__(self, width: int, height: int, blocked: list[bool]):
        self.width = width
        self.height = height
        self.blocked = blocked

    def is_cell_free(self, mx: int, my: int) -> bool:
        if mx < 0 or my < 0 or mx >= self.width or my >= self.height:
            return False
        idx = my * self.width + mx
        return not self.blocked[idx]

    def find_nearest_free_cell(
        self,
        mx: int,
        my: int,
        max_radius_cells: int,
    ) -> tuple[int, int] | None:
        if self.is_cell_free(mx, my):
            return mx, my

        for radius in range(1, max_radius_cells + 1):
            x0 = mx - radius
            x1 = mx + radius
            y0 = my - radius
            y1 = my + radius

            for x in range(x0, x1 + 1):
                if self.is_cell_free(x, y0):
                    return x, y0
                if self.is_cell_free(x, y1):
                    return x, y1
            for y in range(y0 + 1, y1):
                if self.is_cell_free(x0, y):
                    return x0, y
                if self.is_cell_free(x1, y):
                    return x1, y

        return None

    @staticmethod
    def compress_path(cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
        if len(cells) <= 2:
            return cells

        compressed = [cells[0]]
        prev_dx = cells[1][0] - cells[0][0]
        prev_dy = cells[1][1] - cells[0][1]

        for i in range(1, len(cells) - 1):
            cur = cells[i]
            nxt = cells[i + 1]
            dx = nxt[0] - cur[0]
            dy = nxt[1] - cur[1]
            if dx != prev_dx or dy != prev_dy:
                compressed.append(cur)
            prev_dx = dx
            prev_dy = dy

        compressed.append(cells[-1])
        return compressed

    @staticmethod
    def path_turn_count(cells: list[tuple[int, int]]) -> int:
        if len(cells) <= 2:
            return 0

        turns = 0
        prev_dx = cells[1][0] - cells[0][0]
        prev_dy = cells[1][1] - cells[0][1]
        for i in range(1, len(cells) - 1):
            dx = cells[i + 1][0] - cells[i][0]
            dy = cells[i + 1][1] - cells[i][1]
            if dx != prev_dx or dy != prev_dy:
                turns += 1
            prev_dx = dx
            prev_dy = dy
        return turns

    def _plan_with_overlay(
        self,
        start_cell: tuple[int, int],
        goal_cell: tuple[int, int],
        reserved: set[tuple[int, int]],
        search_radius_cells: int,
        extra_blocked: set[tuple[int, int]] | None = None,
    ) -> list[tuple[int, int]] | None:
        sx, sy = start_cell
        gx, gy = goal_cell

        if start_cell in reserved:
            return None

        blocked_overlay = self.blocked[:]

        for rx, ry in reserved:
            if rx < 0 or ry < 0 or rx >= self.width or ry >= self.height:
                continue
            blocked_overlay[ry * self.width + rx] = True

        if extra_blocked:
            for bx, by in extra_blocked:
                if bx < 0 or by < 0 or bx >= self.width or by >= self.height:
                    continue
                blocked_overlay[by * self.width + bx] = True

        local_planner = AStarGridPlanner(self.width, self.height, blocked_overlay)

        if search_radius_cells > 0:
            start_free = local_planner.find_nearest_free_cell(sx, sy, search_radius_cells)
            goal_free = local_planner.find_nearest_free_cell(gx, gy, search_radius_cells)
        else:
            start_free = start_cell if local_planner.is_cell_free(sx, sy) else None
            goal_free = goal_cell if local_planner.is_cell_free(gx, gy) else None

        if start_free is None or goal_free is None:
            return None

        path = local_planner.plan(start_free, goal_free)
        if not path:
            return None

        if any(cell in reserved for cell in path):
            return None

        return path

    def _candidate_paths_for_request(
        self,
        start_cell: tuple[int, int],
        goal_cell: tuple[int, int],
        reserved: set[tuple[int, int]],
        search_radius_cells: int,
        max_candidates: int = 16,
    ) -> list[list[tuple[int, int]]]:
        base_path = self._plan_with_overlay(start_cell, goal_cell, reserved, search_radius_cells)
        if not base_path:
            return []

        unique: dict[tuple[tuple[int, int], ...], list[tuple[int, int]]] = {
            tuple(base_path): base_path
        }

        internal_cells = base_path[1:-1]
        for cell in internal_cells:
            if len(unique) >= max_candidates:
                break
            alt_path = self._plan_with_overlay(
                start_cell,
                goal_cell,
                reserved,
                search_radius_cells,
                extra_blocked={cell},
            )
            if not alt_path:
                continue
            unique.setdefault(tuple(alt_path), alt_path)

        # Sort candidates by same lexicographic objective: turns first, then steps.
        candidates = list(unique.values())
        candidates.sort(key=lambda p: (self.path_turn_count(p), max(0, len(p) - 1)))
        return candidates

    def plan_disjoint_paths(
        self,
        requests: list[tuple[tuple[int, int], tuple[int, int]]],
        search_radius_cells: int = 0,
    ) -> tuple[list[list[tuple[int, int]]] | None, list[int] | None]:
        """Plan multiple non-overlapping paths.

        Args:
            requests: list of (start_cell, goal_cell), one per robot.
            search_radius_cells: if >0, relocate blocked start/goal to nearest free cell.

        Returns:
            (paths, planning_order)
            - paths is indexed by the original request order.
            - planning_order is the successful planning sequence of request indices.
        """
        if not requests:
            return [], []

        robot_count = len(requests)
        indices = list(range(robot_count))

        best_paths: list[list[tuple[int, int]]] | None = None
        best_order: list[int] | None = None
        best_score: tuple[int, int] | None = None

        for order in itertools.permutations(indices):
            planned_paths: list[list[tuple[int, int]] | None] = [None] * robot_count

            def dfs(
                depth: int,
                reserved: set[tuple[int, int]],
                total_turns: int,
                total_steps: int,
            ):
                nonlocal best_paths, best_order, best_score

                if best_score is not None:
                    # Branch-and-bound: costs are monotonic increasing.
                    if total_turns > best_score[0]:
                        return
                    if total_turns == best_score[0] and total_steps >= best_score[1]:
                        return

                if depth >= robot_count:
                    score = (total_turns, total_steps)
                    if best_score is None or score < best_score:
                        candidate_paths: list[list[tuple[int, int]]] = []
                        for i in indices:
                            path_i = planned_paths[i]
                            if path_i is None:
                                return
                            candidate_paths.append(path_i)
                        best_score = score
                        best_order = list(order)
                        best_paths = candidate_paths
                    return

                idx = order[depth]
                start_cell, goal_cell = requests[idx]

                candidates = self._candidate_paths_for_request(
                    start_cell,
                    goal_cell,
                    reserved,
                    search_radius_cells,
                )
                if not candidates:
                    return

                for path in candidates:
                    if any(cell in reserved for cell in path):
                        continue

                    planned_paths[idx] = path
                    turns = self.path_turn_count(path)
                    steps = max(0, len(path) - 1)
                    next_reserved = set(reserved)
                    next_reserved.update(path)
                    dfs(depth + 1, next_reserved, total_turns + turns, total_steps + steps)
                    planned_paths[idx] = None

            dfs(0, set(), 0, 0)

        if best_paths is None or best_order is None:
            return None, None

        return best_paths, best_order

    def plan(
        self,
        start_cell: tuple[int, int],
        goal_cell: tuple[int, int],
    ) -> list[tuple[int, int]] | None:
        sx, sy = start_cell
        gx, gy = goal_cell
        start_idx = sy * self.width + sx
        goal_idx = gy * self.width + gx

        if start_idx == goal_idx:
            return [start_cell]

        # 4-neighbor motion; each move carries a direction index.
        neighbors = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
        ]

        # State = (cell_index, heading_dir).
        # heading_dir is -1 at start (no previous motion direction).
        start_state = (start_idx, -1)
        goal_state: tuple[int, int] | None = None

        # Lexicographic objective:
        #   1) minimize turn count
        #   2) then minimize total step count
        open_heap: list[tuple[int, int, int, int]] = []
        heapq.heappush(open_heap, (0, 0, start_idx, -1))
        best_cost: dict[tuple[int, int], tuple[int, int]] = {start_state: (0, 0)}
        came_from: dict[tuple[int, int], tuple[int, int]] = {}

        while open_heap:
            cur_turns, cur_steps, cur_idx, cur_dir = heapq.heappop(open_heap)
            cur_state = (cur_idx, cur_dir)

            if best_cost.get(cur_state) != (cur_turns, cur_steps):
                continue

            if cur_idx == goal_idx:
                goal_state = cur_state
                break

            cx = cur_idx % self.width
            cy = cur_idx // self.width

            for next_dir, (dx, dy) in enumerate(neighbors):
                nx = cx + dx
                ny = cy + dy
                if not self.is_cell_free(nx, ny):
                    continue

                n_idx = ny * self.width + nx
                turn_cost = 0 if cur_dir == -1 or cur_dir == next_dir else 1
                n_turns = cur_turns + turn_cost
                n_steps = cur_steps + 1
                n_state = (n_idx, next_dir)
                n_cost = (n_turns, n_steps)

                prev_best = best_cost.get(n_state)
                if prev_best is not None and n_cost >= prev_best:
                    continue

                best_cost[n_state] = n_cost
                came_from[n_state] = cur_state
                heapq.heappush(open_heap, (n_turns, n_steps, n_idx, next_dir))

        if goal_state is None:
            return None

        states = [goal_state]
        while states[-1] != start_state:
            parent = came_from.get(states[-1])
            if parent is None:
                return None
            states.append(parent)
        states.reverse()

        indices = [state[0] for state in states]

        return [(idx % self.width, idx // self.width) for idx in indices]