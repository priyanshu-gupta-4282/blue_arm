import numpy as np
import cv2
from queue import PriorityQueue
from typing import List, Tuple
import json

class ImagePathfinder:
    def __init__(self, threshold: int = 127, buffer_size: int = 10, video_source: int | str = 0):
        self.original_img = self.capture_frame_from_video(video_source)
        self.gray_img = cv2.cvtColor(self.original_img, cv2.COLOR_BGR2GRAY)

        _, binary_img = cv2.threshold(self.gray_img, threshold, 255, cv2.THRESH_BINARY)
        binary_img = cv2.bitwise_not(binary_img)

        self.processed_grid = self.create_obstacle_boundaries(binary_img, buffer_size)
        self.rows, self.cols = self.processed_grid.shape

        self.show_boundaries()
        self.start, self.end = self.select_points()

    def capture_frame_from_video(self, source):
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            raise Exception("Cannot open video source.")

        print("Press SPACE to capture a frame for pathfinding. Press ESC to exit.")
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame.")
                break
            cv2.imshow("Video Feed - Press SPACE to capture", frame)
            key = cv2.waitKey(1)
            if key == 32:  # SPACE
                img = frame.copy()
                break
            elif key == 27:  # ESC
                cap.release()
                cv2.destroyAllWindows()
                raise Exception("User exited before capturing frame.")

        cap.release()
        cv2.destroyAllWindows()
        return img

    def create_obstacle_boundaries(self, binary_img, buffer_size):
        contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        grid = np.ones_like(binary_img)
        self.boundary_img = self.original_img.copy()
        min_contour_area = 100

        for cnt in contours:
            if cv2.contourArea(cnt) < min_contour_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            x1, y1 = max(0, x - buffer_size), max(0, y - buffer_size)
            x2, y2 = min(binary_img.shape[1], x + w + buffer_size), min(binary_img.shape[0], y + h + buffer_size)
            grid[y1:y2, x1:x2] = 0
            cv2.rectangle(self.boundary_img, (x1, y1), (x2, y2), (0, 0, 255), 2)

        return grid

    def show_boundaries(self):
        cv2.imshow("Obstacle Boundaries", self.boundary_img)
        cv2.imshow("Processed Grid", self.processed_grid * 255)
        print("Showing boundaries. Press any key to continue with point selection...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def find_nearest_valid_pixel(self, x, y):
        if self.processed_grid[y, x] == 1:
            return (y, x)
        for r in range(1, 50):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < self.rows and 0 <= nx < self.cols:
                        if self.processed_grid[ny, nx] == 1:
                            print(f"Snapped ({y},{x}) to nearest valid point ({ny},{nx})")
                            return (ny, nx)
        raise Exception("No valid point found near the selected location.")

    def select_points(self):
        start, end = None, None

        def on_mouse(event, x, y, flags, param):
            nonlocal start, end
            if event == cv2.EVENT_LBUTTONDOWN:
                if start is None:
                    start = self.find_nearest_valid_pixel(x, y)
                    cv2.circle(self.boundary_img, (start[1], start[0]), 5, (0, 255, 0), -1)
                elif end is None:
                    end = self.find_nearest_valid_pixel(x, y)
                    cv2.circle(self.boundary_img, (end[1], end[0]), 5, (0, 0, 255), -1)
                cv2.imshow('Select Points', self.boundary_img)

        cv2.imshow('Select Points', self.boundary_img)
        cv2.setMouseCallback('Select Points', on_mouse)
        print("Click to select start point (green), then end point (red)")
        while start is None or end is None:
            cv2.waitKey(1)
        cv2.destroyAllWindows()
        return start, end

    def get_neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        row, col = node
        directions = [(-1, 0), (1, 0),  (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        return [
            (row + dr, col + dc)
            for dr, dc in directions
            if 0 <= row + dr < self.rows and 0 <= col + dc < self.cols
            and self.processed_grid[row + dr, col + dc] == 1
        ]

    def manhattan_distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def find_path(self) -> List[Tuple[int, int]]:
        start, goal = self.start, self.end
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.manhattan_distance(start, goal)}
        open_set_hash = {start}

        while not open_set.empty():
            current = open_set.get()[1]
            open_set_hash.remove(current)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.manhattan_distance(neighbor, goal)
                    if neighbor not in open_set_hash:
                        open_set.put((f_score[neighbor], neighbor))
                        open_set_hash.add(neighbor)

        print("No path found.")
        return []

    def simplify_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        if not path:
            return []
        simplified = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i:
                if self.line_is_clear(path[i], path[j]):
                    simplified.append(path[j])
                    i = j
                    break
                j -= 1
            else:
                i += 1
                simplified.append(path[i])
        return simplified

    def line_is_clear(self, a: Tuple[int, int], b: Tuple[int, int]) -> bool:
        r1, c1 = a
        r2, c2 = b
        for i in range(1, max(abs(r2 - r1), abs(c2 - c1))):
            r = r1 + (r2 - r1) * i // max(abs(r2 - r1), abs(c2 - c1))
            c = c1 + (c2 - c1) * i // max(abs(r2 - r1), abs(c2 - c1))
            if self.processed_grid[r, c] == 0:
                return False
        return True

    def apply_offset(self, point: Tuple[int, int]) -> Tuple[int, int]:
        row, col = point
        u = row / (self.rows - 1)
        v = col / (self.cols - 1)

        X_top_left, Y_top_left = 70, -45
        X_bottom_left, Y_bottom_left = 150, -45
        X_top_right, Y_top_right = 70, 90
        X_bottom_right, Y_bottom_right = 145, 65

        X = (1 - u) * ((1 - v) * X_top_left + v * X_top_right) + u * ((1 - v) * X_bottom_left + v * X_bottom_right)
        Y = (1 - u) * ((1 - v) * Y_top_left + v * Y_top_right) + u * ((1 - v) * Y_bottom_left + v * Y_bottom_right)
        return (round(X), round(Y))

    def interpolate_full_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        full_path = []
        for i in range(len(path) - 1):
            r1, c1 = path[i]
            r2, c2 = path[i + 1]
            length = max(abs(r2 - r1), abs(c2 - c1))
            for t in range(length + 1):
                r = r1 + (r2 - r1) * t // length
                c = c1 + (c2 - c1) * t // length
                if not full_path or (r, c) != full_path[-1]:
                    full_path.append((r, c))
        return full_path

    def store_path_lines(self, path: List[Tuple[int, int]]) -> List[dict]:
        lines = []
        for i in range(len(path) - 1):
            start_world = self.apply_offset(path[i])
            end_world = self.apply_offset(path[i + 1])
            lines.append({'start': start_world, 'end': end_world})
        return lines

    def process_path(self, path: List[Tuple[int, int]]) -> List[dict]:
        simplified_path = self.simplify_path(path)
        result_img = self.original_img.copy()
        for i in range(len(simplified_path) - 1):
            pt1 = (simplified_path[i][1], simplified_path[i][0])
            pt2 = (simplified_path[i + 1][1], simplified_path[i + 1][0])
            cv2.line(result_img, pt1, pt2, (0, 255, 255), 2)
        cv2.imshow("Final Path", result_img)
        print("Press any key to close image.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        lines = self.store_path_lines(simplified_path)
        print("\n=== World Coordinate Path for Robotic Arm ===")
        for i, seg in enumerate(lines):
            print(f"Segment {i+1}: Move from {seg['start']} to {seg['end']}")
        return lines

    def save_dataset(self, lines, filename="path_dataset.json"):
        with open(filename, "w") as f:
            json.dump(lines, f)
        print(f"Path saved to {filename}")

if __name__ == "__main__":
    pathfinder = ImagePathfinder(buffer_size=10, video_source=0)  # Use "video.mp4" for file input
    path = pathfinder.find_path()
    if path:
        interpolated_path = pathfinder.interpolate_full_path(path)
        world_coords = [pathfinder.apply_offset(pix) for pix in interpolated_path]

        print("\n=== Continuous World Coordinates ===")
        for i, coord in enumerate(world_coords):
            print(f"{i+1}: {coord}")

        with open("continuous_path.txt", "w") as f:
            for coord in world_coords:
                f.write(f"{coord[0]}, {coord[1]}\n")
        print("Saved continuous path to continuous_path.txt")

        simplified_lines = pathfinder.process_path(path)
        pathfinder.save_dataset(simplified_lines)
    else:
        print("No valid path found.")
