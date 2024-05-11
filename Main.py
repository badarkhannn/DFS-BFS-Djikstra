from collections import deque
import heapq

class Graph:
    def __init__(self):
        self.graph = {}

    def add_edge(self, u, v, weight=1):
        if u not in self.graph:
            self.graph[u] = {}
        if v not in self.graph:
            self.graph[v] = {}
        self.graph[u][v] = weight

    def dfs(self, start):
        visited = set()
        stack = [start]

        while stack:
            vertex = stack.pop()
            if vertex not in visited:
                visited.add(vertex)
                stack.extend(self.graph[vertex] - visited)
        
        return visited

    def bfs(self, start):
        visited = set()
        queue = deque([start])

        while queue:
            vertex = queue.popleft()
            if vertex not in visited:
                visited.add(vertex)
                queue.extend(self.graph[vertex] - visited)
        
        return visited

    def dijkstra(self, start):
        distances = {vertex: float('infinity') for vertex in self.graph}
        distances[start] = 0
        pq = [(0, start)]

        while pq:
            current_distance, current_vertex = heapq.heappop(pq)

            if current_distance > distances[current_vertex]:
                continue

            for neighbor, weight in self.graph[current_vertex].items():
                distance = current_distance + weight

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    heapq.heappush(pq, (distance, neighbor))

        return distances

# Time complexities
time_complexities = {
    "Algorithm": ["Depth First Search (DFS)", "Breadth First Search (BFS)", "Dijkstra's Algorithm"],
    "Time Complexity": ["O(V + E)", "O(V + E)", "O((V + E) log V)"]
}

# Print time complexity table
print("Time Complexities:")
print("{:<30} {:<20}".format("Algorithm", "Time Complexity"))
for i in range(len(time_complexities["Algorithm"])):
    print("{:<30} {:<20}".format(time_complexities["Algorithm"][i], time_complexities["Time Complexity"][i]))
