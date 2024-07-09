import random
import heapq
import math

def generate_tsp_instance(num_cities):
    cities = [(random.uniform(0, 1), random.uniform(0, 1)) for _ in range(num_cities)]
    return cities

class DisjointSetUnion:
    def __init__(self, n):
        self.parent = list(range(n))
        self.rank = [0] * n

    def find(self, u):
        if self.parent[u] != u:
            self.parent[u] = self.find(self.parent[u])
        return self.parent[u]

    def union(self, u, v):
        root_u = self.find(u)
        root_v = self.find(v)
        if root_u != root_v:
            if self.rank[root_u] > self.rank[root_v]:
                self.parent[root_v] = root_u
            elif self.rank[root_u] < self.rank[root_v]:
                self.parent[root_u] = root_v
            else:
                self.parent[root_v] = root_u
                self.rank[root_u] += 1

def kruskal_mst(num_cities, edges):
    dsu = DisjointSetUnion(num_cities)
    edges.sort(key=lambda x: x[2])
    mst_cost = 0
    for u, v, weight in edges:
        if dsu.find(u) != dsu.find(v):
            dsu.union(u, v)
            mst_cost += weight
    return mst_cost

def euclidean_distance(city1, city2):
    return math.sqrt((city1[0] - city2[0])**2 + (city1[1] - city2[1])**2)

def generate_edges(cities):
    edges = []
    num_cities = len(cities)
    for i in range(num_cities):
        for j in range(i + 1, num_cities):
            distance = euclidean_distance(cities[i], cities[j])
            edges.append((i, j, distance))
    return edges

def mst_heuristic(cities, partial_tour):
    remaining_cities = [i for i in range(len(cities)) if i not in partial_tour]
    if not remaining_cities:
        return 0
    remaining_city_indices = {index: i for i, index in enumerate(remaining_cities)}
    edges = generate_edges([cities[i] for i in remaining_cities])
    mapped_edges = [(remaining_city_indices[u], remaining_city_indices[v], w) for u, v, w in edges]
    return kruskal_mst(len(remaining_cities), mapped_edges)

def a_star_tsp(cities):
    num_cities = len(cities)
    start = (0, tuple([0]))  # Starting city and partial tour
    pq = [(0, start)]
    heapq.heapify(pq)
    visited = set()

    while pq:
        cost, (current_cost, partial_tour) = heapq.heappop(pq)
        if len(partial_tour) == num_cities:
            return list(partial_tour) + [0]  # Returning to the starting city

        last_city = partial_tour[-1]
        if partial_tour in visited:
            continue
        visited.add(partial_tour)

        for next_city in range(num_cities):
            if next_city not in partial_tour:
                new_tour = partial_tour + (next_city,)
                new_cost = current_cost + euclidean_distance(cities[last_city], cities[next_city])
                estimated_cost = new_cost + mst_heuristic(cities, new_tour)
                heapq.heappush(pq, (estimated_cost, (new_cost, new_tour)))

    return []

# Example usage:
cities = generate_tsp_instance(5)
print(a_star_tsp(cities))
