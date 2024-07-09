import heapq

def prim_mst(graph):
    num_nodes = len(graph)
    visited = [False] * num_nodes
    min_heap = [(0, 0)]  # (cost, node)
    mst_cost = 0

    while min_heap:
        cost, node = heapq.heappop(min_heap)
        if visited[node]:
            continue
        visited[node] = True
        mst_cost += cost
        for neighbor, weight in graph[node]:
            if not visited[neighbor]:
                heapq.heappush(min_heap, (weight, neighbor))

    return mst_cost

# Example usage:
graph = [
    [(1, 1), (2, 3)],  # Edges from node 0
    [(0, 1), (2, 1)],  # Edges from node 1
    [(0, 3), (1, 1)]   # Edges from node 2
]
mst_cost = prim_mst(graph)
print(mst_cost)
