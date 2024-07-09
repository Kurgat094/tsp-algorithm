import random

def generate_tsp_instance(n):
    """Generates a TSP instance with n cities represented by random points in the unit square."""
    cities = [(random.random(), random.random()) for _ in range(n)]
    return cities

# Example usage:
n = 10  # Number of cities
cities = generate_tsp_instance(n)
print(cities)
