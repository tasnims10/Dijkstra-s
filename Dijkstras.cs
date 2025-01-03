using System;
using System.Collections.Generic;
using System.Linq;

class Graph
{
    public int Nodes { get; }
    public Dictionary<int, List<(int neighbor, double weight)>> AdjacencyList { get; }

    public Graph(int nodes)
    {
        //initializes a graph with nodes number of vertices
        Nodes = nodes;
        AdjacencyList = new Dictionary<int, List<(int neighbor, double weight)>>();//creates an empty adjacency list for all nodes
        for (int i = 0; i < nodes; i++)
        {
            AdjacencyList[i] = new List<(int neighbor, double weight)>();
        }
    }

    public void AddEdge(int u, int v, double weight)
    {
        // adds an edge between nodes u and v with the specified weight
        AdjacencyList[u].Add((v, weight));
        AdjacencyList[v].Add((u, weight)); // If the graph is undirected
    }

    public List<(int neighbor, double weight)> GetNeighbors(int node)
    {
        return AdjacencyList[node];//returns the list of of neighbours of a given node along with their edge weights 
    }

    public static double[] Dijkstra(Graph graph, int src)
        //implement dijkstra's  algorithm to compute the shortest distance from the source node to all other nodes in the graph
    {
        int n = graph.Nodes;
        double[] distances = new double[n];// array to tore the shortest distance of all nodes from source node 
        for (int i = 0; i < n; i++)
        {
            distances[i] = double.PositiveInfinity;//initially all are set to infinity, except source node 
        }
        distances[src] = 0;

        var priorityQueue = new SortedSet<(double, int)>();// maintains the nodes to be processed, sorted by distance 
        priorityQueue.Add((0, src));
        var visited = new HashSet<int>();//track nodes that have been visited to avoid revisiting them 

        while (priorityQueue.Count > 0)
        {
            //while the priority queue is not empty
            var currentNode = priorityQueue.Min;
            priorityQueue.Remove(currentNode);//dequeue the node with the smallest distance 

            double currentDistance = currentNode.Item1;
            int current = currentNode.Item2;

            if (visited.Contains(current))
                continue;//if the node has laready been visited, skip it

            visited.Add(current);//add the current node to the visited list

            foreach (var (neighbor, weight) in graph.GetNeighbors(current))
            {
                double newDistance = currentDistance + weight;//for each neighbour of the current node compute the distance

                if (newDistance < distances[neighbor])
                {
                    distances[neighbor] = newDistance;//if the distance is smaller to what was previously known, update with the new value and add the neighbour to the priority queue
                    priorityQueue.Add((newDistance, neighbor));
                }
            }
        }

        return distances;//returns the array of the shortest distances of all nodes to the source node
    }

    public double ComputeInfluenceScore(int u)
        //computes the influence score of node u 
    {
        double[] distances = Dijkstra(this, u);//calls Dijkstra's algorithm to get the shortest distances from u to all other nodes.
        double totalDistance = 0;//sums the shortest distance from u to all other nodes 

        for (int v = 0; v < Nodes; v++)
        {
            if (v != u)
            {
                totalDistance += distances[v];
            }
        }

        if (totalDistance > 0)
        {
            return (Nodes - 1) / totalDistance;
        }
        else
        {
            return 0;//no influence score if the node is isolated
        }
    }
}

class Program
{
    static void Main()
    {
        // Example graph usage
        Graph graph = new Graph(5);
        graph.AddEdge(0, 1, 1);
        graph.AddEdge(1, 2, 2);
        graph.AddEdge(2, 3, 1);
        graph.AddEdge(3, 4, 3);
        graph.AddEdge(0, 4, 10);

        int u = 0; // Node for which to calculate influence score
        double influenceScore = graph.ComputeInfluenceScore(u);
        Console.WriteLine($"Influence Score of node {u}: {influenceScore}");
    }
}
