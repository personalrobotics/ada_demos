from IPython import embed
import argparse
import numpy
import networkx as nx

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='script for generating obstacle files')

    args = parser.parse_args()

    G = nx.Graph()

    directory = "/home/herb/Workspace/demo_ws/devel/bin/"
    default_vertex_location = directory + 'vertices.txt'
    default_edges_location = directory + 'edges.txt'
    edgeStates_location = directory + 'edgesViz.txt'
    graph_save_location = directory + 'graph_1000_feeding.graphml'


    # Collect vertices.
    halton = numpy.loadtxt(default_vertex_location)
    n = numpy.shape(halton)[0]

    # Add vertices
    print("Adding vertices")
    for i in range(n):
        state = ''
        for j in range(6):
            state += str(halton[i][j]) + ' '
        state += str(halton[i][-1])

        G.add_node(i, state = state)

    # Collect edges.
    haltonEdges = numpy.loadtxt(default_edges_location)
    edgeStates = numpy.loadtxt(edgeStates_location)

    # Adding Edges
    print("Adding edges")
    for i in range(numpy.shape(haltonEdges)[0]):
        dist = haltonEdges[i][-1]
        G.add_edge(int(haltonEdges[i][0]), int(haltonEdges[i][1]), length = float(dist))

    print('Average degree: ', G.number_of_edges() * 2.0 / n)
    print('Connected: ', nx.is_connected(G))
    print('Number of Connected Components: ', nx.number_connected_components(G))
    nx.write_graphml(G, graph_save_location)
