from IPython import embed
import argparse
import numpy
import networkx as nx

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='script for generating obstacle files')

    args = parser.parse_args()

    G = nx.Graph()

    default_vertex_location = 'vertices.txt'
    default_edges_location = 'edges.txt'
    edgeStates_location = 'edgesViz.txt'
    graph_save_location = 'graph_1000_feeding.graphml'


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
        if numpy.shape(haltonEdges)[1] == 2:
            source = numpy.array(edgeStates[i, 0:8])
            target = numpy.array(edgeStates[i, 8:])
            dist = numpy.linalg.norm(source - target)
        else:
            dist = haltonEdges[i][-1]
        G.add_edge(int(haltonEdges[i][0]), int(haltonEdges[i][1]), length = dist)

    print('Average degree: ', G.number_of_edges() * 2.0 / n)
    print('Connected: ', nx.is_connected(G))
    print('Number of Connected Components: ', nx.number_connected_components(G))
    nx.write_graphml(G, graph_save_location)
